#!/usr/bin/env python3
"""ROS 2 bridge for the quadruped worker; starts disarmed, never auto-enables."""
import argparse
import json
from pathlib import Path
import threading
import time
from urllib.request import Request, urlopen


class WorkerClient:
    def __init__(self, port, token_path):
        self.url=f'http://127.0.0.1:{port}'
        self.token=token_path.read_text().strip()
        self.boot_id=None

    def request(self,path,body=None):
        data=None if body is None else json.dumps(body,allow_nan=False).encode()
        req=Request(self.url+path,data=data,headers={'X-Omindos-Token':self.token,'Content-Type':'application/json'})
        with urlopen(req,timeout=1.8 if body and body.get('operation')=='arm' else .3) as response:
            return json.load(response)

    def status(self):
        status=self.request('/status')
        if self.boot_id is None: self.boot_id=status['boot_id']
        if self.boot_id != status['boot_id']: raise RuntimeError('worker restarted; restart ROS bridge')
        return status

    def command(self,operation,**kwargs):
        return self.request('/command',{'boot_id':self.boot_id,'sent_at':time.time(),
                                        'operation':operation,**kwargs})


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import TwistStamped
    from sensor_msgs.msg import JointState, Imu
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',type=int,default=18097)
    parser.add_argument('--token-file',type=Path,required=True)
    args,ros_args=parser.parse_known_args()
    client=WorkerClient(args.port,args.token_file)
    initial=client.status()
    rclpy.init(args=ros_args)

    class Bridge(Node):
        def __init__(self):
            super().__init__('omindos_quadruped')
            self.last_velocity_stamp=-1.; self.last_imu_stamp=-1.
            self.rejects=0; self.last_error=''
            self.last_target_key=None
            self.velocity_lock=threading.Lock(); self.imu_lock=threading.Lock()
            self.services_group=MutuallyExclusiveCallbackGroup()
            self.inputs_group=ReentrantCallbackGroup()
            self.status_pub=self.create_publisher(String,'/omindos_quadruped/status',10)
            # Read-only diagnostic copy. Preserve source time/session/sequence.
            # This 50 Hz topic is not a hardware command transport.
            self.targets=self.create_publisher(String,'/omindos_quadruped/joint_targets',10)
            self.joints=self.create_publisher(JointState,'/omindos_quadruped/joint_states',10)
            self.imu_pub=(self.create_publisher(Imu,'/omindos_quadruped/imu',10)
                          if initial['backend']=='mujoco' else None)
            self.create_subscription(TwistStamped,'/omindos_quadruped/cmd_vel',self.velocity,
                                     qos_profile_sensor_data,callback_group=self.inputs_group)
            if not self.imu_pub:
                self.create_subscription(Imu,'/omindos_quadruped/imu',self.imu,
                                         qos_profile_sensor_data,callback_group=self.inputs_group)
            self.command_services=[]
            for op in ('arm','stand','walk','stop','disarm','estop'):
                self.command_services.append(self.create_service(Trigger,'/omindos_quadruped/'+op,
                    self.service(op),callback_group=self.services_group))
            self.create_timer(.02,self.publish)

        def service(self,op):
            def callback(request,response):
                try:
                    result=client.command(op)
                    response.success=True; response.message=json.dumps(result)
                except Exception as error:
                    response.success=False; response.message=str(error)
                return response
            return callback

        def valid_stamp(self,header,last):
            stamp=header.stamp.sec+header.stamp.nanosec/1e9
            now=self.get_clock().now().nanoseconds/1e9
            if header.frame_id!='base_link' or not 0 <= now-stamp <= .15 or stamp<=last:
                raise ValueError('stale_replayed_or_wrong_frame')
            return stamp

        def velocity(self,message):
            with self.velocity_lock:
                try:
                    stamp=self.valid_stamp(message.header,self.last_velocity_stamp)
                    t=message.twist
                    if any(v!=0 for v in [t.linear.z,t.angular.x,t.angular.y]):
                        raise ValueError('unsupported_velocity_axes')
                    client.command('velocity',velocity=[t.linear.x,t.linear.y,t.angular.z],
                                   sample_age_s=max(0.,self.get_clock().now().nanoseconds/1e9-stamp))
                    self.last_velocity_stamp=stamp
                except Exception as error:
                    self.rejects+=1; self.last_error=str(error)

        def imu(self,message):
            with self.imu_lock:
                try:
                    stamp=self.valid_stamp(message.header,self.last_imu_stamp)
                    if message.orientation_covariance[0]<0 or message.angular_velocity_covariance[0]<0:
                        raise ValueError('IMU_orientation_or_gyro_unavailable')
                    q,g=message.orientation,message.angular_velocity
                    client.command('imu',quaternion=[q.x,q.y,q.z,q.w],gyro=[g.x,g.y,g.z],
                                   sample_age_s=max(0.,self.get_clock().now().nanoseconds/1e9-stamp))
                    self.last_imu_stamp=stamp
                except Exception as error:
                    self.rejects+=1; self.last_error=str(error)

        def publish(self):
            try:
                status=client.status()
                if time.monotonic()-status['monotonic_time']>.15:
                    raise RuntimeError('worker_status_stale')
                stamp=self.get_clock().now().to_msg()
                command=status.get('joint_command')
                key=(command['session_id'],command['sequence']) if command else None
                if command and key!=self.last_target_key and 0 <= time.monotonic()-command['issued_monotonic_s'] <= command['valid_for_s']:
                    target_message=String(); target_message.data=json.dumps(command,allow_nan=False)
                    self.targets.publish(target_message)
                    self.last_target_key=key
                if len(status['q'])==12 and status.get('sensor_age_s',1.) <= .1:
                    joints=JointState(); joints.header.stamp=stamp
                    joints.name=status['joint_names']; joints.position=status['q']; joints.velocity=status['dq']
                    self.joints.publish(joints)
                    if self.imu_pub:
                        imu=Imu(); imu.header.stamp=stamp; imu.header.frame_id='base_link'
                        imu.orientation.x,imu.orientation.y,imu.orientation.z,imu.orientation.w=status['quaternion_xyzw']
                        imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z=status['gyro_body']
                        imu.linear_acceleration_covariance[0]=-1.
                        self.imu_pub.publish(imu)
                status['rejected_inputs']=self.rejects; status['last_input_error']=self.last_error
            except Exception as error:
                status={'state':'BRIDGE_ERROR','reason':str(error),'hardware_validated':False}
            msg=String(); msg.data=json.dumps(status,allow_nan=False); self.status_pub.publish(msg)

    node=Bridge(); executor=MultiThreadedExecutor(num_threads=4); executor.add_node(node)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally:
        try: client.command('stop')  # preserve standing balance on normal bridge exit
        except Exception: pass
        executor.shutdown(); node.destroy_node(); rclpy.shutdown()


if __name__=='__main__': main()
