#!/usr/bin/env python3
"""Real ROS 2 DDS -> worker -> floating-base dynamics acceptance. Simulation only."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    rclpy.init(); node=Node('check_quadruped_ros2')
    status={}; counts={'status':0,'joint_state':0,'imu':0,'joint_targets':0}; checks={}
    target_sequences=[]; target_session=None; invalid_targets=[]
    def received(message):
        status.clear(); status.update(json.loads(message.data)); counts['status']+=1
    def joints(message):
        if len(message.name)==12 and len(message.position)==12: counts['joint_state']+=1
    def imu(message):
        if message.header.frame_id=='base_link': counts['imu']+=1
    def targets(message):
        nonlocal target_session
        try:
            packet=json.loads(message.data)
            if packet['schema']!='omindos.joint_position.v1' or packet['unit']!='rad':
                raise ValueError('schema/unit')
            if packet['joint_names']!=status.get('joint_names') or len(packet['position'])!=12:
                raise ValueError('joint names/count')
            if not all(math.isfinite(x) for x in packet['position']): raise ValueError('nonfinite')
            if target_session is None: target_session=packet['session_id']
            if packet['session_id']!=target_session: raise ValueError('session changed')
            if target_sequences and packet['sequence']<=target_sequences[-1]: raise ValueError('replay')
            if time.monotonic()-packet['issued_monotonic_s']>.1: raise ValueError('old target')
            target_sequences.append(packet['sequence']); counts['joint_targets']+=1
        except Exception as error:
            invalid_targets.append(str(error))
    subscriptions=[node.create_subscription(String,'/omindos_quadruped/status',received,10),
                   node.create_subscription(JointState,'/omindos_quadruped/joint_states',joints,10),
                   node.create_subscription(Imu,'/omindos_quadruped/imu',imu,10),
                   node.create_subscription(String,'/omindos_quadruped/joint_targets',targets,10)]
    publisher=node.create_publisher(TwistStamped,'/omindos_quadruped/cmd_vel',10)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end: rclpy.spin_once(node,timeout_sec=.02)
    def wait(predicate,timeout=5):
        end=time.monotonic()+timeout
        while not predicate():
            if time.monotonic()>end: raise AssertionError(f'timeout: {status}')
            rclpy.spin_once(node,timeout_sec=.02)
    def check(name,condition):
        checks[name]=bool(condition)
        if not condition: raise AssertionError(f'{name}: {status}')
    def action(op):
        client=node.create_client(Trigger,'/omindos_quadruped/'+op)
        if not client.wait_for_service(timeout_sec=5): raise AssertionError('service absent: '+op)
        future=client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node,future,timeout_sec=3)
        if not future.done(): raise AssertionError('service timeout: '+op)
        response=future.result(); node.destroy_client(client)
        return response
    def velocity(x=0.,yaw=0.,stale=False):
        message=TwistStamped(); message.header.frame_id='base_link'
        message.header.stamp=node.get_clock().now().to_msg()
        if stale: message.header.stamp.sec-=5
        message.twist.linear.x=x; message.twist.angular.z=yaw; publisher.publish(message)
    try:
        wait(lambda:'backend' in status)
        check('simulation_only_disarmed',status['backend']=='mujoco' and status['state']=='DISARMED'
              and status['hardware_active'] is False)
        check('walk_before_arm_rejected',not action('walk').success)
        check('default_joint_position_interface',status.get('control_mode')=='joint_position'
              and status.get('joint_command') is None and counts['joint_targets']==0)
        check('explicit_arm',action('arm').success)
        check('explicit_stand',action('stand').success)
        wait(lambda:status.get('state')=='HOLD',6)
        check('standing_height',status['base_xyz'][2]>.2)
        rejects=status['rejected_inputs']
        velocity(.2,stale=True); spin(.2)
        check('old_ros_stamp_rejected',status['rejected_inputs']>rejects)
        check('explicit_walk',action('walk').success)
        start=status['base_xyz'][0]
        min_upright=1.; end=time.monotonic()+6
        while time.monotonic()<end:
            velocity(.2); spin(.05)
            min_upright=min(min_upright,status.get('upright',0.))
            if status.get('state')=='FAULT': raise AssertionError(status)
        displacement=status['base_xyz'][0]-start
        check('ROS_forward_motion',displacement>.3 and min_upright>.8)
        wait(lambda:status.get('reason')=='velocity_command_timeout',2)
        spin(3)
        check('loss_of_command_balanced_hold',status['state']=='HOLD' and
              math.hypot(*status['body_velocity'][:2])<.15 and status['upright']>.8)
        rejects=status['rejected_inputs']; velocity(.2); spin(.2)
        check('fresh_command_does_not_auto_resume',status['state']=='HOLD' and status['rejected_inputs']>rejects)
        check('explicit_resume',action('walk').success)
        for _ in range(20): velocity(0.,.3); spin(.05)
        check('operator_stop',action('stop').success)
        wait(lambda:status.get('reason')=='operator_zero_velocity_hold')
        check('estop',action('estop').success)
        wait(lambda:status.get('state')=='FAULT')
        check('fault_latched',not action('arm').success)
        spin(.2); stopped_count=counts['joint_targets']; spin(.2)
        check('no_target_stream_after_estop',counts['joint_targets']==stopped_count and status.get('joint_command') is None)
        check('named_angle_stream_without_replay',not invalid_targets and len(target_sequences)>100)
        check('real_DDS_sensor_streams',min(counts.values())>100)
        report={'status':'PASS','checks':checks,'counts':counts,
                'forward_displacement_m':displacement,'minimum_upright_while_walking':min_upright,
                'hardware_tested':False,'customer_RS04_dynamics_tested':False,
                'model':status.get('model'),'mass_sensitivity':status.get('mass_sensitivity'),
                'control_mode':status.get('control_mode'),'joint_interface':status.get('joint_interface'),
                'target_sequence_first':target_sequences[0] if target_sequences else None,
                'target_sequence_last':target_sequences[-1] if target_sequences else None,
                'invalid_targets':invalid_targets,
                'control_profile_sha256':status.get('control_profile_sha256'),
                'transport':status.get('transport'),'simulated_CAN_tx_frames':status.get('simulated_CAN_tx_frames'),
                'scope':'ROS2_Humble_DDS_loopback_worker_pinned_MEVIUS2_MuJoCo'}
    except Exception as error:
        report={'status':'FAIL','checks':checks,'counts':counts,'error':str(error),'last_status':status}
    finally:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2),flush=True)
        node.destroy_node(); rclpy.shutdown()
    return 0 if report['status']=='PASS' else 1


if __name__=='__main__': raise SystemExit(main())
