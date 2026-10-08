"""Real ROS 2 DDS and HTTP acceptance; isolated domain, synthetic robot, no hardware."""
import copy
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from robot_parameter_api import register_robot_parameters

ROOT=Path(__file__).resolve().parents[1]


def unused_port():
    with socket.socket() as s:
        s.bind(('127.0.0.1',0)); return s.getsockname()[1]


def main():
    checks=[]
    with tempfile.TemporaryDirectory(prefix='omindos-ros-profile-') as directory:
        port=unused_port(); base=f'http://127.0.0.1:{port}'
        profile=json.loads((ROOT/'config/robot_profiles/workbench-demo.json').read_text())
        ident=register_robot_parameters(profile,directory)['profile_id']
        runtime=None; logfile=open(Path(directory)/'runtime.log','w')
        rclpy.init(); node=Node('profile_runtime_acceptance')
        descriptions=[]; joint_messages=[]; status_messages=[]
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(String,'/omindos_workbench/robot_description',lambda m:descriptions.append(m.data),qos)
        node.create_subscription(String,'/omindos_workbench/runtime_status',lambda m:status_messages.append(json.loads(m.data)),qos)
        node.create_subscription(JointState,'/omindos_workbench/joint_states',lambda m:joint_messages.append(m),10)

        def request(op,data=None,expected=200):
            req=urllib.request.Request(base+'/v1/runtime/'+op,data=None if data is None else json.dumps(data).encode(),headers={} if data is None else {'Content-Type':'application/json'})
            try: response=urllib.request.urlopen(req,timeout=8)
            except urllib.error.HTTPError as e: response=e
            with response:
                body=json.load(response)
                assert response.status==expected,(op,response.status,body)
                return body

        def spin_until(condition,timeout=8,heartbeat=None):
            deadline=time.monotonic()+timeout; last_beat=0
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.03)
                if heartbeat and time.monotonic()-last_beat>.3:
                    request('heartbeat',{'lease_token':heartbeat}); last_beat=time.monotonic()
                if condition(): return
            raise AssertionError('ROS observation timed out')

        def launch():
            process=subprocess.Popen([sys.executable,str(ROOT/'scripts/robot_profile_runtime.py'),'--store',directory,'--port',str(port)],stdout=logfile,stderr=subprocess.STDOUT)
            deadline=time.monotonic()+10
            while time.monotonic()<deadline:
                assert process.poll() is None,'runtime exited'
                try:
                    if request('status')['state']=='UNLOADED': return process
                except (OSError,AssertionError): time.sleep(.1)
            process.terminate();process.wait(5);raise AssertionError('runtime startup timeout')

        try:
            runtime=launch();checks.append('restart_requires_explicit_profile_load')
            result=request('load',{'profile_id':ident}); token=result['lease_token']
            assert result['runtime_applied'] and not result['hardware_active']
            spin_until(lambda:descriptions and joint_messages and status_messages)
            assert descriptions[-1]==profile['kinematics']['urdf_xml']
            assert set(joint_messages[-1].name)==set(profile['dynamics']['joints'])
            checks.append('profile_loaded_into_real_ros2_node_and_joint_state_published')
            late=[]
            node.create_subscription(String,'/omindos_workbench/robot_description',lambda m:late.append(m.data),qos)
            spin_until(lambda:bool(late));assert late[-1]==profile['kinematics']['urdf_xml']
            checks.append('late_subscriber_receives_transient_local_robot_description')
            target=result['pose']['positions'].copy();target['left_elbow']=.4
            request('run',{'profile_id':ident,'lease_token':token,'target':target,'duration_s':2})
            def moved():
                m=joint_messages[-1];return m.position[m.name.index('left_elbow')]>.025
            spin_until(moved,heartbeat=token)
            request('load',{'profile_id':ident},expected=409)
            checks.append('named_joint_trajectory_reaches_ros_topic_and_active_reload_rejected')
            stopped=request('stop',{'lease_token':token})
            hold=stopped['pose']['positions']['left_elbow'];mark=len(joint_messages)
            spin_until(lambda:len(joint_messages)>mark+6)
            last=joint_messages[-1];idx=last.name.index('left_elbow')
            assert abs(last.position[idx]-hold)<1e-9 and last.velocity[idx]==0
            checks.append('stop_command_holds_last_ros_joint_position')
            request('run',{'profile_id':ident,'lease_token':token,'target':target,'duration_s':4})
            spin_until(lambda:bool(status_messages) and status_messages[-1]['reason']=='preview_client_heartbeat_lost',timeout=5)
            request('heartbeat',{'lease_token':token});assert request('status')['state']=='STOPPED'
            checks.append('lost_client_stops_preview_without_automatic_resume')
            second=request('load',{'profile_id':ident})
            request('stop',{'lease_token':token},expected=409)
            assert second['lease_token']!=token
            checks.append('stale_session_cannot_control_new_loaded_version')
            topics=dict(node.get_topic_names_and_types())
            assert '/cmd_vel' not in topics and '/joint_trajectory_controller/joint_trajectory' not in topics
            checks.append('no_hardware_command_topics_created')
            runtime.terminate();runtime.wait(8);runtime=launch()
            assert request('status')['profile_id'] is None
            checks.append('process_restart_does_not_restore_activation')
            print(json.dumps({'status':'PASS','checks':checks,'ros_distro':os.environ.get('ROS_DISTRO'),
                              'runtime_target':'ros2_joint_preview','hardware_tested':False},indent=2))
        finally:
            if runtime and runtime.poll() is None:runtime.terminate();runtime.wait(8)
            node.destroy_node();rclpy.shutdown();logfile.close()


if __name__=='__main__':main()
