import json, os, pathlib, socket, subprocess, sys, tempfile, time, urllib.request, urllib.error
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
ROOT=pathlib.Path(os.environ.get('WORKBENCH_PACKAGE','/verification'))
def port():
    with socket.socket() as s:s.bind(('127.0.0.1',0));return s.getsockname()[1]
def main():
    checks=[]; processes=[]
    with tempfile.TemporaryDirectory() as folder:
        rport,dport=port(),port();base=f'http://127.0.0.1:{dport}'
        logfile=open(pathlib.Path(folder)/'services.log','w')
        def launch(args):
            p=subprocess.Popen(args,stdout=logfile,stderr=subprocess.STDOUT);processes.append(p);return p
        def request(path,data=None,expected=200):
            req=urllib.request.Request(base+path,data=None if data is None else json.dumps(data).encode(),headers={'Content-Type':'application/json'})
            try:response=urllib.request.urlopen(req,timeout=8)
            except urllib.error.HTTPError as e:response=e
            with response:
                value=json.load(response);assert response.status==expected,(path,response.status,value);return value
        rclpy.init();node=Node('quadruped_split_acceptance');joint=[];desc=[];status=[]
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(JointState,'/omindos_workbench/joint_states',joint.append,10)
        node.create_subscription(String,'/omindos_workbench/robot_description',lambda m:desc.append(m.data),qos)
        node.create_subscription(String,'/omindos_workbench/runtime_status',lambda m:status.append(json.loads(m.data)),qos)
        def spin(condition,heartbeat=None,timeout=8):
            until=time.monotonic()+timeout;beat=0
            while time.monotonic()<until:
                rclpy.spin_once(node,timeout_sec=.03)
                if heartbeat and time.monotonic()-beat>.25:
                    request('/v1/runtime/heartbeat',{'lease_token':heartbeat});beat=time.monotonic()
                if condition():return
            raise AssertionError('ROS observation timed out')
        try:
            runtime=launch(['/usr/local/bin/omindos-runtime','profile-runtime','--store',folder,'--port',str(rport)])
            launch([str(ROOT/'OmindOS-Workbench'),'--no-browser','--port',str(dport),'--store',folder,'--runtime-url',f'http://127.0.0.1:{rport}'])
            deadline=time.monotonic()+20
            while True:
                assert all(p.poll() is None for p in processes),pathlib.Path(folder,'services.log').read_text()
                try:
                    if request('/v1/runtime/status')['state']=='UNLOADED':break
                except (OSError,AssertionError):
                    if time.monotonic()>deadline:raise
                    time.sleep(.1)
            checks.append('desktop_connects_to_released_compiled_ros2_runtime')
            profile=json.loads((ROOT/'config/robot_profiles/workbench-demo.json').read_text())
            applied=request('/v1/workbench-active-profile',profile)
            ident=request('/v1/robot-profiles',profile,expected=201)['profile_id']
            loaded=request('/v1/runtime/load',{'profile_id':ident})
            token=loaded['lease_token'];assert loaded['runtime_applied'] and not loaded['hardware_active']
            spin(lambda:joint and desc and status)
            assert desc[-1]==profile['kinematics']['urdf_xml']
            assert set(joint[-1].name)==set(profile['dynamics']['joints'])
            checks.append('shared_profile_store_loads_quadruped_model_and_publishes_dds')
            late=[];node.create_subscription(String,'/omindos_workbench/robot_description',lambda m:late.append(m.data),qos)
            spin(lambda:late);assert late[-1]==desc[-1]
            checks.append('late_subscriber_receives_robot_description')
            target=loaded['pose']['positions'].copy();target['fl_hip']=.2
            run={'profile_id':ident,'lease_token':token,'target':target,'duration_s':2}
            request('/v1/runtime/run',run)
            spin(lambda:joint[-1].position[joint[-1].name.index('fl_hip')]>.025,heartbeat=token)
            checks.append('desktop_run_and_heartbeat_produce_named_joint_motion_on_dds')
            stop=request('/v1/workbench-stop',{'engaged':True});assert stop['ros_preview_stop_confirmed']
            held=request('/v1/runtime/status')['pose']['positions']['fl_hip'];mark=len(joint)
            spin(lambda:len(joint)>mark+6)
            idx=joint[-1].name.index('fl_hip');assert abs(joint[-1].position[idx]-held)<1e-9 and joint[-1].velocity[idx]==0
            request('/v1/runtime/run',run,expected=423)
            checks.append('desktop_estop_stops_real_ros_node_and_dds_pose_holds')
            request('/v1/workbench-stop',{'engaged':False,'expected_epoch':stop['motion_epoch']})
            assert request('/v1/runtime/status')['state']=='STOPPED'
            checks.append('manual_reset_never_resumes_ros_motion')
            request('/v1/runtime/run',run)
            spin(lambda:status and status[-1]['reason']=='preview_client_heartbeat_lost',timeout=5)
            request('/v1/runtime/heartbeat',{'lease_token':token})
            assert request('/v1/runtime/status')['state']=='STOPPED'
            checks.append('lost_heartbeat_stops_real_node_without_automatic_resume')
            topics=dict(node.get_topic_names_and_types())
            assert '/cmd_vel' not in topics and '/joint_trajectory_controller/joint_trajectory' not in topics
            checks.append('no_hardware_command_topics_created')
            runtime.terminate();runtime.wait(8)
            request('/v1/runtime/status',expected=503)
            request('/v1/workbench-active-profile')
            checks.append('runtime_exit_reported_while_local_configuration_remains_available')
            print(json.dumps({'status':'PASS','checks':checks,'ros_distro':os.environ.get('ROS_DISTRO'),'desktop_source_commit':'80cd7d907a97e6b3bdc35691fe5dcc3be5849a52','runtime_image':'omindos/control-size-candidate:20261009','native_ros_validation':True,'desktop_compiled_binary':True,'hardware_tested':False},indent=2),flush=True)
        finally:
            for p in processes:
                if p.poll() is None:p.terminate();p.wait(8)
            node.destroy_node();rclpy.shutdown();logfile.close()
if __name__=='__main__':main()
