#!/usr/bin/env python3
import argparse
import rospy
from std_msgs.msg import Bool
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_ as LowStateGo2
from remote_controller import RemoteController, KeyMap
from dds_config import add_dds_arguments, initialize_channel, resolved_interface

class EmergencyStopMonitor:
    def __init__(self):
        rospy.init_node('go2_emergency_stop_monitor', anonymous=True)
        
        # 初始化遥控器
        self.remote_controller = RemoteController()
        
        # 初始化状态订阅 - 使用正确的话题名称
        self.low_state = unitree_go_msg_dds__LowState_()
        self.lowstate_subscriber = ChannelSubscriber("rt/lf/lowstate", LowStateGo2)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)
        
        # 发布急停信号
        self.emergency_stop_pub = rospy.Publisher('/emergency_stop', Bool, queue_size=1)
        
        # 状态变量
        self.last_button_state = 0
        self.debug_counter = 0
        
        rospy.loginfo("Go2 Emergency Stop Monitor initialized.")
        
        # 10Hz检查频率，足够响应急停
        self.timer = rospy.Timer(rospy.Duration(0.1), self.check_emergency_stop)
    
    def LowStateHandler(self, msg: LowStateGo2):
        """处理来自机器人的低级状态消息"""
        self.low_state = msg
        try:
            self.remote_controller.set(self.low_state.wireless_remote)
            # 每10次调用打印一次调试信息
            self.debug_counter += 1
            if self.debug_counter % 10 == 0:
                rospy.loginfo(f"Remote data received. Select button state: {self.remote_controller.button[KeyMap.select]}")
        except Exception as e:
            rospy.logerr(f"Failed to parse remote data: {e}")
    
    def check_emergency_stop(self, event):
        """检查急停按键"""
        try:
            current_button_state = self.remote_controller.button[KeyMap.B]
            
            # 每5秒打印一次按键状态用于调试
            if self.debug_counter % 50 == 0:
                rospy.loginfo(f"Select button state: {current_button_state}, Last state: {self.last_button_state}")
            
            # 检测按键按下事件（边沿触发）
            if current_button_state == 1 and self.last_button_state == 0:
                rospy.logwarn("EMERGENCY STOP TRIGGERED!")
                
                # 发布急停信号
                emergency_msg = Bool()
                emergency_msg.data = True
                self.emergency_stop_pub.publish(emergency_msg)
                rospy.loginfo("Emergency stop signal published to /emergency_stop topic")
                
                # 也可以直接调用系统shutdown
                rospy.signal_shutdown("Emergency stop requested by remote controller.")
            
            self.last_button_state = current_button_state
            
        except Exception as e:
            rospy.logwarn(f"Emergency stop check failed: {e}")

def parse_args():
    parser = argparse.ArgumentParser(description="Go2 remote emergency-stop monitor")
    add_dds_arguments(parser)
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


if __name__ == '__main__':
    try:
        args = parse_args()
        print(
            f"Initializing Go2 DDS: iface={resolved_interface(args)}, "
            f"peer={args.peer or 'none'}, domain={args.domain_id}"
        )
        initialize_channel(args)
        monitor = EmergencyStopMonitor()
        rospy.spin()
    except rospy.ROSInitException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Go2 Emergency stop monitor shutdown.") 
