#!/usr/bin/env python3
import argparse
import time
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

class RemoteTest:
    def __init__(self, topic):
        # 初始化遥控器
        self.remote_controller = RemoteController()
        
        # 初始化状态订阅
        self.low_state = unitree_go_msg_dds__LowState_()
        self.lowstate_subscriber = ChannelSubscriber(topic, LowStateGo2)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)
        
        print(f"Go2 Remote Test initialized. topic={topic}")
        print("Press Select button on remote to test...")
    
    def LowStateHandler(self, msg: LowStateGo2):
        """处理来自机器人的低级状态消息"""
        self.low_state = msg
        try:
            # 打印原始数据
            print(f"Raw wireless_remote data: {list(msg.wireless_remote)}")
            
            self.remote_controller.set(msg.wireless_remote)
            
            # 打印按键状态
            print(f"Select button: {self.remote_controller.button[KeyMap.select]}")
            print(f"All buttons: {self.remote_controller.button}")
            print(f"Joystick: Lx={self.remote_controller.lx:.2f}, Ly={self.remote_controller.ly:.2f}, Rx={self.remote_controller.rx:.2f}, Ry={self.remote_controller.ry:.2f}")
            print("-" * 50)
            
        except Exception as e:
            print(f"Error parsing remote data: {e}")

def parse_args():
    parser = argparse.ArgumentParser(description="Read-only Go2 remote/lowstate DDS test")
    add_dds_arguments(parser)
    parser.add_argument(
        "--topic",
        default="rt/lf/lowstate",
        help="LowState DDS topic to subscribe (default: rt/lf/lowstate)",
    )
    return parser.parse_args()


if __name__ == '__main__':
    try:
        args = parse_args()
        print(
            f"Initializing Go2 DDS: iface={resolved_interface(args)}, "
            f"peer={args.peer or 'none'}, domain={args.domain_id}"
        )
        initialize_channel(args)
        test = RemoteTest(args.topic)
        
        print("Press Ctrl+C to stop...")
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("Test stopped.") 
