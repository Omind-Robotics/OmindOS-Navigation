#!/usr/bin/env python3
import struct

class KeyMap:
    """按键映射"""
    A = 0
    B = 1
    X = 2
    Y = 3
    L1 = 4
    R1 = 5
    L2 = 6
    R2 = 7
    select = 8
    start = 9
    F1 = 10
    F3 = 11
    up = 12
    right = 13
    down = 14
    left = 15

class RemoteController:
    def __init__(self):
        self.lx = 0
        self.ly = 0
        self.rx = 0
        self.ry = 0
        self.button = [0] * 16

    def set(self, data):
        # 使用官方示例的解析方法
        self.parse_key(data)
        self.parse_button(data[2], data[3])
    
    def parse_button(self, data1, data2):
        """解析按键数据"""
        # 第一个字节的按键
        self.button[KeyMap.R1] = (data1 >> 0) & 1
        self.button[KeyMap.L1] = (data1 >> 1) & 1
        self.button[KeyMap.start] = (data1 >> 2) & 1
        self.button[KeyMap.select] = (data1 >> 3) & 1
        self.button[KeyMap.R2] = (data1 >> 4) & 1
        self.button[KeyMap.L2] = (data1 >> 5) & 1
        self.button[KeyMap.F1] = (data1 >> 6) & 1
        self.button[KeyMap.F3] = (data1 >> 7) & 1
        
        # 第二个字节的按键
        self.button[KeyMap.A] = (data2 >> 0) & 1
        self.button[KeyMap.B] = (data2 >> 1) & 1
        self.button[KeyMap.X] = (data2 >> 2) & 1
        self.button[KeyMap.Y] = (data2 >> 3) & 1
        self.button[KeyMap.up] = (data2 >> 4) & 1
        self.button[KeyMap.right] = (data2 >> 5) & 1
        self.button[KeyMap.down] = (data2 >> 6) & 1
        self.button[KeyMap.left] = (data2 >> 7) & 1

    def parse_key(self, data):
        """解析摇杆数据"""
        lx_offset = 4
        self.lx = struct.unpack('<f', data[lx_offset:lx_offset + 4])[0]
        rx_offset = 8
        self.rx = struct.unpack('<f', data[rx_offset:rx_offset + 4])[0]
        ry_offset = 12
        self.ry = struct.unpack('<f', data[ry_offset:ry_offset + 4])[0]
        ly_offset = 20
        self.ly = struct.unpack('<f', data[ly_offset:ly_offset + 4])[0] 