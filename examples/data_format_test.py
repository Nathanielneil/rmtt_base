#!/usr/bin/env python3
"""
数据格式测试脚本
用于验证Tello无人机状态数据的解析和记录格式
"""

import sys
import time
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))

from core.connection import ConnectionManager
from data.flight_data_recorder import FlightDataRecorder
from utils.logger import Logger


def test_data_format():
    logger = Logger("DataFormatTest")
    connection_manager = ConnectionManager()
    
    try:
        logger.info("开始数据格式测试...")
        
        # 连接无人机
        logger.info("连接到Tello...")
        connection_manager.connect()
        
        # 创建数据记录器
        data_recorder = FlightDataRecorder(connection_manager)
        
        # 获取一次完整状态数据进行测试
        tello = connection_manager.get_tello()
        
        logger.info("获取完整状态字符串:")
        state = tello.get_current_state()
        print(f"原始状态字符串: {state}")
        
        # 测试各个数据解析
        logger.info("测试数据解析:")
        
        if state and isinstance(state, str):
            # 位置数据
            position_data = data_recorder._parse_state_data(state, ['x', 'y', 'z'])
            print(f"位置坐标 (x, y, z): {position_data}")
            
            # 速度数据
            velocity_data = data_recorder._parse_state_data(state, ['vgx', 'vgy', 'vgz'])
            print(f"速度分量 (vgx, vgy, vgz): {velocity_data}")
            
            # 加速度数据
            acceleration_data = data_recorder._parse_state_data(state, ['agx', 'agy', 'agz'])
            print(f"加速度分量 (agx, agy, agz): {acceleration_data}")
        
        # 姿态数据
        try:
            pitch = tello.get_pitch()
            roll = tello.get_roll()
            yaw = tello.get_yaw()
            print(f"姿态角度 (pitch, roll, yaw): {pitch}, {roll}, {yaw}")
        except Exception as e:
            print(f"获取姿态数据失败: {e}")
        
        # 测试完整数据记录
        logger.info("测试完整数据记录:")
        test_data = data_recorder._collect_drone_data()
        
        if test_data:
            print(f"CSV头部: {data_recorder.csv_headers}")
            print(f"数据长度: {len(test_data)}")
            print(f"头部长度: {len(data_recorder.csv_headers)}")
            
            if len(test_data) == len(data_recorder.csv_headers):
                print("数据字段匹配正确!")
                for i, (header, value) in enumerate(zip(data_recorder.csv_headers, test_data)):
                    print(f"  {header}: {value}")
            else:
                print("警告: 数据字段数量不匹配!")
        
        # 短时间数据记录测试
        logger.info("开始5秒数据记录测试...")
        data_recorder.start_recording("format_test")
        
        time.sleep(5)
        
        data_recorder.stop_recording()
        
        status = data_recorder.get_recording_status()
        logger.info(f"测试完成，记录文件: {status.get('file_path', 'N/A')}")
        
    except Exception as e:
        logger.error(f"测试过程中出错: {e}")
    
    finally:
        connection_manager.disconnect()


if __name__ == "__main__":
    test_data_format()