#!/usr/bin/env python3

import sys
import time
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))

from core.connection import ConnectionManager
from core.tello_controller import TelloController
from data.flight_data_recorder import FlightDataRecorder
from utils.logger import Logger


def flight_data_recording_demo():
    logger = Logger("FlightDataRecordingDemo")
    connection_manager = ConnectionManager()
    
    try:
        logger.info("开始飞行数据记录演示")
        
        # 连接无人机
        connection_manager.connect()
        
        # 创建控制器和数据记录器
        controller = TelloController(connection_manager)
        data_recorder = FlightDataRecorder(connection_manager)
        
        # 开始数据记录
        logger.info("开始记录飞行数据...")
        data_recorder.start_recording("demo_flight")
        
        # 等待一些初始数据
        time.sleep(2)
        
        # 执行飞行序列
        logger.info("执行飞行序列...")
        
        # 起飞
        controller.takeoff()
        data_recorder.add_command_log('takeoff', True)
        time.sleep(3)
        
        # 记录飞行动作
        movements = [
            ('move_up', 100),
            ('move_forward', 100),
            ('rotate_clockwise', 90),
            ('move_right', 80),
            ('move_back', 100),
            ('rotate_counter_clockwise', 90),
            ('move_down', 50)
        ]
        
        for movement, value in movements:
            logger.info(f"执行: {movement} {value}")
            if movement == 'move_up':
                controller.move_up(value)
            elif movement == 'move_down':
                controller.move_down(value)
            elif movement == 'move_forward':
                controller.move_forward(value)
            elif movement == 'move_back':
                controller.move_back(value)
            elif movement == 'move_right':
                controller.move_right(value)
            elif movement == 'move_left':
                controller.move_left(value)
            elif movement == 'rotate_clockwise':
                controller.rotate_clockwise(value)
            elif movement == 'rotate_counter_clockwise':
                controller.rotate_counter_clockwise(value)
            
            # 记录命令
            data_recorder.add_command_log(f'{movement} {value}', True)
            time.sleep(2)
        
        # 悬停并记录数据
        logger.info("悬停5秒记录稳定数据...")
        controller.hover(5)
        data_recorder.add_command_log('hover 5', True)
        
        # 降落
        controller.land()
        data_recorder.add_command_log('land', True)
        
        # 停止数据记录
        time.sleep(2)
        data_recorder.stop_recording()
        
        # 显示记录统计
        status = data_recorder.get_recording_status()
        logger.info("飞行数据记录演示完成")
        logger.info(f"记录的数据文件: {status.get('file_path', 'N/A')}")
        
    except Exception as e:
        logger.error(f"演示过程中出错: {e}")
        try:
            if 'data_recorder' in locals():
                data_recorder.stop_recording()
            if 'controller' in locals() and controller.in_flight:
                controller.emergency_stop()
        except:
            pass
    
    finally:
        # 清理资源
        connection_manager.disconnect()


if __name__ == "__main__":
    flight_data_recording_demo()