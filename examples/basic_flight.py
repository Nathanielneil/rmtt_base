#!/usr/bin/env python3

import sys
import time
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))

from core.connection import ConnectionManager
from core.tello_controller import TelloController
from media.video_stream import VideoStreamHandler
from utils.logger import Logger


def basic_flight_demo():
    logger = Logger("BasicFlightDemo")
    connection_manager = ConnectionManager()
    
    try:
        logger.info("开始基础飞行演示")
        
        # 连接无人机
        logger.info("连接到Tello...")
        connection_manager.connect()
        
        # 创建控制器和视频处理器
        controller = TelloController(connection_manager)
        video_handler = VideoStreamHandler(connection_manager)
        
        # 启动视频流
        logger.info("启动视频流...")
        video_handler.start_stream()
        time.sleep(2)
        
        # 基础飞行序列
        logger.info("执行飞行序列...")
        
        # 起飞
        controller.takeoff()
        time.sleep(3)
        
        # 拍一张起飞照片
        video_handler.capture_image("takeoff.jpg")
        
        # 上升
        controller.move_up(100)
        time.sleep(2)
        
        # 前进
        controller.move_forward(100)
        time.sleep(2)
        
        # 右转90度
        controller.rotate_clockwise(90)
        time.sleep(2)
        
        # 再前进
        controller.move_forward(100)
        time.sleep(2)
        
        # 右转90度
        controller.rotate_clockwise(90)
        time.sleep(2)
        
        # 后退回到起始位置上方
        controller.move_back(100)
        time.sleep(2)
        
        # 左转90度
        controller.rotate_counter_clockwise(90)
        time.sleep(2)
        
        # 后退
        controller.move_back(100)
        time.sleep(2)
        
        # 左转90度回到初始方向
        controller.rotate_counter_clockwise(90)
        time.sleep(2)
        
        # 拍一张完成照片
        video_handler.capture_image("completed.jpg")
        
        # 降落
        controller.land()
        
        logger.info("基础飞行演示完成")
        
    except Exception as e:
        logger.error(f"演示过程中出错: {e}")
        try:
            if controller and controller.in_flight:
                controller.emergency_stop()
        except:
            pass
    
    finally:
        # 清理资源
        if 'video_handler' in locals():
            video_handler.stop_stream()
        connection_manager.disconnect()


if __name__ == "__main__":
    basic_flight_demo()