#!/usr/bin/env python3

import sys
import time
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))

from core.connection import ConnectionManager
from core.tello_controller import TelloController
from media.video_stream import VideoStreamHandler
from utils.logger import Logger


def video_recording_demo():
    logger = Logger("VideoRecordingDemo")
    connection_manager = ConnectionManager()
    
    try:
        logger.info("开始视频录制演示")
        
        # 连接无人机
        connection_manager.connect()
        
        # 创建控制器和视频处理器
        controller = TelloController(connection_manager)
        video_handler = VideoStreamHandler(connection_manager)
        
        # 启动视频流
        logger.info("启动视频流...")
        video_handler.start_stream()
        time.sleep(3)
        
        # 开始录制
        logger.info("开始录制视频...")
        video_handler.start_recording("circle_flight.avi")
        
        # 起飞
        logger.info("起飞并开始圆形飞行...")
        controller.takeoff()
        time.sleep(3)
        
        # 上升到合适高度
        controller.move_up(150)
        time.sleep(2)
        
        # 执行圆形飞行 (8个45度转弯)
        for i in range(8):
            logger.info(f"圆形飞行第 {i+1}/8 段")
            
            # 前进
            controller.move_forward(80)
            time.sleep(2)
            
            # 转45度
            controller.rotate_clockwise(45)
            time.sleep(2)
            
            # 悬停观察
            controller.hover(1)
        
        # 回到中心并下降
        controller.move_down(100)
        time.sleep(2)
        
        # 停止录制
        logger.info("停止录制...")
        video_handler.stop_recording()
        
        # 降落
        controller.land()
        
        logger.info("视频录制演示完成")
        
    except Exception as e:
        logger.error(f"演示过程中出错: {e}")
        try:
            video_handler.stop_recording()
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
    video_recording_demo()