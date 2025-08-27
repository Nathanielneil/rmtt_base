#!/usr/bin/env python3

import sys
import time
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))

from core.connection import ConnectionManager
from core.tello_controller import TelloController
from utils.logger import Logger


def status_monitoring_demo():
    logger = Logger("StatusMonitorDemo")
    connection_manager = ConnectionManager()
    
    try:
        logger.info("开始状态监控演示")
        
        # 连接无人机
        connection_manager.connect()
        controller = TelloController(connection_manager)
        
        # 监控时长
        monitor_duration = 30  # 30秒
        start_time = time.time()
        
        logger.info(f"开始监控无人机状态，持续时间: {monitor_duration}秒")
        
        while time.time() - start_time < monitor_duration:
            status = controller.get_status()
            
            if status:
                logger.info("=== 当前状态 ===")
                logger.info(f"连接状态: {'已连接' if status['connected'] else '未连接'}")
                logger.info(f"飞行状态: {'飞行中' if status['in_flight'] else '地面'}")
                logger.info(f"电池电量: {status['battery']}%")
                logger.info(f"飞行高度: {status['height']}cm")
                logger.info(f"温度: {status['temperature']}°C")
                
                # 电池警告
                if status['battery'] < 30:
                    logger.warning(f"电池电量较低: {status['battery']}%")
                
                # 如果正在飞行，显示飞行时间
                if 'flight_time' in status:
                    logger.info(f"飞行时间: {status['flight_time']}秒")
            else:
                logger.error("无法获取无人机状态")
            
            # 等待5秒后再次检查
            time.sleep(5)
        
        logger.info("状态监控演示完成")
        
    except KeyboardInterrupt:
        logger.info("用户中断监控")
    
    except Exception as e:
        logger.error(f"监控过程中出错: {e}")
    
    finally:
        connection_manager.disconnect()


if __name__ == "__main__":
    status_monitoring_demo()