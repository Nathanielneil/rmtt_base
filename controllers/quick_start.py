#!/usr/bin/env python3
"""
RMTT自定义控制器快速启动示例

这是一个简化的示例，展示如何快速使用RMTT自定义控制器进行基本飞行控制
"""

import sys
import time
from pathlib import Path

# 添加项目根目录到路径
sys.path.append(str(Path(__file__).parent.parent))

# 导入RMTT核心组件
from core.connection import ConnectionManager
from utils.logger import Logger

# 导入自定义控制器
from controllers import (
    ControllerManager, ControlMode,
    PositionController, SensorData
)


def quick_position_control_demo():
    """快速位置控制演示"""
    
    logger = Logger("QuickStart")
    logger.info("RMTT自定义控制器快速启动演示")
    
    try:
        # 1. 初始化连接
        logger.info("连接到Tello无人机...")
        connection_manager = ConnectionManager()
        connection_manager.connect()
        tello = connection_manager.get_tello()
        
        # 2. 创建控制器管理器
        controller_manager = ControllerManager()
        
        # 3. 创建位置控制器
        position_controller = PositionController(
            config={
                'altitude_gains': {'kp': 0.8, 'ki': 0.1, 'kd': 0.3},
                'position_gains': {'kp': 0.6, 'ki': 0.05, 'kd': 0.25}
            }
        )
        
        # 4. 注册控制器
        controller_manager.register_controller("position", position_controller)
        
        # 5. 设置控制指令回调
        def send_control_command(command):
            tello.send_rc_control(
                int(command.roll), 
                int(command.pitch), 
                int(command.throttle), 
                int(command.yaw)
            )
        
        controller_manager.register_control_command_callback(send_control_command)
        
        # 6. 启动控制循环
        controller_manager.start_control_loop()
        controller_manager.switch_control_mode(ControlMode.POSITION)
        
        # 7. 起飞
        logger.info("起飞...")
        tello.takeoff()
        time.sleep(3)
        
        # 8. 设置目标位置并悬停
        logger.info("设置目标位置：高度100cm...")
        position_controller.set_target({
            'height_cm': 100.0,
            'x_cm': 0.0,
            'y_cm': 0.0,
            'yaw_deg': 0.0
        })
        
        # 9. 主控制循环
        logger.info("开始位置控制...")
        start_time = time.time()
        
        while time.time() - start_time < 30.0:  # 运行30秒
            # 获取传感器数据
            sensor_data = SensorData(
                timestamp=time.time(),
                height_cm=tello.get_height() or 0,
                tof_distance_cm=tello.get_distance_tof() or 0,
                barometer_cm=tello.get_barometer() or 0,
                pitch_deg=tello.get_pitch() or 0,
                roll_deg=tello.get_roll() or 0,
                yaw_deg=tello.get_yaw() or 0,
                vgx_cm_s=0, vgy_cm_s=0, vgz_cm_s=0,  # 简化版本
                agx_0001g=0, agy_0001g=0, agz_0001g=0,
                battery_percent=tello.get_battery() or 0,
                temperature_deg=tello.get_temperature() or 20,
                wifi_snr=-50
            )
            
            # 更新控制器
            controller_manager.update_sensor_data(sensor_data)
            
            # 检查是否到达目标
            if position_controller.is_at_target():
                logger.info("已到达目标位置！")
            
            # 显示状态（每5秒一次）
            if int(time.time() - start_time) % 5 == 0:
                errors = position_controller.get_control_errors()
                logger.info(f"高度误差: {errors.get('height_error_cm', 0):.1f}cm")
            
            time.sleep(0.02)  # 50Hz控制频率
        
        # 10. 降落
        logger.info("控制演示完成，降落...")
        tello.land()
        time.sleep(3)
        
        logger.info("快速演示成功完成！")
        
    except Exception as e:
        logger.error(f"演示过程出现异常: {e}")
    finally:
        # 清理资源
        try:
            controller_manager.stop_control_loop()
            connection_manager.disconnect()
        except:
            pass


def quick_attitude_control_demo():
    """快速姿态控制演示"""
    
    logger = Logger("QuickAttitudeDemo")
    logger.info("RMTT姿态控制快速演示")
    
    try:
        # 简化的姿态控制演示
        from controllers import AttitudeController
        
        # 初始化
        connection_manager = ConnectionManager()
        connection_manager.connect()
        tello = connection_manager.get_tello()
        
        controller_manager = ControllerManager()
        
        # 创建姿态控制器
        attitude_controller = AttitudeController(
            config={
                'roll_gains': {'kp': 1.2, 'ki': 0.1, 'kd': 0.25},
                'pitch_gains': {'kp': 1.2, 'ki': 0.1, 'kd': 0.25},
                'yaw_gains': {'kp': 1.8, 'ki': 0.15, 'kd': 0.3}
            }
        )
        
        controller_manager.register_controller("attitude", attitude_controller)
        
        # 控制指令回调
        def send_control_command(command):
            tello.send_rc_control(
                int(command.roll), int(command.pitch), 
                int(command.throttle), int(command.yaw)
            )
        
        controller_manager.register_control_command_callback(send_control_command)
        
        # 启动系统
        controller_manager.start_control_loop()
        controller_manager.switch_control_mode(ControlMode.ATTITUDE)
        
        # 起飞
        logger.info("起飞...")
        tello.takeoff()
        time.sleep(3)
        
        # 姿态控制序列
        attitudes = [
            {'roll_deg': 0, 'pitch_deg': 0, 'yaw_deg': 0, 'duration': 3},    # 水平
            {'roll_deg': 15, 'pitch_deg': 0, 'yaw_deg': 0, 'duration': 2},   # 右倾
            {'roll_deg': -15, 'pitch_deg': 0, 'yaw_deg': 0, 'duration': 2},  # 左倾
            {'roll_deg': 0, 'pitch_deg': 0, 'yaw_deg': 45, 'duration': 2},   # 右转
            {'roll_deg': 0, 'pitch_deg': 0, 'yaw_deg': 0, 'duration': 3},    # 回正
        ]
        
        for i, attitude in enumerate(attitudes):
            logger.info(f"执行姿态 {i+1}/{len(attitudes)}: {attitude}")
            
            attitude_controller.set_target({
                'roll_deg': attitude['roll_deg'],
                'pitch_deg': attitude['pitch_deg'],
                'yaw_deg': attitude['yaw_deg']
            })
            
            # 执行控制
            start_time = time.time()
            while time.time() - start_time < attitude['duration']:
                # 更新传感器数据
                sensor_data = SensorData(
                    timestamp=time.time(),
                    height_cm=tello.get_height() or 0,
                    tof_distance_cm=tello.get_distance_tof() or 0,
                    barometer_cm=tello.get_barometer() or 0,
                    pitch_deg=tello.get_pitch() or 0,
                    roll_deg=tello.get_roll() or 0,
                    yaw_deg=tello.get_yaw() or 0,
                    vgx_cm_s=0, vgy_cm_s=0, vgz_cm_s=0,
                    agx_0001g=0, agy_0001g=0, agz_0001g=0,
                    battery_percent=tello.get_battery() or 0,
                    temperature_deg=tello.get_temperature() or 20,
                    wifi_snr=-50
                )
                
                controller_manager.update_sensor_data(sensor_data)
                time.sleep(0.02)
        
        # 降落
        logger.info("姿态演示完成，降落...")
        tello.land()
        time.sleep(3)
        
        logger.info("姿态控制演示成功完成！")
        
    except Exception as e:
        logger.error(f"姿态演示异常: {e}")
    finally:
        try:
            controller_manager.stop_control_loop()
            connection_manager.disconnect()
        except:
            pass


def main():
    """主函数"""
    print("RMTT自定义控制器快速启动")
    print("=" * 40)
    print("1. 位置控制演示")
    print("2. 姿态控制演示")
    print("=" * 40)
    
    try:
        choice = input("请选择演示类型 (1/2): ").strip()
        
        if choice == "1":
            quick_position_control_demo()
        elif choice == "2":
            quick_attitude_control_demo()
        else:
            print("无效选择，运行默认位置控制演示")
            quick_position_control_demo()
            
    except KeyboardInterrupt:
        print("\n用户中断演示")
    except Exception as e:
        print(f"演示异常: {e}")


if __name__ == "__main__":
    main()