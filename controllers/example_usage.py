#!/usr/bin/env python3
"""
RMTT自定义控制器使用示例

本文件展示了如何使用RMTT自定义控制器系统进行各种飞行控制任务
"""

import sys
import time
import numpy as np
from pathlib import Path

# 添加项目根目录到路径
sys.path.append(str(Path(__file__).parent.parent))

# 导入RMTT核心组件
from core.connection import ConnectionManager
from data.flight_data_recorder import FlightDataRecorder
from utils.logger import Logger

# 导入自定义控制器
from controllers import (
    ControllerManager, ControllerManagerConfig, ControlMode,
    AttitudeController,
    PositionController, VelocityController, TrajectoryController,
    SensorData, ControlCommand, TrajectoryType, Waypoint
)


class CustomControllerDemo:
    """自定义控制器演示类"""
    
    def __init__(self):
        self.logger = Logger("CustomControllerDemo")
        
        # 初始化RMTT基础组件
        self.connection_manager = ConnectionManager()
        self.data_recorder = FlightDataRecorder(self.connection_manager)
        
        # 初始化控制器管理器
        manager_config = ControllerManagerConfig(
            update_frequency_hz=50.0,
            safety_check_enabled=True,
            auto_fallback_enabled=True
        )
        self.controller_manager = ControllerManager(manager_config)
        
        # 设置回调函数
        self.controller_manager.register_control_command_callback(self._on_control_command)
        
        self.logger.info("自定义控制器演示系统初始化完成")
    
    def initialize_system(self) -> bool:
        """初始化系统"""
        try:
            # 连接无人机
            self.logger.info("连接到Tello无人机...")
            self.connection_manager.connect()
            
            # 创建并注册控制器
            self._create_and_register_controllers()
            
            # 启动控制器管理器
            self.controller_manager.start_control_loop()
            
            # 启动数据记录
            self.data_recorder.start_recording("custom_controller_demo")
            
            self.logger.info("系统初始化完成")
            return True
            
        except Exception as e:
            self.logger.error(f"系统初始化失败: {e}")
            return False
    
    def _create_and_register_controllers(self):
        """创建并注册所有控制器"""
        
        # 1. 姿态控制器
        attitude_controller = AttitudeController(
            name="demo_attitude",
            config={
                'roll_gains': {'kp': 1.2, 'ki': 0.1, 'kd': 0.25},
                'pitch_gains': {'kp': 1.2, 'ki': 0.1, 'kd': 0.25},
                'yaw_gains': {'kp': 1.8, 'ki': 0.15, 'kd': 0.3},
                'altitude_hold_enabled': True
            }
        )
        self.controller_manager.register_controller("attitude", attitude_controller)
        
        # 2. 位置控制器
        position_controller = PositionController(
            name="demo_position", 
            config={
                'altitude_gains': {'kp': 0.9, 'ki': 0.1, 'kd': 0.35},
                'position_gains': {'kp': 0.6, 'ki': 0.05, 'kd': 0.25}
            }
        )
        self.controller_manager.register_controller("position", position_controller)
        
        # 3. 速度控制器
        velocity_controller = VelocityController(
            name="demo_velocity",
            config={
                'vz_gains': {'kp': 1.2, 'ki': 0.2, 'kd': 0.15},
                'max_vertical_speed': 40.0,
                'max_horizontal_speed': 60.0
            }
        )
        self.controller_manager.register_controller("velocity", velocity_controller)
        
        # 4. 轨迹控制器
        trajectory_controller = TrajectoryController(
            name="demo_trajectory",
            config={
                'max_speed_cm_s': 40.0,
                'feedforward_gain': 0.7,
                'position_kp': 0.5
            }
        )
        self.controller_manager.register_controller("trajectory", trajectory_controller)
        
        self.logger.info("所有控制器注册完成")
    
    def demo_attitude_control(self):
        """演示姿态控制"""
        self.logger.info("=== 姿态控制演示 ===")
        
        # 切换到姿态控制模式
        self.controller_manager.switch_control_mode(ControlMode.ATTITUDE)
        
        # 获取姿态控制器
        attitude_controller = self.controller_manager.controllers["attitude"]
        
        # 起飞
        self.logger.info("起飞...")
        self.connection_manager.get_tello().takeoff()
        time.sleep(3)
        
        # 姿态控制序列
        attitude_sequence = [
            {'roll_deg': 0, 'pitch_deg': 0, 'yaw_deg': 0, 'hold_time': 3},      # 水平悬停
            {'roll_deg': 10, 'pitch_deg': 0, 'yaw_deg': 0, 'hold_time': 2},     # 右倾
            {'roll_deg': -10, 'pitch_deg': 0, 'yaw_deg': 0, 'hold_time': 2},    # 左倾
            {'roll_deg': 0, 'pitch_deg': 10, 'yaw_deg': 0, 'hold_time': 2},     # 前倾
            {'roll_deg': 0, 'pitch_deg': -10, 'yaw_deg': 0, 'hold_time': 2},    # 后倾
            {'roll_deg': 0, 'pitch_deg': 0, 'yaw_deg': 45, 'hold_time': 2},     # 右转45度
            {'roll_deg': 0, 'pitch_deg': 0, 'yaw_deg': -45, 'hold_time': 2},    # 左转45度
            {'roll_deg': 0, 'pitch_deg': 0, 'yaw_deg': 0, 'hold_time': 3},      # 回归水平
        ]
        
        for i, attitude_cmd in enumerate(attitude_sequence):
            self.logger.info(f"执行姿态指令 {i+1}/{len(attitude_sequence)}: {attitude_cmd}")
            
            # 设置目标姿态
            attitude_controller.set_target({
                'roll_deg': attitude_cmd['roll_deg'],
                'pitch_deg': attitude_cmd['pitch_deg'], 
                'yaw_deg': attitude_cmd['yaw_deg']
            })
            
            # 等待姿态稳定
            self._wait_for_attitude_stable(attitude_controller, attitude_cmd['hold_time'])
        
        self.logger.info("姿态控制演示完成")
    
    def demo_position_control(self):
        """演示位置控制"""
        self.logger.info("=== 位置控制演示 ===")
        
        # 切换到位置控制模式
        self.controller_manager.switch_control_mode(ControlMode.POSITION)
        
        # 获取位置控制器
        position_controller = self.controller_manager.controllers["position"]
        
        # 位置控制序列（相对位置）
        position_sequence = [
            {'height_cm': 80, 'x_cm': 0, 'y_cm': 0, 'yaw_deg': 0},      # 初始位置
            {'height_cm': 120, 'x_cm': 0, 'y_cm': 0, 'yaw_deg': 0},     # 上升
            {'height_cm': 120, 'x_cm': 50, 'y_cm': 0, 'yaw_deg': 0},    # 前进
            {'height_cm': 120, 'x_cm': 50, 'y_cm': 50, 'yaw_deg': 0},   # 右移
            {'height_cm': 120, 'x_cm': 0, 'y_cm': 50, 'yaw_deg': 0},    # 后退
            {'height_cm': 120, 'x_cm': 0, 'y_cm': 0, 'yaw_deg': 90},    # 回归并转向
            {'height_cm': 80, 'x_cm': 0, 'y_cm': 0, 'yaw_deg': 0},      # 下降并回正
        ]
        
        for i, pos_cmd in enumerate(position_sequence):
            self.logger.info(f"执行位置指令 {i+1}/{len(position_sequence)}: {pos_cmd}")
            
            # 设置目标位置
            position_controller.set_target(pos_cmd)
            
            # 等待到达目标位置
            self._wait_for_position_reached(position_controller, timeout=15.0)
        
        self.logger.info("位置控制演示完成")
    
    def demo_velocity_control(self):
        """演示速度控制"""
        self.logger.info("=== 速度控制演示 ===")
        
        # 切换到速度控制模式
        self.controller_manager.switch_control_mode(ControlMode.VELOCITY)
        
        # 获取速度控制器
        velocity_controller = self.controller_manager.controllers["velocity"]
        
        # 速度控制序列
        velocity_sequence = [
            {'vx_cm_s': 0, 'vy_cm_s': 0, 'vz_cm_s': 0, 'vyaw_deg_s': 0, 'duration': 2},      # 悬停
            {'vx_cm_s': 30, 'vy_cm_s': 0, 'vz_cm_s': 0, 'vyaw_deg_s': 0, 'duration': 3},     # 前进
            {'vx_cm_s': 0, 'vy_cm_s': 0, 'vz_cm_s': 0, 'vyaw_deg_s': 0, 'duration': 1},      # 停止
            {'vx_cm_s': -30, 'vy_cm_s': 0, 'vz_cm_s': 0, 'vyaw_deg_s': 0, 'duration': 3},    # 后退
            {'vx_cm_s': 0, 'vy_cm_s': 30, 'vz_cm_s': 0, 'vyaw_deg_s': 0, 'duration': 3},     # 右移
            {'vx_cm_s': 0, 'vy_cm_s': -30, 'vz_cm_s': 0, 'vyaw_deg_s': 0, 'duration': 3},    # 左移
            {'vx_cm_s': 0, 'vy_cm_s': 0, 'vz_cm_s': 20, 'vyaw_deg_s': 0, 'duration': 2},     # 上升
            {'vx_cm_s': 0, 'vy_cm_s': 0, 'vz_cm_s': -20, 'vyaw_deg_s': 0, 'duration': 2},    # 下降
            {'vx_cm_s': 0, 'vy_cm_s': 0, 'vz_cm_s': 0, 'vyaw_deg_s': 45, 'duration': 2},     # 旋转
            {'vx_cm_s': 0, 'vy_cm_s': 0, 'vz_cm_s': 0, 'vyaw_deg_s': 0, 'duration': 2},      # 停止
        ]
        
        for i, vel_cmd in enumerate(velocity_sequence):
            self.logger.info(f"执行速度指令 {i+1}/{len(velocity_sequence)}: {vel_cmd}")
            
            # 设置目标速度
            velocity_controller.set_target({
                'vx_cm_s': vel_cmd['vx_cm_s'],
                'vy_cm_s': vel_cmd['vy_cm_s'],
                'vz_cm_s': vel_cmd['vz_cm_s'], 
                'vyaw_deg_s': vel_cmd['vyaw_deg_s']
            })
            
            # 执行指定时间
            time.sleep(vel_cmd['duration'])
        
        self.logger.info("速度控制演示完成")
    
    def demo_trajectory_control(self):
        """演示轨迹控制"""
        self.logger.info("=== 轨迹控制演示 ===")
        
        # 切换到轨迹控制模式
        self.controller_manager.switch_control_mode(ControlMode.TRAJECTORY)
        
        # 获取轨迹控制器
        trajectory_controller = self.controller_manager.controllers["trajectory"]
        
        # 演示1：方形轨迹
        self.logger.info("执行方形轨迹...")
        trajectory_controller.set_target({
            'trajectory_type': TrajectoryType.SQUARE.value,
            'parameters': {
                'size_cm': 120.0,
                'height_cm': 100.0,
                'speed_cm_s': 35.0
            }
        })
        
        # 等待轨迹完成
        self._wait_for_trajectory_completion(trajectory_controller, timeout=30.0)
        
        time.sleep(2)  # 中间休息
        
        # 演示2：圆形轨迹
        self.logger.info("执行圆形轨迹...")
        trajectory_controller.set_target({
            'trajectory_type': TrajectoryType.CIRCULAR.value,
            'parameters': {
                'radius_cm': 80.0,
                'height_cm': 100.0,
                'speed_cm_s': 30.0,
                'num_laps': 1.0
            }
        })
        
        # 等待轨迹完成
        self._wait_for_trajectory_completion(trajectory_controller, timeout=25.0)
        
        self.logger.info("轨迹控制演示完成")
    
    def demo_waypoint_navigation(self):
        """演示航点导航"""
        self.logger.info("=== 航点导航演示 ===")
        
        # 切换到轨迹控制模式用于航点导航
        self.controller_manager.switch_control_mode(ControlMode.TRAJECTORY)
        
        # 获取轨迹控制器
        trajectory_controller = self.controller_manager.controllers["trajectory"]
        
        # 定义航点序列
        waypoints = [
            {'x_cm': 0, 'y_cm': 0, 'height_cm': 80, 'yaw_deg': 0, 'speed_cm_s': 25, 'hold_time_s': 1, 'tolerance_cm': 10},
            {'x_cm': 100, 'y_cm': 0, 'height_cm': 80, 'yaw_deg': 0, 'speed_cm_s': 25, 'hold_time_s': 2, 'tolerance_cm': 10},
            {'x_cm': 100, 'y_cm': 100, 'height_cm': 120, 'yaw_deg': 90, 'speed_cm_s': 20, 'hold_time_s': 2, 'tolerance_cm': 10},
            {'x_cm': 0, 'y_cm': 100, 'height_cm': 120, 'yaw_deg': 180, 'speed_cm_s': 25, 'hold_time_s': 2, 'tolerance_cm': 10},
            {'x_cm': 0, 'y_cm': 0, 'height_cm': 80, 'yaw_deg': 270, 'speed_cm_s': 30, 'hold_time_s': 3, 'tolerance_cm': 10},
        ]
        
        # 设置航点轨迹
        trajectory_controller.set_target({
            'trajectory_type': TrajectoryType.WAYPOINT.value,
            'waypoints': waypoints
        })
        
        # 监控航点导航进度
        self._monitor_waypoint_progress(trajectory_controller, timeout=60.0)
        
        self.logger.info("航点导航演示完成")
    
    def _wait_for_attitude_stable(self, controller, hold_time: float):
        """等待姿态稳定"""
        start_time = time.time()
        stable_start_time = None
        
        while time.time() - start_time < hold_time + 5.0:  # 最大等待时间
            # 更新传感器数据
            self._update_sensor_data()
            
            if controller.is_attitude_stable():
                if stable_start_time is None:
                    stable_start_time = time.time()
                elif time.time() - stable_start_time >= hold_time:
                    break
            else:
                stable_start_time = None
            
            time.sleep(0.1)
    
    def _wait_for_position_reached(self, controller, timeout: float):
        """等待到达目标位置"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            # 更新传感器数据
            self._update_sensor_data()
            
            if controller.is_at_target():
                self.logger.info("目标位置已到达")
                return True
            
            time.sleep(0.2)
        
        self.logger.warning("位置控制超时")
        return False
    
    def _wait_for_trajectory_completion(self, controller, timeout: float):
        """等待轨迹完成"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            # 更新传感器数据
            self._update_sensor_data()
            
            status = controller.get_trajectory_status()
            if status['completed']:
                self.logger.info(f"轨迹跟踪完成，进度: {status['progress']*100:.1f}%")
                return True
            
            # 显示进度
            if int(time.time()) % 5 == 0:  # 每5秒显示一次进度
                self.logger.info(f"轨迹进度: {status['progress']*100:.1f}%")
            
            time.sleep(0.5)
        
        self.logger.warning("轨迹跟踪超时")
        return False
    
    def _monitor_waypoint_progress(self, controller, timeout: float):
        """监控航点导航进度"""
        start_time = time.time()
        last_waypoint_index = -1
        
        while time.time() - start_time < timeout:
            # 更新传感器数据
            self._update_sensor_data()
            
            status = controller.get_trajectory_status()
            
            if status['completed']:
                self.logger.info("所有航点导航完成")
                return True
            
            # 检查是否到达新航点
            current_index = status.get('current_waypoint_index', 0)
            if current_index != last_waypoint_index:
                self.logger.info(f"正在前往航点 {current_index + 1}")
                last_waypoint_index = current_index
            
            time.sleep(1.0)
        
        self.logger.warning("航点导航超时")
        return False
    
    def _update_sensor_data(self):
        """更新传感器数据"""
        try:
            tello = self.connection_manager.get_tello()
            
            # 构建传感器数据
            sensor_data = SensorData(
                timestamp=time.time(),
                height_cm=tello.get_height() or 0,
                tof_distance_cm=tello.get_distance_tof() or 0,
                barometer_cm=tello.get_barometer() or 0,
                pitch_deg=tello.get_pitch() or 0,
                roll_deg=tello.get_roll() or 0,
                yaw_deg=tello.get_yaw() or 0,
                vgx_cm_s=0,  # 简化版本，实际需要从状态获取
                vgy_cm_s=0,
                vgz_cm_s=0,
                agx_0001g=0,
                agy_0001g=0,
                agz_0001g=0,
                battery_percent=tello.get_battery() or 0,
                temperature_deg=tello.get_temperature() or 20,
                wifi_snr=-50
            )
            
            # 更新控制器管理器的传感器数据
            self.controller_manager.update_sensor_data(sensor_data)
            
        except Exception as e:
            self.logger.error(f"更新传感器数据失败: {e}")
    
    def _on_control_command(self, command: ControlCommand):
        """处理控制指令"""
        try:
            tello = self.connection_manager.get_tello()
            
            # 发送RC控制指令到无人机
            tello.send_rc_control(
                int(command.roll),      # 左右 
                int(command.pitch),     # 前后
                int(command.throttle),  # 上下
                int(command.yaw)        # 旋转
            )
        except Exception as e:
            self.logger.error(f"发送控制指令失败: {e}")
    
    def run_full_demo(self):
        """运行完整演示"""
        try:
            # 初始化系统
            if not self.initialize_system():
                return False
            
            self.logger.info("开始RMTT自定义控制器完整演示")
            
            # 启动数据采集线程
            import threading
            sensor_thread = threading.Thread(target=self._sensor_data_loop, daemon=True)
            sensor_thread.start()
            
            # 依次执行各种控制演示
            self.demo_attitude_control()
            time.sleep(3)
            
            self.demo_position_control()  
            time.sleep(3)
            
            self.demo_velocity_control()
            time.sleep(3)
            
            self.demo_trajectory_control()
            time.sleep(3)
            
            self.demo_waypoint_navigation()
            
            # 最终降落
            self.logger.info("演示完成，执行降落...")
            self.connection_manager.get_tello().land()
            time.sleep(3)
            
            self.logger.info("RMTT自定义控制器演示成功完成！")
            return True
            
        except Exception as e:
            self.logger.error(f"演示过程出现异常: {e}")
            return False
        finally:
            self._cleanup()
    
    def _sensor_data_loop(self):
        """传感器数据采集循环"""
        while self.controller_manager.is_running:
            self._update_sensor_data()
            time.sleep(0.02)  # 50Hz频率
    
    def _cleanup(self):
        """清理资源"""
        try:
            # 停止控制器管理器
            self.controller_manager.stop_control_loop()
            
            # 停止数据记录
            self.data_recorder.stop_recording()
            
            # 断开连接
            self.connection_manager.disconnect()
            
            self.logger.info("资源清理完成")
        except Exception as e:
            self.logger.error(f"资源清理失败: {e}")


def main():
    """主函数"""
    print("RMTT自定义控制器系统演示")
    print("=" * 50)
    
    # 创建演示实例
    demo = CustomControllerDemo()
    
    try:
        # 运行完整演示
        success = demo.run_full_demo()
        
        if success:
            print("\n演示成功完成！")
            print("查看日志文件获取详细信息")
            print("查看记录的飞行数据进行分析")
        else:
            print("\n演示执行失败！")
            print("请检查日志文件了解错误详情")
    
    except KeyboardInterrupt:
        print("\n用户中断演示")
        demo._cleanup()
    
    except Exception as e:
        print(f"\n演示异常终止: {e}")
        demo._cleanup()


if __name__ == "__main__":
    main()