"""
位置控制器

实现基于PID算法的3D位置控制，支持高度、前后、左右位置控制
"""

import time
import numpy as np
from typing import Dict, Any
from dataclasses import dataclass

from ..base_controller import BaseController, SensorData, ControlCommand


@dataclass
class PIDGains:
    """PID增益参数"""
    kp: float = 0.0  # 比例增益
    ki: float = 0.0  # 积分增益  
    kd: float = 0.0  # 微分增益
    
    # 限制参数
    max_integral: float = 100.0    # 积分饱和限制
    max_derivative: float = 50.0   # 微分限制
    max_output: float = 100.0      # 输出限制


class PIDController:
    """单轴PID控制器"""
    
    def __init__(self, gains: PIDGains):
        self.gains = gains
        
        # 内部状态
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = 0.0
        self.first_run = True
        
    def compute(self, current_value: float, target_value: float, dt: float) -> float:
        """
        计算PID输出
        
        Args:
            current_value: 当前值
            target_value: 目标值
            dt: 时间间隔
            
        Returns:
            float: PID控制输出
        """
        if dt <= 0:
            return 0.0
            
        # 计算误差
        error = target_value - current_value
        
        # 比例项
        proportional = self.gains.kp * error
        
        # 积分项
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.gains.max_integral, self.gains.max_integral)
        integral = self.gains.ki * self.integral
        
        # 微分项
        if self.first_run:
            derivative = 0.0
            self.first_run = False
        else:
            derivative = self.gains.kd * (error - self.last_error) / dt
            derivative = np.clip(derivative, -self.gains.max_derivative, self.gains.max_derivative)
        
        # 总输出
        output = proportional + integral + derivative
        output = np.clip(output, -self.gains.max_output, self.gains.max_output)
        
        # 保存状态
        self.last_error = error
        self.last_time = time.time()
        
        return output
    
    def reset(self):
        """重置PID状态"""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = 0.0
        self.first_run = True


class PositionController(BaseController):
    """
    3D位置控制器
    
    控制无人机到达指定的空间位置，支持：
    - 高度控制 (基于TOF传感器)
    - 前后位置控制 (基于视觉里程计或外部定位)
    - 左右位置控制 (基于视觉里程计或外部定位)
    - 偏航角控制
    """
    
    def __init__(self, name: str = "position_controller", config: Dict[str, Any] = None):
        super().__init__(name, config)
        
        # 默认PID参数配置
        default_config = {
            'altitude_gains': {'kp': 0.8, 'ki': 0.1, 'kd': 0.3, 'max_output': 50.0},
            'position_gains': {'kp': 0.5, 'ki': 0.05, 'kd': 0.2, 'max_output': 30.0},
            'yaw_gains': {'kp': 1.0, 'ki': 0.1, 'kd': 0.1, 'max_output': 50.0},
            
            # 控制限制
            'max_altitude_cm': 300.0,
            'min_altitude_cm': 20.0,
            'position_tolerance_cm': 5.0,
            'altitude_tolerance_cm': 3.0,
            'yaw_tolerance_deg': 5.0,
            
            # 安全参数
            'emergency_descent_rate': -20.0,
            'max_tilt_angle_deg': 25.0
        }
        
        # 合并用户配置
        for key, value in default_config.items():
            if key not in self.config:
                self.config[key] = value
        
        # 创建PID控制器
        self.altitude_pid = PIDController(PIDGains(**self.config['altitude_gains']))
        self.x_position_pid = PIDController(PIDGains(**self.config['position_gains']))
        self.y_position_pid = PIDController(PIDGains(**self.config['position_gains']))
        self.yaw_pid = PIDController(PIDGains(**self.config['yaw_gains']))
        
        # 位置估计 (简化版本，实际应用中需要更复杂的状态估计)
        self.estimated_x = 0.0
        self.estimated_y = 0.0
        self.estimated_yaw = 0.0
        
        self.logger.info("位置控制器初始化完成")
    
    def set_target(self, target: Dict[str, float]) -> bool:
        """
        设置目标位置
        
        Args:
            target: 包含目标位置的字典
                   - 'height_cm': 目标高度 (厘米)
                   - 'x_cm': 目标X位置 (厘米，可选)
                   - 'y_cm': 目标Y位置 (厘米，可选) 
                   - 'yaw_deg': 目标偏航角 (度，可选)
                   
        Returns:
            bool: 设置是否成功
        """
        if not self._validate_target(target):
            return False
            
        self.target_setpoint = target.copy()
        
        # 如果没有指定位置，使用当前估计位置
        if 'x_cm' not in self.target_setpoint:
            self.target_setpoint['x_cm'] = self.estimated_x
        if 'y_cm' not in self.target_setpoint:
            self.target_setpoint['y_cm'] = self.estimated_y
        if 'yaw_deg' not in self.target_setpoint:
            self.target_setpoint['yaw_deg'] = self.estimated_yaw
            
        self.logger.info(f"设置目标位置: {self.target_setpoint}")
        return True
    
    def compute_control(self, sensor_data: SensorData, target: Dict[str, float]) -> ControlCommand:
        """
        计算位置控制指令
        
        Args:
            sensor_data: 传感器数据
            target: 目标位置
            
        Returns:
            ControlCommand: 控制指令
        """
        current_time = time.time()
        dt = current_time - self.last_update_time if self.last_update_time > 0 else 0.02
        
        # 安全检查
        if not self._safety_check(sensor_data):
            return self.emergency_stop()
        
        # 更新位置估计 (简化版本)
        self._update_position_estimate(sensor_data, dt)
        
        # 获取当前高度 (优先使用TOF传感器)
        current_height = sensor_data.tof_distance_cm if sensor_data.tof_distance_cm > 0 else sensor_data.height_cm
        
        # 计算各轴控制输出
        # 1. 高度控制 (垂直油门)
        throttle = self.altitude_pid.compute(
            current_height, 
            target['height_cm'], 
            dt
        )
        
        # 2. 前后位置控制 (俯仰角)
        pitch = -self.x_position_pid.compute(  # 负号因为前进需要负俯仰
            self.estimated_x,
            target['x_cm'],
            dt
        )
        
        # 3. 左右位置控制 (横滚角) 
        roll = self.y_position_pid.compute(
            self.estimated_y,
            target['y_cm'],
            dt
        )
        
        # 4. 偏航角控制
        yaw = self.yaw_pid.compute(
            self.estimated_yaw,
            target['yaw_deg'],
            dt
        )
        
        # 应用安全限制
        throttle = self._apply_safety_limits(throttle, pitch, roll, yaw, sensor_data)
        
        return ControlCommand(
            timestamp=current_time,
            roll=roll,
            pitch=pitch,
            throttle=throttle,
            yaw=yaw
        )
    
    def _update_position_estimate(self, sensor_data: SensorData, dt: float):
        """
        更新位置估计 (简化版本)
        
        在实际应用中，这里应该集成更复杂的状态估计算法，
        如扩展卡尔曼滤波器(EKF)或无迹卡尔曼滤波器(UKF)
        """
        # 基于速度积分的位置估计 (非常简化)
        # 注意: 这只是示例，实际应用中需要更精确的方法
        if dt > 0:
            # 转换速度单位并积分
            vx_ms = sensor_data.vgx_cm_s / 100.0  # cm/s -> m/s
            vy_ms = sensor_data.vgy_cm_s / 100.0
            
            self.estimated_x += vx_ms * dt * 100  # 转回cm
            self.estimated_y += vy_ms * dt * 100
        
        # 偏航角更新
        self.estimated_yaw = sensor_data.yaw_deg
    
    def _apply_safety_limits(self, throttle: float, pitch: float, roll: float, yaw: float, sensor_data: SensorData) -> float:
        """
        应用安全限制
        
        Args:
            throttle: 油门指令
            pitch: 俯仰指令  
            roll: 横滚指令
            yaw: 偏航指令
            sensor_data: 传感器数据
            
        Returns:
            float: 安全限制后的油门指令
        """
        # 高度安全限制
        current_height = sensor_data.tof_distance_cm if sensor_data.tof_distance_cm > 0 else sensor_data.height_cm
        
        if current_height > self.config['max_altitude_cm']:
            # 超过最大高度，强制下降
            throttle = min(throttle, -10.0)
            self.logger.warning(f"超过最大高度限制: {current_height}cm")
            
        elif current_height < self.config['min_altitude_cm']:
            # 低于最小高度，强制上升
            throttle = max(throttle, 10.0)
            self.logger.warning(f"低于最小高度限制: {current_height}cm")
        
        # 姿态角限制
        max_tilt = self.config['max_tilt_angle_deg']
        if abs(sensor_data.pitch_deg) > max_tilt or abs(sensor_data.roll_deg) > max_tilt:
            # 姿态角过大，减小控制输出
            throttle *= 0.5
            self.logger.warning(f"姿态角过大: pitch={sensor_data.pitch_deg}, roll={sensor_data.roll_deg}")
        
        # 电池安全
        if sensor_data.battery_percent < 15:
            # 电池严重不足，执行紧急下降
            throttle = self.config['emergency_descent_rate']
            self.logger.critical("电池电量严重不足，执行紧急下降")
        
        return throttle
    
    def _validate_target(self, target: Dict[str, float]) -> bool:
        """验证目标参数"""
        if 'height_cm' not in target:
            self.logger.error("目标参数必须包含 'height_cm'")
            return False
        
        height = target['height_cm']
        if not (self.config['min_altitude_cm'] <= height <= self.config['max_altitude_cm']):
            self.logger.error(f"目标高度 {height}cm 超出安全范围")
            return False
        
        return True
    
    def is_at_target(self, tolerance_multiplier: float = 1.0) -> bool:
        """
        检查是否到达目标位置
        
        Args:
            tolerance_multiplier: 容差乘数
            
        Returns:
            bool: 是否到达目标
        """
        if self.target_setpoint is None or self.current_sensor_data is None:
            return False
        
        # 高度检查
        current_height = (
            self.current_sensor_data.tof_distance_cm 
            if self.current_sensor_data.tof_distance_cm > 0 
            else self.current_sensor_data.height_cm
        )
        height_error = abs(current_height - self.target_setpoint['height_cm'])
        height_ok = height_error <= (self.config['altitude_tolerance_cm'] * tolerance_multiplier)
        
        # 位置检查 (简化)
        x_error = abs(self.estimated_x - self.target_setpoint['x_cm'])
        y_error = abs(self.estimated_y - self.target_setpoint['y_cm'])
        position_ok = (
            x_error <= (self.config['position_tolerance_cm'] * tolerance_multiplier) and
            y_error <= (self.config['position_tolerance_cm'] * tolerance_multiplier)
        )
        
        # 偏航角检查
        yaw_error = abs(self.estimated_yaw - self.target_setpoint['yaw_deg'])
        yaw_ok = yaw_error <= (self.config['yaw_tolerance_deg'] * tolerance_multiplier)
        
        return height_ok and position_ok and yaw_ok
    
    def _on_reset(self) -> bool:
        """重置控制器状态"""
        self.altitude_pid.reset()
        self.x_position_pid.reset()
        self.y_position_pid.reset()
        self.yaw_pid.reset()
        
        self.estimated_x = 0.0
        self.estimated_y = 0.0 
        self.estimated_yaw = 0.0
        
        return True
    
    def get_position_estimate(self) -> Dict[str, float]:
        """获取当前位置估计"""
        return {
            'x_cm': self.estimated_x,
            'y_cm': self.estimated_y,
            'yaw_deg': self.estimated_yaw
        }
    
    def get_control_errors(self) -> Dict[str, float]:
        """获取当前控制误差"""
        if self.target_setpoint is None or self.current_sensor_data is None:
            return {}
        
        current_height = (
            self.current_sensor_data.tof_distance_cm 
            if self.current_sensor_data.tof_distance_cm > 0 
            else self.current_sensor_data.height_cm
        )
        
        return {
            'height_error_cm': self.target_setpoint['height_cm'] - current_height,
            'x_error_cm': self.target_setpoint['x_cm'] - self.estimated_x,
            'y_error_cm': self.target_setpoint['y_cm'] - self.estimated_y,
            'yaw_error_deg': self.target_setpoint['yaw_deg'] - self.estimated_yaw
        }