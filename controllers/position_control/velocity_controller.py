"""
速度控制器

实现基于速度的控制，直接控制无人机的线速度和角速度
"""

import time
import numpy as np
from typing import Dict, Any
from dataclasses import dataclass

from ..base_controller import BaseController, SensorData, ControlCommand
from .position_controller import PIDController, PIDGains


class VelocityController(BaseController):
    """
    速度控制器
    
    直接控制无人机的速度，包括：
    - 垂直速度控制
    - 前后速度控制  
    - 左右速度控制
    - 偏航角速度控制
    
    这种控制方式更适合需要精确速度控制的应用，
    如跟踪移动目标、轨迹跟踪等。
    """
    
    def __init__(self, name: str = "velocity_controller", config: Dict[str, Any] = None):
        super().__init__(name, config)
        
        # 默认配置
        default_config = {
            # PID参数 - 速度控制通常需要更快的响应
            'vz_gains': {'kp': 1.0, 'ki': 0.2, 'kd': 0.1, 'max_output': 80.0},
            'vx_gains': {'kp': 0.8, 'ki': 0.1, 'kd': 0.15, 'max_output': 60.0},
            'vy_gains': {'kp': 0.8, 'ki': 0.1, 'kd': 0.15, 'max_output': 60.0},
            'vyaw_gains': {'kp': 1.2, 'ki': 0.15, 'kd': 0.1, 'max_output': 70.0},
            
            # 速度限制 (cm/s)
            'max_vertical_speed': 50.0,
            'max_horizontal_speed': 80.0,
            'max_yaw_rate_deg_s': 90.0,
            
            # 速度滤波参数
            'velocity_filter_alpha': 0.7,  # 低通滤波系数
            
            # 安全参数
            'min_altitude_cm': 20.0,
            'max_altitude_cm': 300.0,
            'emergency_stop_deceleration': 2.0  # m/s²
        }
        
        # 合并配置
        for key, value in default_config.items():
            if key not in self.config:
                self.config[key] = value
        
        # 创建速度PID控制器
        self.vz_pid = PIDController(PIDGains(**self.config['vz_gains']))
        self.vx_pid = PIDController(PIDGains(**self.config['vx_gains'])) 
        self.vy_pid = PIDController(PIDGains(**self.config['vy_gains']))
        self.vyaw_pid = PIDController(PIDGains(**self.config['vyaw_gains']))
        
        # 速度估计和滤波
        self.filtered_vx = 0.0
        self.filtered_vy = 0.0
        self.filtered_vz = 0.0
        self.filtered_vyaw = 0.0
        
        # 上一次传感器数据用于计算角速度
        self.last_yaw_deg = None
        self.last_yaw_time = None
        
        self.logger.info("速度控制器初始化完成")
    
    def set_target(self, target: Dict[str, float]) -> bool:
        """
        设置目标速度
        
        Args:
            target: 目标速度字典
                   - 'vz_cm_s': 垂直速度 (cm/s，正值上升)
                   - 'vx_cm_s': 前后速度 (cm/s，正值前进)
                   - 'vy_cm_s': 左右速度 (cm/s，正值右移)
                   - 'vyaw_deg_s': 偏航角速度 (deg/s，正值顺时针)
                   
        Returns:
            bool: 设置是否成功
        """
        if not self._validate_target(target):
            return False
        
        # 设置默认值
        default_target = {
            'vz_cm_s': 0.0,
            'vx_cm_s': 0.0,  
            'vy_cm_s': 0.0,
            'vyaw_deg_s': 0.0
        }
        
        self.target_setpoint = {**default_target, **target}
        
        self.logger.info(f"设置目标速度: {self.target_setpoint}")
        return True
    
    def compute_control(self, sensor_data: SensorData, target: Dict[str, float]) -> ControlCommand:
        """
        计算速度控制指令
        
        Args:
            sensor_data: 传感器数据
            target: 目标速度
            
        Returns:
            ControlCommand: 控制指令
        """
        current_time = time.time()
        dt = current_time - self.last_update_time if self.last_update_time > 0 else 0.02
        
        # 安全检查
        if not self._safety_check(sensor_data):
            return self.emergency_stop()
        
        # 更新速度估计
        self._update_velocity_estimates(sensor_data, dt)
        
        # 计算控制输出
        
        # 1. 垂直速度控制
        throttle = self.vz_pid.compute(
            self.filtered_vz,
            target['vz_cm_s'],
            dt
        )
        
        # 2. 前后速度控制 (转换为俯仰角指令)
        pitch = -self.vx_pid.compute(  # 负号：前进需要负俯仰
            self.filtered_vx,
            target['vx_cm_s'],
            dt
        )
        
        # 3. 左右速度控制 (转换为横滚角指令)
        roll = self.vy_pid.compute(
            self.filtered_vy,
            target['vy_cm_s'],
            dt
        )
        
        # 4. 偏航角速度控制
        yaw = self.vyaw_pid.compute(
            self.filtered_vyaw,
            target['vyaw_deg_s'],
            dt
        )
        
        # 应用安全限制
        throttle, pitch, roll, yaw = self._apply_safety_limits(
            throttle, pitch, roll, yaw, sensor_data
        )
        
        return ControlCommand(
            timestamp=current_time,
            roll=roll,
            pitch=pitch,
            throttle=throttle,
            yaw=yaw
        )
    
    def _update_velocity_estimates(self, sensor_data: SensorData, dt: float):
        """更新速度估计和滤波"""
        # 获取原始速度数据
        raw_vx = sensor_data.vgx_cm_s
        raw_vy = sensor_data.vgy_cm_s  
        raw_vz = sensor_data.vgz_cm_s
        
        # 低通滤波
        alpha = self.config['velocity_filter_alpha']
        self.filtered_vx = alpha * self.filtered_vx + (1 - alpha) * raw_vx
        self.filtered_vy = alpha * self.filtered_vy + (1 - alpha) * raw_vy
        self.filtered_vz = alpha * self.filtered_vz + (1 - alpha) * raw_vz
        
        # 计算偏航角速度
        current_yaw = sensor_data.yaw_deg
        current_time = time.time()
        
        if self.last_yaw_deg is not None and self.last_yaw_time is not None:
            dt_yaw = current_time - self.last_yaw_time
            if dt_yaw > 0:
                # 处理角度跳跃 (-180° 到 180°)
                yaw_diff = current_yaw - self.last_yaw_deg
                if yaw_diff > 180:
                    yaw_diff -= 360
                elif yaw_diff < -180:
                    yaw_diff += 360
                    
                raw_vyaw = yaw_diff / dt_yaw
                
                # 滤波偏航角速度
                self.filtered_vyaw = alpha * self.filtered_vyaw + (1 - alpha) * raw_vyaw
        
        self.last_yaw_deg = current_yaw
        self.last_yaw_time = current_time
    
    def _apply_safety_limits(self, throttle: float, pitch: float, roll: float, yaw: float, sensor_data: SensorData) -> tuple:
        """应用安全限制"""
        
        # 高度安全检查
        current_height = sensor_data.tof_distance_cm if sensor_data.tof_distance_cm > 0 else sensor_data.height_cm
        
        if current_height > self.config['max_altitude_cm']:
            # 超过最大高度，禁止上升
            throttle = min(throttle, 0.0)
            self.logger.warning(f"达到最大高度限制: {current_height}cm")
            
        elif current_height < self.config['min_altitude_cm']:
            # 低于最小高度，禁止下降
            throttle = max(throttle, 0.0)
            self.logger.warning(f"接近最小高度限制: {current_height}cm")
        
        # 速度限制检查
        if abs(self.filtered_vz) > self.config['max_vertical_speed']:
            # 垂直速度过大，减小油门输出
            throttle *= 0.5
            self.logger.warning(f"垂直速度过大: {self.filtered_vz} cm/s")
        
        if abs(self.filtered_vx) > self.config['max_horizontal_speed']:
            # 水平速度过大，减小俯仰输出
            pitch *= 0.5
            self.logger.warning(f"前后速度过大: {self.filtered_vx} cm/s")
        
        if abs(self.filtered_vy) > self.config['max_horizontal_speed']:
            # 侧向速度过大，减小横滚输出  
            roll *= 0.5
            self.logger.warning(f"左右速度过大: {self.filtered_vy} cm/s")
        
        if abs(self.filtered_vyaw) > self.config['max_yaw_rate_deg_s']:
            # 偏航角速度过大，减小偏航输出
            yaw *= 0.5
            self.logger.warning(f"偏航角速度过大: {self.filtered_vyaw} deg/s")
        
        # 电池紧急处理
        if sensor_data.battery_percent < 15:
            # 电池不足，执行紧急下降
            throttle = -30.0
            pitch = roll = yaw = 0.0
            self.logger.critical("电池电量不足，执行紧急下降")
        
        return throttle, pitch, roll, yaw
    
    def _validate_target(self, target: Dict[str, float]) -> bool:
        """验证目标速度参数"""
        
        # 检查速度限制
        if 'vz_cm_s' in target:
            if abs(target['vz_cm_s']) > self.config['max_vertical_speed']:
                self.logger.error(f"垂直速度目标超限: {target['vz_cm_s']} cm/s")
                return False
        
        if 'vx_cm_s' in target:
            if abs(target['vx_cm_s']) > self.config['max_horizontal_speed']:
                self.logger.error(f"前后速度目标超限: {target['vx_cm_s']} cm/s")
                return False
        
        if 'vy_cm_s' in target:
            if abs(target['vy_cm_s']) > self.config['max_horizontal_speed']:
                self.logger.error(f"左右速度目标超限: {target['vy_cm_s']} cm/s")
                return False
        
        if 'vyaw_deg_s' in target:
            if abs(target['vyaw_deg_s']) > self.config['max_yaw_rate_deg_s']:
                self.logger.error(f"偏航角速度目标超限: {target['vyaw_deg_s']} deg/s")
                return False
        
        return True
    
    def _on_reset(self) -> bool:
        """重置控制器状态"""
        self.vz_pid.reset()
        self.vx_pid.reset()
        self.vy_pid.reset()
        self.vyaw_pid.reset()
        
        self.filtered_vx = 0.0
        self.filtered_vy = 0.0
        self.filtered_vz = 0.0
        self.filtered_vyaw = 0.0
        
        self.last_yaw_deg = None
        self.last_yaw_time = None
        
        return True
    
    def get_velocity_estimates(self) -> Dict[str, float]:
        """获取当前速度估计"""
        return {
            'vx_cm_s': self.filtered_vx,
            'vy_cm_s': self.filtered_vy,
            'vz_cm_s': self.filtered_vz,
            'vyaw_deg_s': self.filtered_vyaw
        }
    
    def get_velocity_errors(self) -> Dict[str, float]:
        """获取速度控制误差"""
        if self.target_setpoint is None:
            return {}
        
        return {
            'vx_error_cm_s': self.target_setpoint['vx_cm_s'] - self.filtered_vx,
            'vy_error_cm_s': self.target_setpoint['vy_cm_s'] - self.filtered_vy,
            'vz_error_cm_s': self.target_setpoint['vz_cm_s'] - self.filtered_vz,
            'vyaw_error_deg_s': self.target_setpoint['vyaw_deg_s'] - self.filtered_vyaw
        }
    
    def is_velocity_stable(self, tolerance_multiplier: float = 1.0) -> bool:
        """
        检查速度是否稳定在目标值附近
        
        Args:
            tolerance_multiplier: 容差乘数
            
        Returns:
            bool: 速度是否稳定
        """
        if self.target_setpoint is None:
            return False
        
        # 定义稳定性容差
        vz_tolerance = 5.0 * tolerance_multiplier  # cm/s
        vxy_tolerance = 8.0 * tolerance_multiplier  # cm/s
        vyaw_tolerance = 10.0 * tolerance_multiplier  # deg/s
        
        errors = self.get_velocity_errors()
        
        vz_stable = abs(errors['vz_error_cm_s']) <= vz_tolerance
        vx_stable = abs(errors['vx_error_cm_s']) <= vxy_tolerance
        vy_stable = abs(errors['vy_error_cm_s']) <= vxy_tolerance
        vyaw_stable = abs(errors['vyaw_error_deg_s']) <= vyaw_tolerance
        
        return vz_stable and vx_stable and vy_stable and vyaw_stable
    
    def emergency_brake(self) -> ControlCommand:
        """
        紧急制动 - 快速停止所有运动
        
        Returns:
            ControlCommand: 制动控制指令
        """
        self.logger.warning("执行紧急制动")
        
        # 设置停止目标
        self.set_target({
            'vx_cm_s': 0.0,
            'vy_cm_s': 0.0,
            'vz_cm_s': 0.0,
            'vyaw_deg_s': 0.0
        })
        
        # 返回强制停止指令
        return ControlCommand(
            timestamp=time.time(),
            roll=0.0,
            pitch=0.0,
            throttle=0.0,
            yaw=0.0
        )