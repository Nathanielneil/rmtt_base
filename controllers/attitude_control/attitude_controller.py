"""
姿态控制器

实现直接的姿态角控制，用于精确控制无人机的roll、pitch、yaw角度
"""

import time
import numpy as np
from typing import Dict, Any
from dataclasses import dataclass

from ..base_controller import BaseController, SensorData, ControlCommand
from ..position_control.position_controller import PIDController, PIDGains


class AttitudeController(BaseController):
    """
    姿态控制器
    
    直接控制无人机的姿态角，包括：
    - Roll角控制 (横滚角)
    - Pitch角控制 (俯仰角)  
    - Yaw角控制 (偏航角)
    - 高度保持 (辅助功能)
    
    适用场景：
    - 精确的姿态控制需求
    - 视觉伺服控制
    - 特殊飞行动作
    - 姿态稳定测试
    """
    
    def __init__(self, name: str = "attitude_controller", config: Dict[str, Any] = None):
        super().__init__(name, config)
        
        # 默认配置
        default_config = {
            # 姿态角PID参数
            'roll_gains': {'kp': 1.5, 'ki': 0.1, 'kd': 0.2, 'max_output': 80.0},
            'pitch_gains': {'kp': 1.5, 'ki': 0.1, 'kd': 0.2, 'max_output': 80.0},
            'yaw_gains': {'kp': 2.0, 'ki': 0.15, 'kd': 0.3, 'max_output': 90.0},
            
            # 高度保持PID参数 (如果启用)
            'altitude_gains': {'kp': 0.8, 'ki': 0.05, 'kd': 0.25, 'max_output': 60.0},
            
            # 角度限制 (度)
            'max_roll_deg': 30.0,
            'max_pitch_deg': 30.0,
            'max_yaw_rate_deg_s': 120.0,
            
            # 容差 (度)  
            'roll_tolerance_deg': 2.0,
            'pitch_tolerance_deg': 2.0,
            'yaw_tolerance_deg': 3.0,
            
            # 功能开关
            'altitude_hold_enabled': True,      # 是否启用高度保持
            'angle_rate_limit_enabled': True,   # 是否启用角速度限制
            
            # 安全参数
            'max_angle_error_deg': 45.0,       # 最大允许角度误差
            'attitude_timeout_s': 5.0,         # 姿态控制超时时间
        }
        
        # 合并配置
        for key, value in default_config.items():
            if key not in self.config:
                self.config[key] = value
        
        # 创建PID控制器
        self.roll_pid = PIDController(PIDGains(**self.config['roll_gains']))
        self.pitch_pid = PIDController(PIDGains(**self.config['pitch_gains']))
        self.yaw_pid = PIDController(PIDGains(**self.config['yaw_gains']))
        
        # 高度保持控制器
        if self.config['altitude_hold_enabled']:
            self.altitude_pid = PIDController(PIDGains(**self.config['altitude_gains']))
            self.hold_altitude_cm = None
        
        # 角速度限制器状态
        self.last_roll_command = 0.0
        self.last_pitch_command = 0.0
        self.last_yaw_command = 0.0
        
        # 姿态控制超时检测
        self.attitude_command_time = None
        
        self.logger.info("姿态控制器初始化完成")
    
    def set_target(self, target: Dict[str, float]) -> bool:
        """
        设置目标姿态角
        
        Args:
            target: 目标姿态字典
                   - 'roll_deg': 目标横滚角 (度)
                   - 'pitch_deg': 目标俯仰角 (度)  
                   - 'yaw_deg': 目标偏航角 (度)
                   - 'hold_altitude_cm': 保持高度 (厘米，可选)
                   
        Returns:
            bool: 设置是否成功
        """
        if not self._validate_target(target):
            return False
        
        # 设置默认值 - 保持当前姿态
        default_target = {
            'roll_deg': 0.0,
            'pitch_deg': 0.0, 
            'yaw_deg': 0.0 if self.current_sensor_data is None else self.current_sensor_data.yaw_deg
        }
        
        self.target_setpoint = {**default_target, **target}
        
        # 设置高度保持
        if self.config['altitude_hold_enabled']:
            if 'hold_altitude_cm' in target:
                self.hold_altitude_cm = target['hold_altitude_cm']
            elif self.current_sensor_data is not None and self.hold_altitude_cm is None:
                # 自动设置当前高度为保持高度
                current_height = (
                    self.current_sensor_data.tof_distance_cm 
                    if self.current_sensor_data.tof_distance_cm > 0 
                    else self.current_sensor_data.height_cm
                )
                self.hold_altitude_cm = current_height
        
        self.attitude_command_time = time.time()
        
        self.logger.info(f"设置目标姿态: {self.target_setpoint}")
        if self.hold_altitude_cm is not None:
            self.logger.info(f"高度保持: {self.hold_altitude_cm}cm")
            
        return True
    
    def compute_control(self, sensor_data: SensorData, target: Dict[str, float]) -> ControlCommand:
        """
        计算姿态控制指令
        
        Args:
            sensor_data: 传感器数据
            target: 目标姿态
            
        Returns:
            ControlCommand: 控制指令
        """
        current_time = time.time()
        dt = current_time - self.last_update_time if self.last_update_time > 0 else 0.02
        
        # 安全检查
        if not self._safety_check(sensor_data):
            return self.emergency_stop()
        
        # 超时检查
        if self._check_attitude_timeout(current_time):
            self.logger.warning("姿态控制超时，切换到稳定模式")
            return self._stable_attitude_command()
        
        # 获取当前姿态
        current_roll = sensor_data.roll_deg
        current_pitch = sensor_data.pitch_deg
        current_yaw = sensor_data.yaw_deg
        
        # 计算姿态角控制输出
        roll_command = self.roll_pid.compute(
            current_roll,
            target['roll_deg'],
            dt
        )
        
        pitch_command = self.pitch_pid.compute(
            current_pitch,
            target['pitch_deg'],
            dt
        )
        
        # 偏航角处理 - 考虑角度跳跃
        yaw_error = self._calculate_yaw_error(current_yaw, target['yaw_deg'])
        yaw_command = self.yaw_pid.compute(
            0.0,  # 当前误差为0
            yaw_error,  # 目标为误差值
            dt
        )
        
        # 高度控制 (如果启用)
        throttle_command = 0.0
        if self.config['altitude_hold_enabled'] and self.hold_altitude_cm is not None:
            current_height = (
                sensor_data.tof_distance_cm 
                if sensor_data.tof_distance_cm > 0 
                else sensor_data.height_cm
            )
            
            throttle_command = self.altitude_pid.compute(
                current_height,
                self.hold_altitude_cm,
                dt
            )
        
        # 应用角速度限制
        if self.config['angle_rate_limit_enabled']:
            roll_command = self._apply_rate_limit(roll_command, self.last_roll_command, dt, 'roll')
            pitch_command = self._apply_rate_limit(pitch_command, self.last_pitch_command, dt, 'pitch') 
            yaw_command = self._apply_rate_limit(yaw_command, self.last_yaw_command, dt, 'yaw')
        
        # 保存指令历史
        self.last_roll_command = roll_command
        self.last_pitch_command = pitch_command
        self.last_yaw_command = yaw_command
        
        # 应用安全限制
        roll_command, pitch_command, throttle_command, yaw_command = self._apply_safety_limits(
            roll_command, pitch_command, throttle_command, yaw_command, sensor_data
        )
        
        return ControlCommand(
            timestamp=current_time,
            roll=roll_command,
            pitch=pitch_command,
            throttle=throttle_command,
            yaw=yaw_command
        )
    
    def _calculate_yaw_error(self, current_yaw: float, target_yaw: float) -> float:
        """
        计算偏航角误差，处理角度跳跃问题
        
        Args:
            current_yaw: 当前偏航角
            target_yaw: 目标偏航角
            
        Returns:
            float: 偏航角误差
        """
        error = target_yaw - current_yaw
        
        # 处理角度跳跃 (-180° 到 180°)
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360
            
        return error
    
    def _apply_rate_limit(self, new_command: float, last_command: float, dt: float, axis: str) -> float:
        """
        应用角速度限制
        
        Args:
            new_command: 新的控制指令
            last_command: 上一次控制指令
            dt: 时间间隔
            axis: 控制轴 ('roll', 'pitch', 'yaw')
            
        Returns:
            float: 限制后的控制指令
        """
        if dt <= 0:
            return new_command
        
        # 计算指令变化率
        command_rate = (new_command - last_command) / dt
        
        # 根据轴设置最大变化率
        if axis in ['roll', 'pitch']:
            max_rate = 200.0  # deg/s equivalent in command units
        else:  # yaw
            max_rate = self.config['max_yaw_rate_deg_s'] * 2  # Convert to command units
        
        # 限制变化率
        if abs(command_rate) > max_rate:
            limited_rate = max_rate if command_rate > 0 else -max_rate
            limited_command = last_command + limited_rate * dt
            
            if axis == 'yaw':
                self.logger.debug(f"{axis}轴角速度限制: {command_rate:.1f} -> {limited_rate:.1f}")
            
            return limited_command
        
        return new_command
    
    def _apply_safety_limits(self, roll: float, pitch: float, throttle: float, yaw: float, sensor_data: SensorData) -> tuple:
        """应用安全限制"""
        
        # 角度安全限制
        current_roll = abs(sensor_data.roll_deg)
        current_pitch = abs(sensor_data.pitch_deg)
        
        if current_roll > self.config['max_roll_deg']:
            roll *= 0.5  # 减小横滚指令
            self.logger.warning(f"横滚角过大: {current_roll}°")
        
        if current_pitch > self.config['max_pitch_deg']:
            pitch *= 0.5  # 减小俯仰指令
            self.logger.warning(f"俯仰角过大: {current_pitch}°")
        
        # 角度误差安全检查
        if self.target_setpoint is not None:
            roll_error = abs(sensor_data.roll_deg - self.target_setpoint['roll_deg'])
            pitch_error = abs(sensor_data.pitch_deg - self.target_setpoint['pitch_deg'])
            
            if roll_error > self.config['max_angle_error_deg']:
                roll = 0.0
                self.logger.error(f"横滚角误差过大: {roll_error}°，停止控制")
            
            if pitch_error > self.config['max_angle_error_deg']:
                pitch = 0.0
                self.logger.error(f"俯仰角误差过大: {pitch_error}°，停止控制")
        
        # 高度安全
        if self.config['altitude_hold_enabled']:
            current_height = (
                sensor_data.tof_distance_cm 
                if sensor_data.tof_distance_cm > 0 
                else sensor_data.height_cm
            )
            
            if current_height < 15:  # 过低
                throttle = max(throttle, 20.0)
                self.logger.warning(f"高度过低: {current_height}cm，强制上升")
            elif current_height > 350:  # 过高
                throttle = min(throttle, -20.0)
                self.logger.warning(f"高度过高: {current_height}cm，强制下降")
        
        # 电池安全
        if sensor_data.battery_percent < 20:
            # 电池不足时，限制大幅度动作
            roll = np.clip(roll, -30, 30)
            pitch = np.clip(pitch, -30, 30)
            yaw = np.clip(yaw, -30, 30)
            
            if sensor_data.battery_percent < 10:
                self.logger.critical("电池电量极低，建议立即降落")
        
        return roll, pitch, throttle, yaw
    
    def _check_attitude_timeout(self, current_time: float) -> bool:
        """检查姿态控制是否超时"""
        if self.attitude_command_time is None:
            return False
            
        return (current_time - self.attitude_command_time) > self.config['attitude_timeout_s']
    
    def _stable_attitude_command(self) -> ControlCommand:
        """生成稳定姿态指令 (水平飞行)"""
        return ControlCommand(
            timestamp=time.time(),
            roll=0.0,
            pitch=0.0,
            throttle=0.0,  # 保持当前高度
            yaw=0.0
        )
    
    def _validate_target(self, target: Dict[str, float]) -> bool:
        """验证目标姿态参数"""
        
        # 检查角度限制
        if 'roll_deg' in target:
            if abs(target['roll_deg']) > self.config['max_roll_deg']:
                self.logger.error(f"横滚角目标超限: {target['roll_deg']}°")
                return False
        
        if 'pitch_deg' in target:
            if abs(target['pitch_deg']) > self.config['max_pitch_deg']:
                self.logger.error(f"俯仰角目标超限: {target['pitch_deg']}°")
                return False
        
        # 偏航角可以是任意值，会自动处理角度跳跃
        
        return True
    
    def _on_reset(self) -> bool:
        """重置控制器状态"""
        self.roll_pid.reset()
        self.pitch_pid.reset()
        self.yaw_pid.reset()
        
        if self.config['altitude_hold_enabled']:
            self.altitude_pid.reset()
            
        self.hold_altitude_cm = None
        self.last_roll_command = 0.0
        self.last_pitch_command = 0.0
        self.last_yaw_command = 0.0
        self.attitude_command_time = None
        
        return True
    
    def is_attitude_stable(self, tolerance_multiplier: float = 1.0) -> bool:
        """
        检查姿态是否稳定在目标值附近
        
        Args:
            tolerance_multiplier: 容差乘数
            
        Returns:
            bool: 姿态是否稳定
        """
        if self.target_setpoint is None or self.current_sensor_data is None:
            return False
        
        # 计算角度误差
        roll_error = abs(self.current_sensor_data.roll_deg - self.target_setpoint['roll_deg'])
        pitch_error = abs(self.current_sensor_data.pitch_deg - self.target_setpoint['pitch_deg'])
        yaw_error = abs(self._calculate_yaw_error(
            self.current_sensor_data.yaw_deg, 
            self.target_setpoint['yaw_deg']
        ))
        
        # 检查是否在容差范围内
        roll_stable = roll_error <= (self.config['roll_tolerance_deg'] * tolerance_multiplier)
        pitch_stable = pitch_error <= (self.config['pitch_tolerance_deg'] * tolerance_multiplier)
        yaw_stable = yaw_error <= (self.config['yaw_tolerance_deg'] * tolerance_multiplier)
        
        return roll_stable and pitch_stable and yaw_stable
    
    def get_attitude_errors(self) -> Dict[str, float]:
        """获取当前姿态误差"""
        if self.target_setpoint is None or self.current_sensor_data is None:
            return {}
        
        return {
            'roll_error_deg': self.target_setpoint['roll_deg'] - self.current_sensor_data.roll_deg,
            'pitch_error_deg': self.target_setpoint['pitch_deg'] - self.current_sensor_data.pitch_deg,
            'yaw_error_deg': self._calculate_yaw_error(
                self.current_sensor_data.yaw_deg, 
                self.target_setpoint['yaw_deg']
            )
        }
    
    def set_attitude_hold_mode(self, enable: bool = True) -> bool:
        """
        启用/禁用姿态保持模式
        
        Args:
            enable: 是否启用姿态保持
            
        Returns:
            bool: 设置是否成功
        """
        if enable and self.current_sensor_data is not None:
            # 设置当前姿态为目标姿态
            return self.set_target({
                'roll_deg': self.current_sensor_data.roll_deg,
                'pitch_deg': self.current_sensor_data.pitch_deg,
                'yaw_deg': self.current_sensor_data.yaw_deg
            })
        elif not enable:
            # 禁用控制器
            return self.deactivate()
        
        return False