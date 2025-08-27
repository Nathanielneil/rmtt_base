"""
基础控制器抽象类

定义了所有控制器必须实现的接口和通用功能
"""

from abc import ABC, abstractmethod
import time
import numpy as np
from typing import Dict, Any, Optional, Tuple
from dataclasses import dataclass
from enum import Enum

from utils.logger import Logger


class ControllerState(Enum):
    """控制器状态枚举"""
    INACTIVE = "inactive"
    ACTIVE = "active"
    ERROR = "error"
    EMERGENCY_STOP = "emergency_stop"


@dataclass
class ControlCommand:
    """控制命令数据结构"""
    timestamp: float
    roll: float      # 横滚角速度 (-100 to 100)
    pitch: float     # 俯仰角速度 (-100 to 100) 
    throttle: float  # 油门/垂直速度 (-100 to 100)
    yaw: float       # 偏航角速度 (-100 to 100)
    
    def __post_init__(self):
        """确保控制指令在有效范围内"""
        self.roll = np.clip(self.roll, -100, 100)
        self.pitch = np.clip(self.pitch, -100, 100)
        self.throttle = np.clip(self.throttle, -100, 100)
        self.yaw = np.clip(self.yaw, -100, 100)


@dataclass  
class SensorData:
    """传感器数据结构"""
    timestamp: float
    
    # 位置信息
    height_cm: float
    tof_distance_cm: float
    barometer_cm: float
    
    # 姿态信息
    pitch_deg: float
    roll_deg: float
    yaw_deg: float
    
    # 速度信息
    vgx_cm_s: float
    vgy_cm_s: float
    vgz_cm_s: float
    
    # 加速度信息
    agx_0001g: float
    agy_0001g: float
    agz_0001g: float
    
    # 系统信息
    battery_percent: float
    temperature_deg: float
    wifi_snr: int


class BaseController(ABC):
    """
    基础控制器抽象类
    
    所有自定义控制器都应该继承这个类并实现必要的抽象方法
    """
    
    def __init__(self, name: str, config: Dict[str, Any] = None):
        """
        初始化控制器
        
        Args:
            name: 控制器名称
            config: 控制器配置参数
        """
        self.name = name
        self.config = config or {}
        self.logger = Logger(f"Controller_{name}")
        
        # 控制器状态
        self.state = ControllerState.INACTIVE
        self.last_update_time = 0.0
        self.control_frequency = self.config.get('control_frequency', 50.0)  # Hz
        
        # 数据缓存
        self.current_sensor_data: Optional[SensorData] = None
        self.target_setpoint: Optional[Dict[str, float]] = None
        self.last_control_command: Optional[ControlCommand] = None
        
        # 性能统计
        self.total_iterations = 0
        self.total_control_time = 0.0
        self.max_control_time = 0.0
        
        self.logger.info(f"控制器 {name} 初始化完成")
    
    @abstractmethod
    def compute_control(self, sensor_data: SensorData, target: Dict[str, float]) -> ControlCommand:
        """
        计算控制指令 - 抽象方法，必须由子类实现
        
        Args:
            sensor_data: 当前传感器数据
            target: 目标设定点
            
        Returns:
            ControlCommand: 控制指令
        """
        pass
    
    @abstractmethod
    def set_target(self, target: Dict[str, float]) -> bool:
        """
        设置控制目标 - 抽象方法，必须由子类实现
        
        Args:
            target: 目标参数字典
            
        Returns:
            bool: 设置是否成功
        """
        pass
    
    def activate(self) -> bool:
        """
        激活控制器
        
        Returns:
            bool: 激活是否成功
        """
        try:
            if self.state == ControllerState.INACTIVE:
                success = self._on_activate()
                if success:
                    self.state = ControllerState.ACTIVE
                    self.logger.info(f"控制器 {self.name} 已激活")
                    return True
                else:
                    self.logger.error(f"控制器 {self.name} 激活失败")
                    return False
            else:
                self.logger.warning(f"控制器 {self.name} 已经处于 {self.state.value} 状态")
                return False
        except Exception as e:
            self.logger.error(f"控制器激活异常: {e}")
            self.state = ControllerState.ERROR
            return False
    
    def deactivate(self) -> bool:
        """
        停用控制器
        
        Returns:
            bool: 停用是否成功
        """
        try:
            if self.state == ControllerState.ACTIVE:
                success = self._on_deactivate()
                if success:
                    self.state = ControllerState.INACTIVE
                    self.logger.info(f"控制器 {self.name} 已停用")
                    return True
                else:
                    self.logger.error(f"控制器 {self.name} 停用失败")
                    return False
            else:
                self.logger.warning(f"控制器 {self.name} 当前状态: {self.state.value}")
                return False
        except Exception as e:
            self.logger.error(f"控制器停用异常: {e}")
            self.state = ControllerState.ERROR
            return False
    
    def update(self, sensor_data: SensorData) -> Optional[ControlCommand]:
        """
        更新控制器并计算控制指令
        
        Args:
            sensor_data: 当前传感器数据
            
        Returns:
            Optional[ControlCommand]: 控制指令，如果控制器未激活则返回None
        """
        if self.state != ControllerState.ACTIVE:
            return None
            
        if self.target_setpoint is None:
            self.logger.warning(f"控制器 {self.name} 没有设置目标")
            return None
        
        start_time = time.time()
        
        try:
            # 更新传感器数据
            self.current_sensor_data = sensor_data
            
            # 计算控制指令
            control_command = self.compute_control(sensor_data, self.target_setpoint)
            
            # 记录控制指令
            self.last_control_command = control_command
            self.last_update_time = time.time()
            
            # 性能统计
            control_time = time.time() - start_time
            self.total_control_time += control_time
            self.total_iterations += 1
            self.max_control_time = max(self.max_control_time, control_time)
            
            return control_command
            
        except Exception as e:
            self.logger.error(f"控制器更新异常: {e}")
            self.state = ControllerState.ERROR
            return None
    
    def emergency_stop(self) -> ControlCommand:
        """
        紧急停止，返回零控制指令
        
        Returns:
            ControlCommand: 零控制指令
        """
        self.state = ControllerState.EMERGENCY_STOP
        self.logger.critical(f"控制器 {self.name} 触发紧急停止")
        
        return ControlCommand(
            timestamp=time.time(),
            roll=0.0,
            pitch=0.0, 
            throttle=0.0,
            yaw=0.0
        )
    
    def get_status(self) -> Dict[str, Any]:
        """
        获取控制器状态信息
        
        Returns:
            Dict[str, Any]: 状态信息字典
        """
        avg_control_time = (
            self.total_control_time / self.total_iterations 
            if self.total_iterations > 0 else 0.0
        )
        
        return {
            'name': self.name,
            'state': self.state.value,
            'last_update_time': self.last_update_time,
            'target_setpoint': self.target_setpoint,
            'performance': {
                'total_iterations': self.total_iterations,
                'avg_control_time_ms': avg_control_time * 1000,
                'max_control_time_ms': self.max_control_time * 1000,
                'control_frequency_hz': self.control_frequency
            },
            'has_sensor_data': self.current_sensor_data is not None,
            'last_control_command': {
                'roll': self.last_control_command.roll if self.last_control_command else None,
                'pitch': self.last_control_command.pitch if self.last_control_command else None,
                'throttle': self.last_control_command.throttle if self.last_control_command else None,
                'yaw': self.last_control_command.yaw if self.last_control_command else None,
            }
        }
    
    def reset(self) -> bool:
        """
        重置控制器状态
        
        Returns:
            bool: 重置是否成功
        """
        try:
            self.state = ControllerState.INACTIVE
            self.current_sensor_data = None
            self.target_setpoint = None
            self.last_control_command = None
            
            # 重置性能统计
            self.total_iterations = 0
            self.total_control_time = 0.0
            self.max_control_time = 0.0
            
            success = self._on_reset()
            
            self.logger.info(f"控制器 {self.name} 已重置")
            return success
            
        except Exception as e:
            self.logger.error(f"控制器重置异常: {e}")
            self.state = ControllerState.ERROR
            return False
    
    # 可重写的钩子方法
    def _on_activate(self) -> bool:
        """激活时调用的钩子方法，子类可重写"""
        return True
    
    def _on_deactivate(self) -> bool:
        """停用时调用的钩子方法，子类可重写"""
        return True
    
    def _on_reset(self) -> bool:
        """重置时调用的钩子方法，子类可重写"""
        return True
    
    def _validate_target(self, target: Dict[str, float]) -> bool:
        """验证目标参数的有效性，子类可重写"""
        return True
    
    def _safety_check(self, sensor_data: SensorData) -> bool:
        """安全检查，子类可重写"""
        # 基础安全检查
        if sensor_data.battery_percent < 20:
            self.logger.warning("电池电量过低")
            return False
            
        return True