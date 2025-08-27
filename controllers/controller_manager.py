"""
控制器管理器

统一管理和协调多个控制器的工作，提供控制器切换、数据分发、安全监控等功能
"""

import time
import threading
from typing import Dict, Any, Optional, List, Callable
from dataclasses import dataclass
from enum import Enum

from .base_controller import BaseController, SensorData, ControlCommand, ControllerState
from utils.logger import Logger


class ControlMode(Enum):
    """控制模式枚举"""
    MANUAL = "manual"              # 手动控制
    ATTITUDE = "attitude"          # 姿态控制
    POSITION = "position"          # 位置控制
    VELOCITY = "velocity"          # 速度控制
    TRAJECTORY = "trajectory"      # 轨迹控制
    EMERGENCY = "emergency"        # 紧急模式


@dataclass
class ControllerManagerConfig:
    """控制器管理器配置"""
    update_frequency_hz: float = 50.0          # 控制循环频率
    controller_timeout_s: float = 1.0          # 控制器超时时间
    safety_check_enabled: bool = True          # 启用安全检查
    auto_fallback_enabled: bool = True         # 启用自动回退
    fallback_controller: str = "attitude"      # 默认回退控制器
    data_logging_enabled: bool = True          # 启用数据日志记录
    max_control_error_count: int = 5           # 最大控制错误次数


class ControllerManager:
    """
    控制器管理器
    
    功能：
    - 管理多个控制器的生命周期
    - 控制器间的切换和协调
    - 统一的传感器数据分发
    - 控制指令的聚合和输出
    - 安全监控和故障处理
    - 性能监控和日志记录
    """
    
    def __init__(self, config: Optional[ControllerManagerConfig] = None):
        """
        初始化控制器管理器
        
        Args:
            config: 管理器配置，为None时使用默认配置
        """
        self.config = config or ControllerManagerConfig()
        self.logger = Logger("ControllerManager")
        
        # 控制器注册表
        self.controllers: Dict[str, BaseController] = {}
        self.active_controller: Optional[BaseController] = None
        self.current_control_mode = ControlMode.MANUAL
        
        # 运行状态
        self.is_running = False
        self.control_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        
        # 数据管理
        self.latest_sensor_data: Optional[SensorData] = None
        self.latest_control_command: Optional[ControlCommand] = None
        self.sensor_data_callbacks: List[Callable[[SensorData], None]] = []
        self.control_command_callbacks: List[Callable[[ControlCommand], None]] = []
        
        # 性能统计
        self.total_control_cycles = 0
        self.total_control_time = 0.0
        self.max_control_time = 0.0
        self.error_count = 0
        self.last_control_time = 0.0
        
        # 安全监控
        self.controller_errors: Dict[str, int] = {}
        self.last_successful_control_time = time.time()
        
        self.logger.info("控制器管理器初始化完成")
    
    def register_controller(self, name: str, controller: BaseController) -> bool:
        """
        注册控制器
        
        Args:
            name: 控制器名称
            controller: 控制器实例
            
        Returns:
            bool: 注册是否成功
        """
        if name in self.controllers:
            self.logger.warning(f"控制器 '{name}' 已存在，将被替换")
        
        try:
            self.controllers[name] = controller
            self.controller_errors[name] = 0
            self.logger.info(f"控制器 '{name}' 注册成功")
            return True
        except Exception as e:
            self.logger.error(f"注册控制器 '{name}' 失败: {e}")
            return False
    
    def unregister_controller(self, name: str) -> bool:
        """
        注销控制器
        
        Args:
            name: 控制器名称
            
        Returns:
            bool: 注销是否成功
        """
        if name not in self.controllers:
            self.logger.warning(f"控制器 '{name}' 不存在")
            return False
        
        try:
            # 如果是当前激活控制器，先切换到手动模式
            if self.active_controller == self.controllers[name]:
                self.switch_control_mode(ControlMode.MANUAL)
            
            # 停用并删除控制器
            controller = self.controllers[name]
            controller.deactivate()
            controller.reset()
            
            del self.controllers[name]
            del self.controller_errors[name]
            
            self.logger.info(f"控制器 '{name}' 注销成功")
            return True
        except Exception as e:
            self.logger.error(f"注销控制器 '{name}' 失败: {e}")
            return False
    
    def switch_control_mode(self, mode: ControlMode, controller_name: Optional[str] = None) -> bool:
        """
        切换控制模式
        
        Args:
            mode: 目标控制模式
            controller_name: 指定控制器名称（可选）
            
        Returns:
            bool: 切换是否成功
        """
        try:
            # 停用当前控制器
            if self.active_controller is not None:
                self.active_controller.deactivate()
                self.active_controller = None
            
            self.current_control_mode = mode
            
            if mode == ControlMode.MANUAL:
                self.logger.info("切换到手动控制模式")
                return True
            elif mode == ControlMode.EMERGENCY:
                self.logger.critical("切换到紧急控制模式")
                return True
            
            # 选择控制器
            if controller_name:
                target_controller_name = controller_name
            else:
                # 根据控制模式自动选择控制器
                mode_controller_map = {
                    ControlMode.ATTITUDE: "attitude",
                    ControlMode.POSITION: "position",
                    ControlMode.VELOCITY: "velocity",
                    ControlMode.TRAJECTORY: "trajectory"
                }
                target_controller_name = mode_controller_map.get(mode)
            
            if not target_controller_name or target_controller_name not in self.controllers:
                self.logger.error(f"未找到适合模式 {mode.value} 的控制器")
                return False
            
            # 激活目标控制器
            target_controller = self.controllers[target_controller_name]
            if target_controller.activate():
                self.active_controller = target_controller
                self.logger.info(f"切换到 {mode.value} 控制模式，使用控制器 '{target_controller_name}'")
                return True
            else:
                self.logger.error(f"激活控制器 '{target_controller_name}' 失败")
                return False
                
        except Exception as e:
            self.logger.error(f"切换控制模式失败: {e}")
            return False
    
    def start_control_loop(self) -> bool:
        """
        启动控制循环
        
        Returns:
            bool: 启动是否成功
        """
        if self.is_running:
            self.logger.warning("控制循环已在运行")
            return False
        
        try:
            self.is_running = True
            self._stop_event.clear()
            
            self.control_thread = threading.Thread(
                target=self._control_loop,
                daemon=True,
                name="ControllerManager"
            )
            self.control_thread.start()
            
            self.logger.info("控制循环已启动")
            return True
        except Exception as e:
            self.logger.error(f"启动控制循环失败: {e}")
            self.is_running = False
            return False
    
    def stop_control_loop(self) -> bool:
        """
        停止控制循环
        
        Returns:
            bool: 停止是否成功
        """
        if not self.is_running:
            return True
        
        try:
            self.is_running = False
            self._stop_event.set()
            
            if self.control_thread and self.control_thread.is_alive():
                self.control_thread.join(timeout=5.0)
            
            # 停用所有控制器
            if self.active_controller:
                self.active_controller.deactivate()
                self.active_controller = None
            
            self.logger.info("控制循环已停止")
            return True
        except Exception as e:
            self.logger.error(f"停止控制循环失败: {e}")
            return False
    
    def update_sensor_data(self, sensor_data: SensorData) -> None:
        """
        更新传感器数据
        
        Args:
            sensor_data: 新的传感器数据
        """
        self.latest_sensor_data = sensor_data
        
        # 调用传感器数据回调
        for callback in self.sensor_data_callbacks:
            try:
                callback(sensor_data)
            except Exception as e:
                self.logger.error(f"传感器数据回调执行失败: {e}")
    
    def get_control_command(self) -> Optional[ControlCommand]:
        """
        获取最新的控制指令
        
        Returns:
            Optional[ControlCommand]: 控制指令，如果无可用指令则返回None
        """
        return self.latest_control_command
    
    def _control_loop(self) -> None:
        """主控制循环"""
        self.logger.info("控制循环开始运行")
        
        loop_period = 1.0 / self.config.update_frequency_hz
        
        while self.is_running and not self._stop_event.is_set():
            loop_start_time = time.time()
            
            try:
                self._execute_control_cycle()
            except Exception as e:
                self.logger.error(f"控制循环异常: {e}")
                self.error_count += 1
                
                if self.error_count > self.config.max_control_error_count:
                    self.logger.critical("控制循环错误次数过多，切换到紧急模式")
                    self.switch_control_mode(ControlMode.EMERGENCY)
            
            # 控制循环频率
            elapsed = time.time() - loop_start_time
            remaining = max(0, loop_period - elapsed)
            
            if remaining > 0:
                time.sleep(remaining)
        
        self.logger.info("控制循环已退出")
    
    def _execute_control_cycle(self) -> None:
        """执行单次控制循环"""
        if self.latest_sensor_data is None:
            return
        
        cycle_start_time = time.time()
        
        # 安全检查
        if self.config.safety_check_enabled:
            if not self._perform_safety_checks():
                return
        
        # 生成控制指令
        control_command = None
        
        if self.current_control_mode == ControlMode.MANUAL:
            # 手动模式：不产生控制指令
            control_command = ControlCommand(
                timestamp=time.time(),
                roll=0.0, pitch=0.0, throttle=0.0, yaw=0.0
            )
        elif self.current_control_mode == ControlMode.EMERGENCY:
            # 紧急模式：全零控制指令
            control_command = ControlCommand(
                timestamp=time.time(),
                roll=0.0, pitch=0.0, throttle=0.0, yaw=0.0
            )
        elif self.active_controller is not None:
            # 使用激活控制器
            try:
                control_command = self.active_controller.update(self.latest_sensor_data)
                if control_command is not None:
                    self.controller_errors[self.active_controller.name] = 0
                    self.last_successful_control_time = time.time()
            except Exception as e:
                controller_name = self.active_controller.name
                self.controller_errors[controller_name] += 1
                self.logger.error(f"控制器 '{controller_name}' 更新失败: {e}")
                
                # 自动回退处理
                if (self.config.auto_fallback_enabled and 
                    self.controller_errors[controller_name] > 3):
                    self.logger.warning(f"控制器 '{controller_name}' 错误过多，执行回退")
                    self._perform_controller_fallback()
        
        # 更新控制指令
        if control_command is not None:
            self.latest_control_command = control_command
            
            # 调用控制指令回调
            for callback in self.control_command_callbacks:
                try:
                    callback(control_command)
                except Exception as e:
                    self.logger.error(f"控制指令回调执行失败: {e}")
        
        # 更新性能统计
        cycle_time = time.time() - cycle_start_time
        self.total_control_time += cycle_time
        self.total_control_cycles += 1
        self.max_control_time = max(self.max_control_time, cycle_time)
        self.last_control_time = time.time()
    
    def _perform_safety_checks(self) -> bool:
        """执行安全检查"""
        if self.latest_sensor_data is None:
            return False
        
        # 检查控制器超时
        current_time = time.time()
        if (current_time - self.last_successful_control_time > self.config.controller_timeout_s):
            self.logger.warning("控制器超时，切换到紧急模式")
            self.switch_control_mode(ControlMode.EMERGENCY)
            return False
        
        # 检查电池电量
        if self.latest_sensor_data.battery_percent < 10:
            self.logger.critical("电池电量极低，切换到紧急模式")
            self.switch_control_mode(ControlMode.EMERGENCY)
            return False
        
        # 检查传感器数据有效性
        if (abs(self.latest_sensor_data.pitch_deg) > 60 or 
            abs(self.latest_sensor_data.roll_deg) > 60):
            self.logger.error("无人机姿态角异常，切换到紧急模式")
            self.switch_control_mode(ControlMode.EMERGENCY)
            return False
        
        return True
    
    def _perform_controller_fallback(self) -> None:
        """执行控制器回退"""
        fallback_controller = self.config.fallback_controller
        
        if fallback_controller in self.controllers:
            self.logger.info(f"回退到控制器: {fallback_controller}")
            mode_map = {
                "attitude": ControlMode.ATTITUDE,
                "position": ControlMode.POSITION,
                "velocity": ControlMode.VELOCITY
            }
            fallback_mode = mode_map.get(fallback_controller, ControlMode.MANUAL)
            self.switch_control_mode(fallback_mode, fallback_controller)
        else:
            self.logger.warning("回退控制器不存在，切换到手动模式")
            self.switch_control_mode(ControlMode.MANUAL)
    
    def register_sensor_data_callback(self, callback: Callable[[SensorData], None]) -> None:
        """注册传感器数据回调函数"""
        self.sensor_data_callbacks.append(callback)
    
    def register_control_command_callback(self, callback: Callable[[ControlCommand], None]) -> None:
        """注册控制指令回调函数"""
        self.control_command_callbacks.append(callback)
    
    def get_status(self) -> Dict[str, Any]:
        """获取管理器状态"""
        avg_control_time = (
            self.total_control_time / self.total_control_cycles 
            if self.total_control_cycles > 0 else 0.0
        )
        
        return {
            'is_running': self.is_running,
            'current_control_mode': self.current_control_mode.value,
            'active_controller': self.active_controller.name if self.active_controller else None,
            'registered_controllers': list(self.controllers.keys()),
            'performance': {
                'total_cycles': self.total_control_cycles,
                'avg_cycle_time_ms': avg_control_time * 1000,
                'max_cycle_time_ms': self.max_control_time * 1000,
                'update_frequency_hz': self.config.update_frequency_hz,
                'error_count': self.error_count
            },
            'controller_errors': self.controller_errors.copy(),
            'has_sensor_data': self.latest_sensor_data is not None,
            'has_control_command': self.latest_control_command is not None,
            'last_control_time': self.last_control_time
        }
    
    def get_controller_list(self) -> List[Dict[str, Any]]:
        """获取所有已注册控制器的信息"""
        controller_info = []
        
        for name, controller in self.controllers.items():
            status = controller.get_status()
            controller_info.append({
                'name': name,
                'type': type(controller).__name__,
                'state': status['state'],
                'is_active': controller == self.active_controller,
                'error_count': self.controller_errors.get(name, 0),
                'performance': status.get('performance', {})
            })
        
        return controller_info