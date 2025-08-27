"""
RMTT系统适配器

将降落实验控制器适配到RMTT系统，处理坐标系转换和传感器数据融合
"""

import numpy as np
import time
import logging
from typing import Dict, Any, Optional, Union
from enum import Enum

from .landing_state import DesiredState, CurrentState, ControlOutput
from .pid_controller import PIDController
from .ude_controller import UDEController
from .adrc_controller import ADRCController


class ControllerType(Enum):
    """控制器类型枚举 - 对应C++代码中的controller_type参数"""
    PID = 0
    UDE = 1
    ADRC = 2


class RMTTAdapter:
    """RMTT系统适配器类"""
    
    def __init__(self, controller_type: ControllerType = ControllerType.ADRC):
        self.controller_type = controller_type
        self.controller = None
        self.logger = logging.getLogger("RMTTAdapter")
        
        # 状态估计参数
        self.position_estimate = np.zeros(3)  # 位置估计 [x, y, z]
        self.velocity_estimate = np.zeros(3)  # 速度估计 [vx, vy, vz]
        self.last_position = None
        self.last_time = None
        
        # 坐标系转换参数
        self.takeoff_position = None  # 起飞位置作为原点
        self.initialized = False
        
        self.logger.info(f"RMTT Adapter initialized with {controller_type.name} controller")
    
    def init_controller(self, params: Dict[str, Any]):
        """初始化控制器 - 使用与原始launch文件相同的参数"""
        
        if self.controller_type == ControllerType.PID:
            self.controller = PIDController()
            # PID参数配置
            pid_params = {
                "quad_mass": params.get("quad_mass", 1.0),
                "hov_percent": params.get("hov_percent", 0.5),
                "Kp_xy": params.get("Kp_xy", 2.0),
                "Kp_z": params.get("Kp_z", 2.0),
                "Kv_xy": params.get("Kv_xy", 2.0),
                "Kv_z": params.get("Kv_z", 2.0),
                "Kvi_xy": params.get("Kvi_xy", 0.3),
                "Kvi_z": params.get("Kvi_z", 0.3),
                "tilt_angle_max": params.get("tilt_angle_max", 10.0),
                "pxy_int_max": params.get("pxy_int_max", 0.5),
                "pz_int_max": params.get("pz_int_max", 0.5)
            }
            self.controller.init(pid_params)
            
        elif self.controller_type == ControllerType.UDE:
            self.controller = UDEController()
            # UDE参数配置
            ude_params = {
                "quad_mass": params.get("quad_mass", 1.0),
                "hov_percent": params.get("hov_percent", 0.5),
                "Kp_xy": params.get("Kp_xy", 0.5),
                "Kp_z": params.get("Kp_z", 0.5),
                "Kd_xy": params.get("Kd_xy", 2.0),
                "Kd_z": params.get("Kd_z", 2.0),
                "T_ude": params.get("T_ude", 1.0),
                "tilt_angle_max": params.get("tilt_angle_max", 20.0),
                "pxy_int_max": params.get("pxy_int_max", 1.0),
                "pz_int_max": params.get("pz_int_max", 1.0)
            }
            self.controller.init(ude_params)
            
        elif self.controller_type == ControllerType.ADRC:
            self.controller = ADRCController()
            # ADRC参数配置 - 完全对应launch文件参数
            adrc_params = {
                "quad_mass": params.get("quad_mass", 2.5),
                "hov_percent": params.get("hov_percent", 0.5),
                "k": params.get("k", 0.8),
                "k1": params.get("k1", -0.15),
                "k2": params.get("k2", -3.0),
                "c1": params.get("c1", 1.5),
                "c2": params.get("c2", 0.6),
                "lambda_D": params.get("lambda_D", 1.0),
                "beta_max": params.get("beta_max", 1.0),
                "gamma": params.get("gamma", 0.2),
                "lambda": params.get("lambda", 0.8),
                "sigma": params.get("sigma", 0.9),
                "omega_star": params.get("omega_star", 0.02),
                "t1": params.get("t1", 0.02),
                "t2": params.get("t2", 0.04),
                "l": params.get("l", 5.0),
                "kp": params.get("kp", 2.0),
                "ki": params.get("ki", 0.3),
                "kd": params.get("kd", 2.0),
                "pxy_int_max": params.get("pxy_int_max", 0.5),
                "pz_int_max": params.get("pz_int_max", 0.5)
            }
            self.controller.init(adrc_params)
        
        self.logger.info(f"{self.controller_type.name} controller initialized")
    
    def rmtt_to_current_state(self, tello_state: Dict[str, Any]) -> CurrentState:
        """将RMTT传感器数据转换为控制器需要的当前状态"""
        
        current_time = time.time()
        
        # 获取传感器数据
        height_cm = tello_state.get('height_cm', 0)
        tof_distance_cm = tello_state.get('tof_distance_cm', 0)
        barometer_cm = tello_state.get('barometer_cm', 0)
        roll_deg = tello_state.get('roll_deg', 0)
        pitch_deg = tello_state.get('pitch_deg', 0) 
        yaw_deg = tello_state.get('yaw_deg', 0)
        vgx_cm_s = tello_state.get('vgx_cm_s', 0)
        vgy_cm_s = tello_state.get('vgy_cm_s', 0)
        vgz_cm_s = tello_state.get('vgz_cm_s', 0)
        
        # RMTT双重定高传感器融合 - 优先使用TOF传感器
        if tof_distance_cm > 0 and tof_distance_cm < 300:  # TOF有效范围
            z_position = tof_distance_cm / 100.0  # 转换为米
        elif height_cm > 0:
            z_position = height_cm / 100.0  # 转换为米
        else:
            z_position = barometer_cm / 100.0  # 备用气压计
        
        # 初始化处理 - 设置起飞位置为原点
        if not self.initialized:
            self.takeoff_position = np.array([0.0, 0.0, z_position])
            self.position_estimate = np.array([0.0, 0.0, z_position])
            self.initialized = True
            self.logger.info(f"Takeoff position set: {self.takeoff_position}")
        
        # 位置估计 - 简化版本（实际应用中需要更精确的状态估计）
        # 在实际实验中，X,Y坐标需要通过视觉或其他传感器获得
        # 这里假设起飞点为原点，Z轴直接使用高度传感器
        self.position_estimate[2] = z_position - self.takeoff_position[2]
        
        # 速度估计 - 使用RMTT提供的速度数据
        self.velocity_estimate = np.array([
            vgx_cm_s / 100.0,  # 转换为m/s
            vgy_cm_s / 100.0,
            vgz_cm_s / 100.0
        ])
        
        # 加速度估计 - 简化处理
        acceleration_estimate = np.zeros(3)
        
        # 姿态角转换
        yaw_rad = np.deg2rad(yaw_deg)
        
        # 四元数计算（可选）
        quaternion = np.array([
            np.cos(yaw_rad/2), 0, 0, np.sin(yaw_rad/2)
        ])
        
        return CurrentState(
            pos=self.position_estimate.copy(),
            vel=self.velocity_estimate.copy(),
            acc=acceleration_estimate,
            yaw=yaw_rad,
            q=quaternion
        )
    
    def create_desired_state(self, target_pos: np.ndarray, target_vel: np.ndarray = None, 
                           target_acc: np.ndarray = None, target_yaw: float = 0.0) -> DesiredState:
        """创建期望状态"""
        
        if target_vel is None:
            target_vel = np.zeros(3)
        if target_acc is None:
            target_acc = np.zeros(3)
        
        return DesiredState(
            pos=target_pos,
            vel=target_vel,
            acc=target_acc,
            yaw=target_yaw
        )
    
    def update_control(self, current_state: CurrentState, desired_state: DesiredState, dt: float) -> ControlOutput:
        """更新控制器并返回控制输出"""
        
        if self.controller is None:
            raise RuntimeError("Controller not initialized. Call init_controller first.")
        
        # 设置控制器状态
        self.controller.set_current_state(current_state)
        self.controller.set_desired_state(desired_state)
        
        # 计算控制输出
        control_output = self.controller.update(dt)
        
        return control_output
    
    def control_output_to_tello_rc(self, control_output: ControlOutput) -> tuple:
        """将控制输出转换为Tello RC指令"""
        return control_output.to_tello_rc()
    
    def get_controller_status(self) -> Dict[str, Any]:
        """获取控制器状态信息"""
        return {
            'controller_type': self.controller_type.name,
            'initialized': self.initialized,
            'position_estimate': self.position_estimate.tolist(),
            'velocity_estimate': self.velocity_estimate.tolist(),
            'takeoff_position': self.takeoff_position.tolist() if self.takeoff_position is not None else None
        }
    
    def print_debug_info(self):
        """打印调试信息"""
        if self.controller is not None:
            self.controller.printf_result()
        
        self.logger.info(f"Position estimate: [{self.position_estimate[0]:.3f}, {self.position_estimate[1]:.3f}, {self.position_estimate[2]:.3f}]")
        self.logger.info(f"Velocity estimate: [{self.velocity_estimate[0]:.3f}, {self.velocity_estimate[1]:.3f}, {self.velocity_estimate[2]:.3f}]")