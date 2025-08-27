"""
UDE (Uncertainty and Disturbance Estimator) 控制器 - 完全对应原始C++实现

保持所有原始参数和控制逻辑不变
"""

import numpy as np
import time
import logging
from typing import Dict, Any

from .landing_state import DesiredState, CurrentState, ControlOutput, sign, sat, quaternion_to_rotation_matrix


class UDEController:
    """UDE控制器类 - 完全对应C++的UDE_Controller"""
    
    def __init__(self):
        # 初始化状态变量 - 对应C++构造函数
        self.disturbance_estimate = np.zeros(3)
        self.disturbance_filter_state = np.zeros(3)
        self.u_att = np.zeros(4)  # [roll, pitch, yaw, thrust]
        
        # UDE控制参数
        self.kp_pos = np.zeros(3)
        self.kd_pos = np.zeros(3)
        self.filter_param = 1.0  # T_ude滤波器参数
        
        # 基本参数
        self.quad_mass = 0.087  # RMTT实际质量87g
        self.hov_percent = 0.5
        self.max_tilt_angle = 20.0
        self.max_thrust = 1.0
        self.min_thrust = 0.1
        self.int_max_xy = 1.0
        self.int_max_z = 1.0
        
        # 内部状态
        self.desired_state = None
        self.current_state = None
        
        self.logger = logging.getLogger("UDEController")
    
    def init(self, params: Dict[str, Any]):
        """初始化控制器参数 - 完全对应C++的init函数"""
        
        # UDE控制器参数 - 完全对应原始参数
        self.quad_mass = params.get("quad_mass", 0.087)  # RMTT实际质量
        self.hov_percent = params.get("hov_percent", 0.5)
        
        # 位置控制参数
        kp_xy = params.get("Kp_xy", 0.5)
        self.kp_pos[0] = kp_xy
        self.kp_pos[1] = kp_xy
        self.kp_pos[2] = params.get("Kp_z", 0.5)
        
        # 微分控制参数
        kd_xy = params.get("Kd_xy", 2.0)
        self.kd_pos[0] = kd_xy
        self.kd_pos[1] = kd_xy
        self.kd_pos[2] = params.get("Kd_z", 2.0)
        
        # UDE特有参数
        self.filter_param = params.get("T_ude", 1.0)
        self.max_tilt_angle = params.get("tilt_angle_max", 20.0)
        
        self.int_max_xy = params.get("pxy_int_max", 1.0)
        self.int_max_z = params.get("pz_int_max", 1.0)
        
        self.max_thrust = 1.0
        self.min_thrust = 0.1
        
        self.logger.info("UDE Controller initialized")
    
    def set_desired_state(self, desired_state: DesiredState):
        """设置期望状态"""
        self.desired_state = desired_state
    
    def set_current_state(self, current_state: CurrentState):
        """设置当前状态"""
        self.current_state = current_state
    
    def update(self, dt: float) -> ControlOutput:
        """控制器更新函数 - 完全对应C++的update函数"""
        
        # 位置误差和速度误差
        pos_error = self.desired_state.pos - self.current_state.pos
        vel_error = self.desired_state.vel - self.current_state.vel
        
        # 限制最大误差 - 对应C++代码
        for i in range(3):
            pos_error[i] = sat(pos_error[i], 3.0)
            vel_error[i] = sat(vel_error[i], 3.0)
        
        # UDE算法：标称控制律 - 完全对应C++公式
        u_l = self.desired_state.acc + self.kp_pos * pos_error + self.kd_pos * vel_error
        
        # UDE算法：扰动估计 - 对应C++的积分项和扰动估计
        integral_term = np.zeros(3)
        for i in range(3):
            if abs(pos_error[i]) < 0.5:
                integral_term[i] += pos_error[i] * dt
        
        # UDE扰动补偿项 - 完全对应C++公式
        u_d = -(1.0 / self.filter_param) * (self.kp_pos * integral_term + 
                                            self.kd_pos * pos_error + vel_error)
        
        # 扰动补偿限制 - 对应C++限制逻辑
        for i in range(3):
            max_limit = self.int_max_xy if i < 2 else self.int_max_z
            u_d[i] = sat(u_d[i], max_limit)
        
        # UDE控制律 - 完全对应C++公式
        u_total = u_l - u_d
        
        # 转换为期望力 - 对应C++计算
        F_des = u_total * self.quad_mass + np.array([0, 0, self.quad_mass * 9.8])
        
        # 关键安全检查：防止除零和异常推力 - 完全对应C++安全检查
        if abs(F_des[2]) < 0.01:  # 防止除零
            self.logger.error("UDE: Critical thrust too small! Emergency fallback.")
            F_des[2] = 0.5 * self.quad_mass * 9.8  # 紧急回落到悬停推力
            F_des[0] = 0
            F_des[1] = 0
        
        # 推力和角度限制 - 对应C++逻辑
        if F_des[2] < 0.5 * self.quad_mass * 9.8:
            F_des = F_des / F_des[2] * (0.5 * self.quad_mass * 9.8)
        elif F_des[2] > 2.0 * self.quad_mass * 9.8:
            F_des = F_des / F_des[2] * (2.0 * self.quad_mass * 9.8)
        
        # 倾斜角限制 - 完全对应C++的安全检查
        max_tilt_rad = self.max_tilt_angle * np.pi / 180.0
        if abs(F_des[2]) > 0.01:  # 确保分母不为零
            if abs(F_des[0]/F_des[2]) > np.tan(max_tilt_rad):
                F_des[0] = sign(F_des[0]) * F_des[2] * np.tan(max_tilt_rad)
            if abs(F_des[1]/F_des[2]) > np.tan(max_tilt_rad):
                F_des[1] = sign(F_des[1]) * F_des[2] * np.tan(max_tilt_rad)
        
        # 计算期望姿态 - 对应C++坐标变换
        cos_yaw = np.cos(self.current_state.yaw)
        sin_yaw = np.sin(self.current_state.yaw)
        
        F_body = np.array([
             cos_yaw * F_des[0] + sin_yaw * F_des[1],
            -sin_yaw * F_des[0] + cos_yaw * F_des[1],
            F_des[2]
        ])
        
        # 计算期望姿态角 - 完全对应C++计算
        self.u_att[0] = np.arctan2(-F_body[1], F_body[2])  # roll
        self.u_att[1] = np.arctan2(F_body[0], F_body[2])   # pitch
        self.u_att[2] = self.desired_state.yaw              # yaw
        
        # 计算油门 - 对应C++推力计算
        if self.current_state.q is not None:
            R_curr = quaternion_to_rotation_matrix(self.current_state.q)
            z_b_curr = R_curr[:, 2]
            thrust_raw = np.dot(F_des, z_b_curr)
        else:
            # 简化版本：假设当前姿态为水平
            thrust_raw = F_des[2]
        
        # 防止hov_percent为零的安全检查 - 对应C++安全检查
        safe_hov_percent = self.hov_percent if self.hov_percent > 0.01 else 0.5
        full_thrust = self.quad_mass * 9.8 / safe_hov_percent
        self.u_att[3] = thrust_raw / full_thrust
        
        # 油门限制 - 对应C++限制
        self.u_att[3] = sat(self.u_att[3], self.max_thrust)
        if self.u_att[3] < self.min_thrust:
            self.u_att[3] = self.min_thrust
        
        return ControlOutput(
            timestamp=time.time(),
            roll=self.u_att[0],
            pitch=self.u_att[1],
            yaw=self.u_att[2],
            thrust=self.u_att[3]
        )
    
    def printf_result(self):
        """打印控制结果 - 对应C++的printf_result"""
        self.logger.info("UDE Controller - Roll: %.2f deg, Pitch: %.2f deg, Yaw: %.2f deg, Thrust: %.3f", 
                        self.u_att[0]*180/np.pi, self.u_att[1]*180/np.pi, 
                        self.u_att[2]*180/np.pi, self.u_att[3])