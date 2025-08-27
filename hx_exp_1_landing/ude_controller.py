"""
UDE控制器 - 严格对应原始C++实现

完全保持.h头文件中定义的所有成员变量和.cpp中的所有控制逻辑
"""

import numpy as np
import time
import logging
from typing import Dict, Any
from data_structures import DesiredState, CurrentState, ControlOutput, sign, sat, R


class UDE_Controller:
    """UDE控制器类 - 严格对应C++的UDE_Controller类"""
    
    def __init__(self):
        """构造函数 - 严格对应C++构造函数第190-196行"""
        # 初始化所有成员变量 - 严格对应.h文件定义和C++构造函数
        self.disturbance_estimate_ = np.zeros(3)      # Eigen::Vector3d disturbance_estimate_
        self.disturbance_filter_state_ = np.zeros(3) # Eigen::Vector3d disturbance_filter_state_
        self.u_att_ = np.zeros(4)                     # Eigen::Vector4d u_att_
        
        # UDE控制参数 - 严格对应.h文件定义
        self.kp_pos = np.zeros(3)     # Eigen::Vector3d kp_pos
        self.kd_pos = np.zeros(3)     # Eigen::Vector3d kd_pos
        self.filter_param = 0.0       # double filter_param; T_ude滤波器参数
        
        # 基本参数 - 严格对应.h文件定义
        self.quad_mass = 0.0          # double quad_mass
        self.hov_percent = 0.0        # double hov_percent
        self.max_tilt_angle = 0.0     # double max_tilt_angle
        self.max_thrust = 0.0         # double max_thrust
        self.min_thrust = 0.0         # double min_thrust
        self.int_max_xy = 0.0         # double int_max_xy
        self.int_max_z = 0.0          # double int_max_z
        
        # 内部状态变量 - 严格对应.h文件定义
        self.desired_state_ = None    # Desired_State desired_state_
        self.current_state_ = None    # Current_State current_state_
        
        self.logger = logging.getLogger("UDE_Controller")
    
    def init(self, params: Dict[str, Any]):
        """初始化函数 - 严格对应C++的init函数第200-224行"""
        
        # UDE控制器参数 - 严格对应C++参数读取
        self.quad_mass = params.get("ude_gain/quad_mass", 0.087)  # RMTT实际质量
        self.hov_percent = params.get("ude_gain/hov_percent", 0.5)
        
        # 位置控制参数 - 严格对应C++逻辑第206-208行
        kp_xy = params.get("ude_gain/Kp_xy", 0.5)
        self.kp_pos[0] = kp_xy
        self.kp_pos[1] = kp_xy  # kp_pos(1) = kp_pos(0);
        self.kp_pos[2] = params.get("ude_gain/Kp_z", 0.5)
        
        # 微分控制参数 - 严格对应C++逻辑第210-212行
        kd_xy = params.get("ude_gain/Kd_xy", 2.0)
        self.kd_pos[0] = kd_xy
        self.kd_pos[1] = kd_xy  # kd_pos(1) = kd_pos(0);
        self.kd_pos[2] = params.get("ude_gain/Kd_z", 2.0)
        
        # UDE特有参数 - 严格对应C++第214-215行
        self.filter_param = params.get("ude_gain/T_ude", 1.0)
        self.max_tilt_angle = params.get("ude_gain/tilt_angle_max", 20.0)
        
        # 积分限制参数 - 严格对应C++第217-218行
        self.int_max_xy = params.get("ude_gain/pxy_int_max", 1.0)
        self.int_max_z = params.get("ude_gain/pz_int_max", 1.0)
        
        # 固定值 - 对应C++第220-221行
        self.max_thrust = 1.0
        self.min_thrust = 0.1
        
        # 对应C++的ROS_INFO第223行
        self.logger.info("UDE Controller initialized")
    
    def set_desired_state(self, desired_state: DesiredState):
        """设置期望状态 - 严格对应C++函数第226-229行"""
        self.desired_state_ = desired_state
    
    def set_current_state(self, current_state: CurrentState):
        """设置当前状态 - 严格对应C++函数第231-234行"""
        self.current_state_ = current_state
    
    def update(self, dt: float) -> np.ndarray:
        """更新函数 - 严格对应C++的update函数第236-338行"""
        
        # 位置误差和速度误差 - 严格对应C++第238-240行
        pos_error = self.desired_state_.pos - self.current_state_.pos
        vel_error = self.desired_state_.vel - self.current_state_.vel
        
        # 限制最大误差 - 严格对应C++第242-247行
        for i in range(3):
            pos_error[i] = sat(pos_error[i], 3.0)
            vel_error[i] = sat(vel_error[i], 3.0)
        
        # UDE算法：标称控制律 - 严格对应C++第249-250行
        u_l = self.desired_state_.acc + self.kp_pos * pos_error + self.kd_pos * vel_error
        
        # UDE算法：扰动估计 - 严格对应C++第252-260行
        integral_term = np.zeros(3)
        for i in range(3):
            if abs(pos_error[i]) < 0.5:
                integral_term[i] += pos_error[i] * dt
        
        # UDE扰动补偿项 - 严格对应C++第262-263行
        u_d = -(1.0 / self.filter_param) * (self.kp_pos * integral_term + 
                                            self.kd_pos * pos_error + vel_error)
        
        # 扰动补偿限制 - 严格对应C++第265-270行
        for i in range(3):
            max_limit = self.int_max_xy if i < 2 else self.int_max_z
            u_d[i] = sat(u_d[i], max_limit)
        
        # UDE控制律 - 严格对应C++第272-273行
        u_total = u_l - u_d
        
        # 转换为期望力 - 严格对应C++第275-276行
        F_des = u_total * self.quad_mass + np.array([0, 0, self.quad_mass * 9.8])
        
        # 关键安全检查：防止除零和异常推力 - 严格对应C++第278-285行
        if abs(F_des[2]) < 0.01:  # 防止除零
            self.logger.error("UDE: Critical thrust too small! Emergency fallback.")
            F_des[2] = 0.5 * self.quad_mass * 9.8  # 紧急回落到悬停推力
            F_des[0] = 0
            F_des[1] = 0
        
        # 推力和角度限制 - 严格对应C++第287-295行
        if F_des[2] < 0.5 * self.quad_mass * 9.8:
            F_des = F_des / F_des[2] * (0.5 * self.quad_mass * 9.8)
        elif F_des[2] > 2.0 * self.quad_mass * 9.8:
            F_des = F_des / F_des[2] * (2.0 * self.quad_mass * 9.8)
        
        # 倾斜角限制 - 严格对应C++第297-309行
        max_tilt_rad = self.max_tilt_angle * np.pi / 180.0
        if abs(F_des[2]) > 0.01:  # 确保分母不为零
            if abs(F_des[0]/F_des[2]) > np.tan(max_tilt_rad):
                F_des[0] = sign(F_des[0]) * F_des[2] * np.tan(max_tilt_rad)
            if abs(F_des[1]/F_des[2]) > np.tan(max_tilt_rad):
                F_des[1] = sign(F_des[1]) * F_des[2] * np.tan(max_tilt_rad)
        
        # 计算期望姿态 - 严格对应C++第311-318行
        cos_yaw = np.cos(self.current_state_.yaw)
        sin_yaw = np.sin(self.current_state_.yaw)
        
        F_body = np.array([
             cos_yaw * F_des[0] + sin_yaw * F_des[1],
            -sin_yaw * F_des[0] + cos_yaw * F_des[1],
            F_des[2]
        ])
        
        # 计算期望姿态角 - 严格对应C++第320-322行
        self.u_att_[0] = np.arctan2(-F_body[1], F_body[2])  # roll
        self.u_att_[1] = np.arctan2(F_body[0], F_body[2])   # pitch
        self.u_att_[2] = self.desired_state_.yaw             # yaw
        
        # 计算油门 - 严格对应C++第324-327行
        R_curr = self.current_state_.q.as_matrix()  # toRotationMatrix()
        z_b_curr = R_curr[:, 2]                     # R_curr.col(2)
        thrust_raw = np.dot(F_des, z_b_curr)        # F_des.dot(z_b_curr)
        
        # 防止hov_percent为零的安全检查 - 严格对应C++第329-332行
        safe_hov_percent = self.hov_percent if self.hov_percent > 0.01 else 0.5
        full_thrust = self.quad_mass * 9.8 / safe_hov_percent
        self.u_att_[3] = thrust_raw / full_thrust
        
        # 油门限制 - 严格对应C++第334-336行
        self.u_att_[3] = sat(self.u_att_[3], self.max_thrust)
        if self.u_att_[3] < self.min_thrust:
            self.u_att_[3] = self.min_thrust
        
        return self.u_att_  # 返回Eigen::Vector4d对应的numpy数组
    
    def printf_result(self):
        """打印控制结果 - 严格对应C++的printf_result函数第340-343行"""
        self.logger.info("UDE Controller - Roll: %.2f deg, Pitch: %.2f deg, Yaw: %.2f deg, Thrust: %.3f", 
                        self.u_att_[0]*180/np.pi, self.u_att_[1]*180/np.pi, 
                        self.u_att_[2]*180/np.pi, self.u_att_[3])