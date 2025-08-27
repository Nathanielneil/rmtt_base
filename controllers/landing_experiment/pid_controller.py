"""
PID控制器 - 完全对应原始C++实现

保持所有原始参数和控制逻辑不变
"""

import numpy as np
import time
import logging
from typing import Dict, Any

from .landing_state import DesiredState, CurrentState, ControlOutput, sign, sat, quaternion_to_rotation_matrix


class PIDController:
    """PID控制器类 - 完全对应C++的PID_Controller"""
    
    def __init__(self):
        # 初始化状态变量 - 对应C++构造函数
        self.pos_error_integral = np.zeros(3)
        self.vel_error_integral = np.zeros(3)
        self.pos_error_last = np.zeros(3)
        self.vel_error_last = np.zeros(3)
        self.u_att = np.zeros(4)  # [roll, pitch, yaw, thrust]
        
        # 参数初始化
        self.kp_pos = np.zeros(3)    # 位置比例增益
        self.ki_pos = np.zeros(3)    # 位置积分增益
        self.kd_pos = np.zeros(3)    # 位置微分增益
        
        # 基本参数
        self.quad_mass = 1.0         # 无人机质量
        self.hov_percent = 0.5       # 悬停油门百分比
        self.max_tilt_angle = 10.0   # 最大倾斜角度
        self.max_thrust = 1.0        # 最大推力
        self.min_thrust = 0.1        # 最小推力
        self.int_max_xy = 0.5        # XY积分限制
        self.int_max_z = 0.5         # Z积分限制
        
        # 内部状态
        self.desired_state = None
        self.current_state = None
        
        self.logger = logging.getLogger("PIDController")
    
    def init(self, params: Dict[str, Any]):
        """初始化控制器参数 - 对应C++的init函数"""
        
        # PID控制器参数 - 完全对应原始参数
        self.quad_mass = params.get("quad_mass", 1.0)
        self.hov_percent = params.get("hov_percent", 0.5)
        
        # 位置控制参数
        kp_xy = params.get("Kp_xy", 2.0)
        self.kp_pos[0] = kp_xy
        self.kp_pos[1] = kp_xy
        self.kp_pos[2] = params.get("Kp_z", 2.0)
        
        # 速度控制参数
        kv_xy = params.get("Kv_xy", 2.0)
        self.kd_pos[0] = kv_xy
        self.kd_pos[1] = kv_xy
        self.kd_pos[2] = params.get("Kv_z", 2.0)
        
        # 积分控制参数
        kvi_xy = params.get("Kvi_xy", 0.3)
        self.ki_pos[0] = kvi_xy
        self.ki_pos[1] = kvi_xy
        self.ki_pos[2] = params.get("Kvi_z", 0.3)
        
        # 控制量限幅
        self.max_tilt_angle = params.get("tilt_angle_max", 10.0)
        self.int_max_xy = params.get("pxy_int_max", 0.5)
        self.int_max_z = params.get("pz_int_max", 0.5)
        
        self.max_thrust = 1.0
        self.min_thrust = 0.1
        
        self.logger.info("PID Controller initialized")
    
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
        
        # 积分项计算（仅在小误差时启动积分）- 对应C++逻辑
        for i in range(2):  # XY轴
            if abs(pos_error[i]) < 0.2:
                self.pos_error_integral[i] += pos_error[i] * dt
                self.pos_error_integral[i] = sat(self.pos_error_integral[i], self.int_max_xy)
            else:
                self.pos_error_integral[i] = 0
        
        # Z轴积分
        if abs(pos_error[2]) < 0.5:
            self.pos_error_integral[2] += pos_error[2] * dt
            self.pos_error_integral[2] = sat(self.pos_error_integral[2], self.int_max_z)
        else:
            self.pos_error_integral[2] = 0
        
        # PID控制律 - 完全对应C++公式
        des_acc = (self.desired_state.acc + 
                   self.kp_pos * pos_error + 
                   self.kd_pos * vel_error + 
                   self.ki_pos * self.pos_error_integral)
        
        # 期望力 = 质量*控制量 + 重力抵消 - 对应C++计算
        F_des = des_acc * self.quad_mass + np.array([0, 0, self.quad_mass * 9.8])
        
        # 关键安全检查：防止除零和异常推力 - 完全对应C++安全检查
        if abs(F_des[2]) < 0.01:  # 防止除零
            self.logger.error("PID: Critical thrust too small! Emergency fallback.")
            F_des[2] = 0.5 * self.quad_mass * 9.8  # 紧急回落到悬停推力
            F_des[0] = 0
            F_des[1] = 0
        
        # 推力限制 - 对应C++逻辑
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
        
        # 转换到机体坐标系计算姿态角 - 对应C++坐标变换
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
        self.logger.info("PID Controller - Roll: %.2f deg, Pitch: %.2f deg, Yaw: %.2f deg, Thrust: %.3f", 
                        self.u_att[0]*180/np.pi, self.u_att[1]*180/np.pi, 
                        self.u_att[2]*180/np.pi, self.u_att[3])