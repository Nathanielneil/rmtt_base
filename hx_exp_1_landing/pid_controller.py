"""
PID控制器 - 严格对应原始C++实现

完全保持.h头文件中定义的所有成员变量和.cpp中的所有控制逻辑
"""

import numpy as np
import time
import logging
from typing import Dict, Any
from scipy.spatial.transform import Rotation as R

from .data_structures import DesiredState, CurrentState, ControlOutput, sign, sat


class PID_Controller:
    """PID控制器类 - 严格对应C++的PID_Controller类"""
    
    def __init__(self):
        """构造函数 - 完全对应C++构造函数"""
        # 初始化所有成员变量为零 - 严格对应.h文件定义
        self.pos_error_integral_ = np.zeros(3)  # Eigen::Vector3d pos_error_integral_
        self.vel_error_integral_ = np.zeros(3)  # Eigen::Vector3d vel_error_integral_
        self.pos_error_last_ = np.zeros(3)      # Eigen::Vector3d pos_error_last_
        self.vel_error_last_ = np.zeros(3)      # Eigen::Vector3d vel_error_last_
        self.u_att_ = np.zeros(4)               # Eigen::Vector4d u_att_
        
        # PID参数 - 严格对应.h文件定义
        self.kp_pos = np.zeros(3)    # Eigen::Vector3d kp_pos; 位置比例增益
        self.ki_pos = np.zeros(3)    # Eigen::Vector3d ki_pos; 位置积分增益  
        self.kd_pos = np.zeros(3)    # Eigen::Vector3d kd_pos; 位置微分增益
        
        # 基本参数 - 严格对应.h文件定义
        self.quad_mass = 0.0         # double quad_mass; 无人机质量
        self.hov_percent = 0.0       # double hov_percent; 悬停油门百分比
        self.max_tilt_angle = 0.0    # double max_tilt_angle; 最大倾斜角度
        self.max_thrust = 0.0        # double max_thrust; 最大推力
        self.min_thrust = 0.0        # double min_thrust; 最小推力
        self.int_max_xy = 0.0        # double int_max_xy; XY积分限制
        self.int_max_z = 0.0         # double int_max_z; Z积分限制
        
        # 内部状态变量 - 严格对应.h文件定义
        self.desired_state_ = None   # Desired_State desired_state_
        self.current_state_ = None   # Current_State current_state_
        
        # 日志
        self.logger = logging.getLogger("PID_Controller")
    
    def init(self, params: Dict[str, Any]):
        """初始化函数 - 严格对应C++的init(ros::NodeHandle &nh)函数"""
        
        # PID控制器参数 - 完全对应C++参数读取
        self.quad_mass = params.get("pid_gain/quad_mass", 1.0)
        self.hov_percent = params.get("pid_gain/hov_percent", 0.5)
        
        # 位置控制参数 - 严格对应C++逻辑
        kp_xy = params.get("pid_gain/Kp_xy", 2.0)
        self.kp_pos[0] = kp_xy
        self.kp_pos[1] = kp_xy  # kp_pos(1) = kp_pos(0);
        self.kp_pos[2] = params.get("pid_gain/Kp_z", 2.0)
        
        # 速度控制参数 - 严格对应C++逻辑  
        kv_xy = params.get("pid_gain/Kv_xy", 2.0)
        self.kd_pos[0] = kv_xy
        self.kd_pos[1] = kv_xy  # kd_pos(1) = kd_pos(0);
        self.kd_pos[2] = params.get("pid_gain/Kv_z", 2.0)
        
        # 积分控制参数 - 严格对应C++逻辑
        kvi_xy = params.get("pid_gain/Kvi_xy", 0.3)
        self.ki_pos[0] = kvi_xy
        self.ki_pos[1] = kvi_xy  # ki_pos(1) = ki_pos(0);
        self.ki_pos[2] = params.get("pid_gain/Kvi_z", 0.3)
        
        # 控制量限幅 - 严格对应C++参数
        self.max_tilt_angle = params.get("pid_gain/tilt_angle_max", 10.0)
        self.int_max_xy = params.get("pid_gain/pxy_int_max", 0.5)
        self.int_max_z = params.get("pid_gain/pz_int_max", 0.5)
        
        # 固定值 - 对应C++固定赋值
        self.max_thrust = 1.0
        self.min_thrust = 0.1
        
        # 对应C++的ROS_INFO
        self.logger.info("PID Controller initialized")
    
    def set_desired_state(self, desired_state: DesiredState):
        """设置期望状态 - 严格对应C++函数"""
        self.desired_state_ = desired_state
    
    def set_current_state(self, current_state: CurrentState):
        """设置当前状态 - 严格对应C++函数"""
        self.current_state_ = current_state
    
    def update(self, dt: float) -> np.ndarray:
        """更新函数 - 严格对应C++的Eigen::Vector4d update(double dt)"""
        
        # 位置误差和速度误差 - 严格对应C++代码第75-77行
        pos_error = self.desired_state_.pos - self.current_state_.pos
        vel_error = self.desired_state_.vel - self.current_state_.vel
        
        # 限制最大误差 - 严格对应C++代码第79-84行
        for i in range(3):
            pos_error[i] = sat(pos_error[i], 3.0)
            vel_error[i] = sat(vel_error[i], 3.0)
        
        # 积分项计算（仅在小误差时启动积分）- 严格对应C++代码第86-109行
        for i in range(2):  # XY轴 - for(int i = 0; i < 2; i++)
            if abs(pos_error[i]) < 0.2:  # if(std::abs(pos_error[i]) < 0.2)
                self.pos_error_integral_[i] += pos_error[i] * dt
                self.pos_error_integral_[i] = sat(self.pos_error_integral_[i], self.int_max_xy)
            else:
                self.pos_error_integral_[i] = 0
        
        # Z轴积分 - 严格对应C++代码第101-109行
        if abs(pos_error[2]) < 0.5:  # if(std::abs(pos_error[2]) < 0.5)
            self.pos_error_integral_[2] += pos_error[2] * dt
            self.pos_error_integral_[2] = sat(self.pos_error_integral_[2], self.int_max_z)
        else:
            self.pos_error_integral_[2] = 0
        
        # PID控制律 - 严格对应C++代码第111-115行（使用cwiseProduct对应的逐元素乘法）
        des_acc = (self.desired_state_.acc + 
                   self.kp_pos * pos_error +      # kp_pos.cwiseProduct(pos_error)
                   self.kd_pos * vel_error +      # kd_pos.cwiseProduct(vel_error)
                   self.ki_pos * self.pos_error_integral_)  # ki_pos.cwiseProduct(pos_error_integral_)
        
        # 期望力 = 质量*控制量 + 重力抵消 - 严格对应C++代码第117-118行
        F_des = des_acc * self.quad_mass + np.array([0, 0, self.quad_mass * 9.8])
        
        # 关键安全检查：防止除零和异常推力 - 严格对应C++代码第120-127行
        if abs(F_des[2]) < 0.01:  # 防止除零
            self.logger.error("PID: Critical thrust too small! Emergency fallback.")
            F_des[2] = 0.5 * self.quad_mass * 9.8  # 紧急回落到悬停推力
            F_des[0] = 0
            F_des[1] = 0
        
        # 推力限制 - 严格对应C++代码第129-137行
        if F_des[2] < 0.5 * self.quad_mass * 9.8:
            F_des = F_des / F_des[2] * (0.5 * self.quad_mass * 9.8)
        elif F_des[2] > 2.0 * self.quad_mass * 9.8:
            F_des = F_des / F_des[2] * (2.0 * self.quad_mass * 9.8)
        
        # 倾斜角限制 - 严格对应C++代码第139-151行
        max_tilt_rad = self.max_tilt_angle * np.pi / 180.0
        if abs(F_des[2]) > 0.01:  # 确保分母不为零
            if abs(F_des[0]/F_des[2]) > np.tan(max_tilt_rad):
                F_des[0] = sign(F_des[0]) * F_des[2] * np.tan(max_tilt_rad)
            if abs(F_des[1]/F_des[2]) > np.tan(max_tilt_rad):
                F_des[1] = sign(F_des[1]) * F_des[2] * np.tan(max_tilt_rad)
        
        # 转换到机体坐标系计算姿态角 - 严格对应C++代码第153-160行
        cos_yaw = np.cos(self.current_state_.yaw)
        sin_yaw = np.sin(self.current_state_.yaw)
        
        F_body = np.array([
             cos_yaw * F_des[0] + sin_yaw * F_des[1],   # F_body(0) =  cos_yaw * F_des(0) + sin_yaw * F_des(1);
            -sin_yaw * F_des[0] + cos_yaw * F_des[1],   # F_body(1) = -sin_yaw * F_des(0) + cos_yaw * F_des(1);
            F_des[2]                                     # F_body(2) = F_des(2);
        ])
        
        # 计算期望姿态角 - 严格对应C++代码第162-165行
        self.u_att_[0] = np.arctan2(-F_body[1], F_body[2])  # roll
        self.u_att_[1] = np.arctan2(F_body[0], F_body[2])   # pitch
        self.u_att_[2] = self.desired_state_.yaw             # yaw
        
        # 计算油门 - 严格对应C++代码第167-170行
        R_curr = self.current_state_.q.as_matrix()  # toRotationMatrix()
        z_b_curr = R_curr[:, 2]                     # R_curr.col(2)
        thrust_raw = np.dot(F_des, z_b_curr)        # F_des.dot(z_b_curr)
        
        # 防止hov_percent为零的安全检查 - 严格对应C++代码第172-175行
        safe_hov_percent = self.hov_percent if self.hov_percent > 0.01 else 0.5
        full_thrust = self.quad_mass * 9.8 / safe_hov_percent
        self.u_att_[3] = thrust_raw / full_thrust
        
        # 油门限制 - 严格对应C++代码第177-179行
        self.u_att_[3] = sat(self.u_att_[3], self.max_thrust)
        if self.u_att_[3] < self.min_thrust:
            self.u_att_[3] = self.min_thrust
        
        return self.u_att_  # 返回Eigen::Vector4d对应的numpy数组
    
    def printf_result(self):
        """打印控制结果 - 严格对应C++的printf_result函数第184-187行"""
        self.logger.info("PID Controller - Roll: %.2f deg, Pitch: %.2f deg, Yaw: %.2f deg, Thrust: %.3f", 
                        self.u_att_[0]*180/np.pi, self.u_att_[1]*180/np.pi, 
                        self.u_att_[2]*180/np.pi, self.u_att_[3])