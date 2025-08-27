"""
ADRC (Active Disturbance Rejection Control) 控制器 - 完全对应原始C++实现
基于AMESO (Adaptive Model-based Extended State Observer) 论文实现

保持所有原始参数和控制逻辑不变
"""

import numpy as np
import time
import logging
from typing import Dict, Any, List

from .landing_state import DesiredState, CurrentState, ControlOutput, sign, sat


class ADRCController:
    """ADRC控制器类 - 完全对应C++的ADRC_Controller"""
    
    def __init__(self):
        # 初始化所有状态变量为零 - 对应C++构造函数
        self.u_att = np.zeros(4)  # 控制输出
        
        # AMESO观测器状态 - 修改为1维
        self.epsilon_hat1 = 0.0    # AMESO观测器状态1
        self.epsilon_hat2 = 0.0    # AMESO观测器状态2
        self.epsilon_hat3 = 0.0    # AMESO观测器状态3
        
        # 跟踪微分器状态 - 修改为1维
        self.epsilon_bar1 = 0.0    # 跟踪微分器状态1
        self.epsilon_bar2 = 0.0    # 跟踪微分器状态2
        
        # 可变指数项 - 修改为1维
        self.epsilon_beta1 = 0.0   # 可变指数项1
        self.epsilon_beta2 = 0.0   # 可变指数项2
        self.integral_term = 0.0   # 滑模面积分项
        
        # 标称系统状态 - 修改为1维
        self.epsilon_n1 = 0.0      # 标称系统位置误差
        self.epsilon_n2 = 0.0      # 标称系统速度误差
        self.epsilon_n1_init = 0.0 # 初始标称位置误差
        self.epsilon_n2_init = 0.0 # 初始标称速度误差
        
        # 选择油门方式
        self.method_choose = 1
        
        # 初始化自适应权重向量
        self.n_basis = 9  # 基函数数量
        self.omega_hat = [0.0] * self.n_basis
        
        # 初始化标志
        self.initialized = False
        
        # 基本参数 - 对应C++所有参数
        self.quad_mass = 2.5        # 无人机质量
        self.hov_percent = 0.5      # 悬停油门百分比
        self.max_tilt_angle = 10.0  # 最大倾斜角度
        self.max_thrust = 1.0       # 最大推力
        self.min_thrust = 0.1       # 最小推力
        self.int_max_xy = 0.5       # XY积分限制
        self.int_max_z = 0.5        # Z积分限制
        
        # 滑模控制参数
        self.k = 0.8                # 滑模增益
        self.k1 = -0.15             # 反馈增益参数
        self.k2 = -3.0              # 反馈增益参数
        self.c1 = 1.5               # 滑模面系数
        self.c2 = 0.6               # 滑模面系数
        self.lambda_D = 1.0         # 积分参数
        self.beta_max = 1.0         # 最大指数值
        self.gamma_param = 0.2      # 指数参数
        
        # 自适应模型参数
        self.lambda_adapt = 0.8     # 自适应学习率
        self.sigma_adapt = 0.9      # 收缩因子
        self.omega_star = 0.02      # 阈值参数
        
        # 跟踪微分器TD参数
        self.t1 = 0.02              # TD时间常数
        self.t2 = 0.04              # TD时间常数
        
        # AMESO观测器参数
        self.l_eso = 5.0            # ESO增益参数
        
        # PID参数
        self.k_p = 2.0              # 比例增益
        self.k_i = 0.3              # 积分增益
        self.k_d = 2.0              # 微分增益
        self.integral_term_pid = np.zeros(3)  # PID中的积分项
        
        # 内部状态变量
        self.desired_state = None
        self.current_state = None
        
        self.logger = logging.getLogger("ADRCController")
    
    def init(self, params: Dict[str, Any]):
        """初始化控制器参数 - 完全对应C++的init函数"""
        
        # 基本参数从参数字典读取 - 完全对应原始参数
        self.quad_mass = params.get("quad_mass", 2.5)
        self.hov_percent = params.get("hov_percent", 0.5)
        
        # 滑模控制参数，来自论文表1
        self.k = params.get("k", 0.8)
        self.k1 = params.get("k1", -0.15)
        self.k2 = params.get("k2", -3.0)
        self.c1 = params.get("c1", 1.5)
        self.c2 = params.get("c2", 0.6)
        self.lambda_D = params.get("lambda_D", 1.0)
        self.beta_max = params.get("beta_max", 1.0)
        self.gamma_param = params.get("gamma", 0.2)
        
        # 自适应模型参数，来自论文表1
        self.lambda_adapt = params.get("lambda", 0.8)
        self.sigma_adapt = params.get("sigma", 0.9)
        self.omega_star = params.get("omega_star", 0.02)
        
        # 跟踪微分器参数，来自论文表1
        self.t1 = params.get("t1", 0.02)
        self.t2 = params.get("t2", 0.04)
        
        # AMESO观测器参数
        self.l_eso = params.get("l", 5.0)
        
        # 安全限制参数
        self.int_max_xy = params.get("pxy_int_max", 0.5)
        self.int_max_z = params.get("pz_int_max", 0.5)
        
        # PID参数
        self.k_p = params.get("kp", 2.0)
        self.k_i = params.get("ki", 0.3)
        self.k_d = params.get("kd", 2.0)
        
        # 控制量限制
        self.max_tilt_angle = 10.0
        self.max_thrust = 1.0
        self.min_thrust = 0.1
        
        self.logger.info("AMESO-based SM Controller initialized with parameters from paper")
    
    def set_desired_state(self, desired_state: DesiredState):
        """设置期望状态"""
        self.desired_state = desired_state
    
    def set_current_state(self, current_state: CurrentState):
        """设置当前状态"""
        self.current_state = current_state
    
    def update(self, dt: float) -> ControlOutput:
        """基于AMESO论文算法的ADRC控制器更新函数 - 完全对应C++实现"""
        
        # 计算跟踪误差
        epsilon = self.current_state.pos - self.desired_state.pos
        
        # 初始化处理 - 完全对应C++初始化逻辑
        if not self.initialized:
            self.epsilon_n1 = epsilon[2]
            self.epsilon_n2 = self.current_state.vel[2] - self.desired_state.vel[2]
            self.epsilon_n1_init = epsilon[2]
            self.epsilon_n2_init = self.current_state.vel[2] - self.desired_state.vel[2]
            self.epsilon_bar1 = 0.0
            self.epsilon_bar2 = 0.0
            self.integral_term = 0.0
            self.integral_term_pid.fill(0.0)
            self.initialized = True
        
        # ===============================================
        # 以下是z轴输入计算 - 完全对应C++算法
        # ===============================================
        
        # 1. 计算可变指数项，根据论文公式中的β函数
        beta1 = self.compute_variable_exponent(self.epsilon_n1)
        beta2 = self.compute_variable_exponent(self.epsilon_n2)
        
        # 数值稳定性检查
        if abs(self.epsilon_n1) > 1e-6:
            self.epsilon_beta1 = sign(self.epsilon_n1) * pow(abs(self.epsilon_n1), beta1)
        else:
            self.epsilon_beta1 = self.epsilon_n1  # 小值时线性处理
        
        if abs(self.epsilon_n2) > 1e-6:
            self.epsilon_beta2 = sign(self.epsilon_n2) * pow(abs(self.epsilon_n2), beta2)
        else:
            self.epsilon_beta2 = self.epsilon_n2  # 小值时线性处理
        
        # 2. 构建积分滑模面 ISS，其初始值为0
        s = (self.c1 * self.epsilon_n1 + self.c2 * self.epsilon_n2 + 
             self.lambda_D * self.integral_term -
             self.c1 * self.epsilon_n1_init - self.c2 * self.epsilon_n2_init)
        
        # 更新积分项
        self.integral_term += (self.c1 * self.epsilon_beta1 + self.c2 * self.epsilon_beta2) * dt
        
        # 3. 标称控制律，根据论文公式
        u_n = (-(self.quad_mass * self.k / self.c2) * s -
               (self.quad_mass * self.c1 / self.c2) * self.epsilon_n2 +
               self.quad_mass * 9.8 +
               self.quad_mass * self.desired_state.acc[2] -
               self.lambda_D * self.quad_mass * self.c1 / self.c2 * self.epsilon_beta1 -
               self.lambda_D * self.quad_mass * self.epsilon_beta2)
        
        # 4. 反馈控制律u_f
        e_n = epsilon[2] - self.epsilon_n1  # 注意：C++源代码此处有歧义，应该是epsilon[2]
        e_n2 = self.epsilon_bar2 - self.epsilon_n2
        
        u_f = self.quad_mass * self.k1 * e_n + self.quad_mass * self.k2 * e_n2
        
        # 5. 计算自适应模型
        omega_hat_pre = self.omega_hat.copy()  # 保留上一步的omega_hat
        f_a = self.compute_adaptive_model(dt)
        
        # 6. 扰动补偿u_c
        max_omega_change = 0.0
        for i in range(self.n_basis):
            max_omega_change = max(abs(self.omega_hat[i] - omega_hat_pre[i]), max_omega_change)
        
        if max_omega_change > self.omega_star:
            p_t = 0.0  # 使用纯自适应模型
        else:
            p_t = 1.0  # 使用自适应模型+ESO
        
        f_z_hat = f_a + p_t * self.epsilon_hat3
        u_c = -self.quad_mass * f_z_hat
        
        # 7. 计算z轴总控制律
        u_z = u_n + u_f + u_c
        
        # 8. AMESO观测器
        self.update_ameso(dt, u_z, f_z_hat)
        
        # 9. 更新跟踪微分器TD，根据论文公式
        self.update_tracking_differentiator(dt)
        
        # 10. 更新标称系统，根据论文公式
        self.update_nominal_system(dt, u_n)
        
        # ===============================================
        # 以下计算x,y轴输入，PID控制 - 完全对应C++PID部分
        # ===============================================
        
        # 位置误差和速度误差
        epsilon_pos_PID = self.desired_state.pos - self.current_state.pos
        epsilon_vel_PID = self.desired_state.vel - self.current_state.vel
        
        # 限制最大误差
        for i in range(3):
            epsilon_pos_PID[i] = sat(epsilon_pos_PID[i], 3.0)
            epsilon_vel_PID[i] = sat(epsilon_vel_PID[i], 3.0)
        
        # 积分项计算（仅在小误差时启动积分）
        for i in range(2):  # XY轴
            if abs(epsilon_pos_PID[i]) < 0.2:
                self.integral_term_pid[i] += epsilon_pos_PID[i] * dt
                self.integral_term_pid[i] = sat(self.integral_term_pid[i], self.int_max_xy)
            else:
                self.integral_term_pid[i] = 0
        
        # Z轴积分
        if abs(epsilon_pos_PID[2]) < 0.5:
            self.integral_term_pid[2] += epsilon_pos_PID[2] * dt
            self.integral_term_pid[2] = sat(self.integral_term_pid[2], self.int_max_z)
        else:
            self.integral_term_pid[2] = 0
        
        # PID控制律：期望加速度 = 期望加速度 + Kp*位置误差 + Kd*速度误差 + Ki*积分项
        des_acc = (self.desired_state.acc + 
                   self.k_p * epsilon_pos_PID + 
                   self.k_d * epsilon_vel_PID + 
                   self.k_i * self.integral_term_pid)
        
        u_x = des_acc[0] * self.quad_mass
        u_y = des_acc[1] * self.quad_mass
        
        # 关键安全检查：防止除零和异常推力
        if abs(u_z) < 0.01:
            self.logger.error("ADRC: Critical thrust too small! Emergency fallback.")
            u_z = 0.5 * self.quad_mass * 9.8
            u_x = 0
            u_y = 0
        
        # 计算总拉力
        u_total = np.sqrt(u_x * u_x + u_y * u_y + u_z * u_z)
        Thrust_des = u_total
        
        # 计算姿态角 - 对应C++计算
        self.u_att[2] = self.desired_state.yaw
        
        # 防止除零
        if abs(Thrust_des/self.quad_mass) < 1e-6:
            Thrust_des = self.quad_mass * 9.8
        
        self.u_att[0] = np.arcsin(sat(
            (np.sin(self.u_att[2]) * u_x - np.cos(self.u_att[2]) * u_y) / (Thrust_des/self.quad_mass), 
            0.99))  # roll
        self.u_att[1] = np.arctan2(
            np.cos(self.u_att[2]) * u_x + np.sin(self.u_att[2]) * u_y, u_z)  # pitch
        
        # 油门计算方法选择 - 完全对应C++的switch语句
        if self.method_choose == 1:
            # 法一：过原点与点(hov_percent, mg)的线性关系（原方法）
            full_thrust = self.quad_mass * 9.8 / self.hov_percent
            self.u_att[3] = Thrust_des / full_thrust
        elif self.method_choose == 2:
            # 法二：线性关系
            Thr_x1 = self.hov_percent
            Thr_y1 = self.quad_mass * 9.8
            Thr_x2 = 1.0
            Thr_y2 = 2.5 * self.quad_mass * 9.8
            Thr_k = (Thr_y2 - Thr_y1) / (Thr_x2 - Thr_x1)
            Thr_b = Thr_y2 - Thr_k * Thr_x2
            self.u_att[3] = (Thrust_des - Thr_b) / Thr_k
        else:
            # 默认方法
            full_thrust = self.quad_mass * 9.8 / self.hov_percent
            self.u_att[3] = Thrust_des / full_thrust
        
        # 油门限制 - 对应C++限制
        if self.u_att[3] < 0.3:
            self.u_att[3] = 0.3
            self.logger.warning("throttle too low")
        
        if self.u_att[3] > 0.7:
            self.u_att[3] = 0.7
            self.logger.warning("throttle too high")
        
        return ControlOutput(
            timestamp=time.time(),
            roll=self.u_att[0],
            pitch=self.u_att[1],
            yaw=self.u_att[2],
            thrust=self.u_att[3]
        )
    
    def compute_basis_functions(self, z: float, z_dot: float) -> List[float]:
        """计算基函数向量，根据论文公式 - 完全对应C++实现"""
        phi = [0.0] * self.n_basis
        phi[0] = 1.0                     # 常数项
        phi[1] = np.sin(z)               # sin(z)
        phi[2] = np.sin(z_dot)           # sin(z_dot)
        phi[3] = np.cos(z)               # cos(z)
        phi[4] = np.cos(z_dot)           # cos(z_dot)
        phi[5] = np.sin(2.0 * z)         # sin(2z)
        phi[6] = np.sin(2.0 * z_dot)     # sin(2z_dot)
        phi[7] = np.cos(2.0 * z)         # cos(2z)
        phi[8] = np.cos(2.0 * z_dot)     # cos(2z_dot)
        return phi
    
    def compute_adaptive_model(self, dt: float) -> float:
        """计算自适应模型输出，根据论文公式 - 完全对应C++实现"""
        # 计算基函数向量
        z = self.current_state.pos[2]  # Z轴位置
        z_dot = self.epsilon_bar2 + self.desired_state.vel[2]  # Z轴速度估计
        
        phi = self.compute_basis_functions(z, z_dot)
        
        # 计算自适应模型输出
        f_a = 0.0
        for i in range(self.n_basis):
            f_a += self.omega_hat[i] * phi[i]
        
        # 更新自适应权重，根据论文公式
        e_n2_bar = self.epsilon_bar2 - self.epsilon_n2  # 误差项
        
        for i in range(self.n_basis):
            omega_dot = (self.lambda_adapt * e_n2_bar * phi[i] - 
                        self.sigma_adapt * self.lambda_adapt * abs(e_n2_bar) * self.omega_hat[i])
            self.omega_hat[i] += omega_dot * dt
        
        return f_a
    
    def update_tracking_differentiator(self, dt: float):
        """更新跟踪微分器，根据论文公式 - 完全对应C++实现"""
        epsilon = self.current_state.pos - self.desired_state.pos
        
        # 二阶跟踪微分器更新
        epsilon_bar1_dot = self.epsilon_bar2
        epsilon_bar2_dot = (-(1.0 / (self.t1 * self.t2)) * (self.epsilon_bar1 - epsilon[2]) -
                           (self.t1 + self.t2) / (self.t1 * self.t2) * self.epsilon_bar2)
        
        # 数值积分
        self.epsilon_bar1 += epsilon_bar1_dot * dt
        self.epsilon_bar2 += epsilon_bar2_dot * dt
    
    def update_nominal_system(self, dt: float, u_n: float):
        """更新标称系统，根据论文公式 - 完全对应C++实现"""
        # 标称系统状态方程
        epsilon_n1_dot = self.epsilon_n2
        epsilon_n2_dot = -9.8 + (1.0 / self.quad_mass) * u_n - self.desired_state.acc[2]
        
        # 数值积分更新状态
        self.epsilon_n1 += epsilon_n1_dot * dt
        self.epsilon_n2 += epsilon_n2_dot * dt
    
    def update_ameso(self, dt: float, u_z: float, f_z_hat: float):
        """更新AMESO观测器，根据论文公式 - 完全对应C++实现"""
        epsilon = self.current_state.pos - self.desired_state.pos
        
        # AMESO状态更新方程
        e_e1 = epsilon[2] - self.epsilon_hat1  # 观测误差
        
        # 三阶扩展状态观测器
        epsilon_hat1_dot = self.epsilon_hat2 + 3.0 * self.l_eso * e_e1
        epsilon_hat2_dot = (-9.8 + (1.0 / self.quad_mass) * u_z +
                           f_z_hat - self.desired_state.acc[2] +
                           3.0 * self.l_eso * self.l_eso * e_e1)
        epsilon_hat3_dot = self.l_eso * self.l_eso * self.l_eso * e_e1
        
        # 数值积分更新
        self.epsilon_hat1 += epsilon_hat1_dot * dt
        self.epsilon_hat2 += epsilon_hat2_dot * dt
        self.epsilon_hat3 += epsilon_hat3_dot * dt
    
    def compute_variable_exponent(self, epsilon: float) -> float:
        """计算可变指数，根据论文中的β函数 - 完全对应C++实现"""
        # 实现论文中的可变指数计算，增加边界检查
        if abs(epsilon) < 1e-6:
            return 1.0  # 返回默认值
        
        beta_i = 1.0 + min(self.beta_max, pow(abs(epsilon), self.gamma_param)) * (1.0 if abs(epsilon) > 1.0 else -1.0)
        
        # 确保指数在合理范围内
        beta_i = max(0.1, min(beta_i, self.beta_max + 1.0))
        
        return beta_i
    
    def printf_result(self):
        """打印控制结果 - 对应C++的printf_result"""
        self.logger.info("AMESO-based ADRC Controller - Roll: %.2f deg, Pitch: %.2f deg, Yaw: %.2f deg, Thrust: %.3f", 
                        self.u_att[0]*180/np.pi, self.u_att[1]*180/np.pi, 
                        self.u_att[2]*180/np.pi, self.u_att[3])