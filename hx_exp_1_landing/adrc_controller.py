"""
ADRC控制器 - 严格对应原始C++实现

基于AMESO论文的完整ADRC控制器实现，严格保持所有原始算法细节
"""

import numpy as np
import time
import logging
import math
from typing import Dict, Any, List
from data_structures import DesiredState, CurrentState, ControlOutput, sign, sat, R


class ADRC_Controller:
    """ADRC控制器类 - 严格对应C++的ADRC_Controller类"""
    
    def __init__(self):
        """构造函数 - 严格对应C++构造函数第347-384行"""
        
        # 输出初始化 - 对应C++第363行
        self.u_att_ = np.zeros(4)  # Eigen::Vector4d u_att_
        
        # 1维状态变量初始化 - 对应C++第365-377行的修改后参数
        self.epsilon_hat1 = 0.0      # AMESO观测器状态1
        self.epsilon_hat2 = 0.0      # AMESO观测器状态2  
        self.epsilon_hat3 = 0.0      # AMESO观测器状态3
        self.epsilon_bar1 = 0.0      # 跟踪微分器状态1
        self.epsilon_bar2 = 0.0      # 跟踪微分器状态2
        self.epsilon_beta1 = 0.0     # 可变指数项1
        self.epsilon_beta2 = 0.0     # 可变指数项2
        self.integral_term = 0.0     # 滑模面积分项
        self.epsilon_n1 = 0.0        # 标称位置误差
        self.epsilon_n2 = 0.0        # 标称速度误差
        self.epsilon_n1_init = 0.0   # 初始标称位置误差
        self.epsilon_n2_init = 0.0   # 初始标称速度误差
        self.method_choose = 1       # 油门方式选择 - 对应C++第377行
        
        # 基本参数 - 严格对应.h文件第124-131行定义
        self.quad_mass = 0.0         # double quad_mass; 无人机质量
        self.hov_percent = 0.0       # double hov_percent; 悬停油门百分比
        self.max_tilt_angle = 0.0    # double max_tilt_angle; 最大倾斜角度
        self.max_thrust = 0.0        # double max_thrust; 最大推力
        self.min_thrust = 0.0        # double min_thrust; 最小推力
        self.int_max_xy = 0.0        # double int_max_xy; XY积分限制
        self.int_max_z = 0.0         # double int_max_z; Z积分限制
        
        # 滑模控制参数 - 严格对应.h文件第133-139行定义
        self.k = 0.0                 # double k; 滑模增益
        self.k1 = 0.0                # double k1; 反馈增益参数
        self.k2 = 0.0                # double k2; 反馈增益参数
        self.c1 = 0.0                # double c1; 滑模面系数
        self.c2 = 0.0                # double c2; 滑模面系数
        self.lambda_D = 0.0          # double lambda_D; 积分参数
        self.beta_max = 0.0          # double beta_max; 最大指数值
        self.gamma_param = 0.0       # double gamma_param; 指数参数
        
        # 自适应模型参数 - 严格对应.h文件第141-146行定义
        self.lambda_adapt = 0.0      # double lambda_adapt; 自适应学习率
        self.sigma_adapt = 0.0       # double sigma_adapt; 收缩因子
        self.omega_star = 0.0        # double omega_star; 阈值参数
        self.n_basis = 9             # static const int n_basis = 9; 基函数数量
        self.omega_hat = [0.0] * 9   # std::vector<double> omega_hat; 自适应权重参数
        
        # 跟踪微分器TD参数 - 严格对应.h文件第148-154行定义
        self.t1 = 0.0                # double t1; TD时间常数
        self.t2 = 0.0                # double t2; TD时间常数
        
        # AMESO观测器参数 - 严格对应.h文件第156-164行定义
        self.l_eso = 0.0             # double l_eso; ESO增益参数
        
        # 新增PID参数 - 严格对应.h文件第191-199行定义
        self.k_p = 0.0               # double k_p; 比例增益
        self.k_i = 0.0               # double k_i; 积分增益
        self.k_d = 0.0               # double k_d; 微分增益
        self.integral_term_pid = np.zeros(3)  # Eigen::Vector3d integral_term_pid; pid中的积分项
        
        # 内部状态变量 - 严格对应.h文件第186-189行定义
        self.desired_state_ = None   # Desired_State desired_state_
        self.current_state_ = None   # Current_State current_state_
        self.initialized_ = False    # bool initialized_; 初始化标志
        
        # 初始化自适应权重向量 - 对应C++第380行
        # omega_hat.resize(n_basis, 0.0); 已在上面初始化
        
        self.logger = logging.getLogger("ADRC_Controller")
    
    def init(self, params: Dict[str, Any]):
        """初始化函数 - 严格对应C++的init函数第388-433行"""
        
        # 基本参数从参数字典读取 - 对应C++第390-392行
        self.quad_mass = params.get("ameso_gain/quad_mass", 2.5)
        self.hov_percent = params.get("ameso_gain/hov_percent", 0.5)
        
        # 滑模控制参数，来自论文表1 - 对应C++第394-402行
        self.k = params.get("ameso_gain/k", 0.8)
        self.k1 = params.get("ameso_gain/k1", -0.15)
        self.k2 = params.get("ameso_gain/k2", -3.0)
        self.c1 = params.get("ameso_gain/c1", 1.5)
        self.c2 = params.get("ameso_gain/c2", 0.6)
        self.lambda_D = params.get("ameso_gain/lambda_D", 1.0)
        self.beta_max = params.get("ameso_gain/beta_max", 1.0)
        self.gamma_param = params.get("ameso_gain/gamma", 0.2)
        
        # 自适应模型参数，来自论文表1 - 对应C++第404-407行
        self.lambda_adapt = params.get("ameso_gain/lambda", 0.8)
        self.sigma_adapt = params.get("ameso_gain/sigma", 0.9)
        self.omega_star = params.get("ameso_gain/omega_star", 1.0)
        
        # 跟踪微分器参数，来自论文表1 - 对应C++第409-411行
        self.t1 = params.get("ameso_gain/t1", 0.02)
        self.t2 = params.get("ameso_gain/t2", 0.04)
        
        # AMESO观测器参数 - 对应C++第413-414行
        self.l_eso = params.get("ameso_gain/l", 5.0)
        
        # 安全限制参数 - 对应C++第416-418行
        self.int_max_xy = params.get("ameso_gain/pxy_int_max", 0.5)
        self.int_max_z = params.get("ameso_gain/pz_int_max", 0.5)
        
        # PID参数 - 对应C++第420-425行
        self.k_p = params.get("ameso_gain/kp", 2.0)
        self.k_i = params.get("ameso_gain/ki", 0.3)
        self.k_d = params.get("ameso_gain/kd", 2.0)
        # 注意：这里有重复的参数定义，保持原始逻辑
        
        # 控制量限制 - 对应C++第427-430行
        self.max_tilt_angle = 10.0   # 最大倾斜角10度
        self.max_thrust = 1.0        # 最大推力
        self.min_thrust = 0.1        # 最小推力
        
        # 对应C++的ROS_INFO第432行
        self.logger.info("AMESO-based SM Controller initialized with parameters from paper")
    
    def set_desired_state(self, desired_state: DesiredState):
        """设置期望状态 - 严格对应C++函数第435-437行"""
        self.desired_state_ = desired_state
    
    def set_current_state(self, current_state: CurrentState):
        """设置当前状态 - 严格对应C++函数第440-442行"""
        self.current_state_ = current_state
    
    def update(self, dt: float) -> np.ndarray:
        """更新函数 - 严格对应C++的update函数第459-676行"""
        
        # 计算跟踪误差 - 对应C++第461-462行
        epsilon = self.current_state_.pos - self.desired_state_.pos
        
        # 初始化处理 - 严格对应C++第464-475行
        if not self.initialized_:
            self.epsilon_n1 = epsilon[2]
            self.epsilon_n2 = self.current_state_.vel[2] - self.desired_state_.vel[2]
            self.epsilon_n1_init = epsilon[2]
            self.epsilon_n2_init = self.current_state_.vel[2] - self.desired_state_.vel[2]
            self.epsilon_bar1 = 0.0
            self.epsilon_bar2 = 0.0
            self.integral_term = 0.0  # 重置积分项
            self.integral_term_pid = np.zeros(3)  # 对应C++第473行（有语法错误但保持原逻辑）
            self.initialized_ = True
        
        # ///////////////////////////////////////////////以下是z轴输入计算 - 对应C++第480行注释
        
        # 1. 计算可变指数项，根据论文公式中的β函数 - 对应C++第481-496行
        beta1 = self.compute_variable_exponent(self.epsilon_n1)
        beta2 = self.compute_variable_exponent(self.epsilon_n2)
        
        # 数值稳定性检查 - 严格对应C++第485-496行
        if abs(self.epsilon_n1) > 1e-6:
            self.epsilon_beta1 = sign(self.epsilon_n1) * pow(abs(self.epsilon_n1), beta1)
        else:
            self.epsilon_beta1 = self.epsilon_n1  # 小值时线性处理
        
        if abs(self.epsilon_n2) > 1e-6:
            self.epsilon_beta2 = sign(self.epsilon_n2) * pow(abs(self.epsilon_n2), beta2)
        else:
            self.epsilon_beta2 = self.epsilon_n2  # 小值时线性处理
        
        # 2. 构建积分滑模面 ISS，其初始值为0 - 对应C++第498-504行
        s = (self.c1 * self.epsilon_n1 + self.c2 * self.epsilon_n2 + 
             self.lambda_D * self.integral_term -
             self.c1 * self.epsilon_n1_init - self.c2 * self.epsilon_n2_init)
        
        # 更新积分项 - 对应C++第504行
        self.integral_term += (self.c1 * self.epsilon_beta1 + self.c2 * self.epsilon_beta2) * dt
        
        # 3. 标称控制律，根据论文公式 - 对应C++第506-513行
        u_n = (-(self.quad_mass * self.k / self.c2) * s - 
               (self.quad_mass * self.c1 / self.c2) * self.epsilon_n2 + 
               self.quad_mass * 9.8 +
               self.quad_mass * self.desired_state_.acc[2] -
               self.lambda_D * self.quad_mass * self.c1 / self.c2 * self.epsilon_beta1 -
               self.lambda_D * self.quad_mass * self.epsilon_beta2)
        
        # 4. 反馈控制律u_f - 对应C++第515-520行
        e_n = epsilon[2] - self.epsilon_n1  # 注意：这里C++代码有问题，但保持原逻辑
        e_n2 = self.epsilon_bar2 - self.epsilon_n2
        u_f = self.quad_mass * self.k1 * e_n + self.quad_mass * self.k2 * e_n2
        
        # 5. 计算自适应模型 - 对应C++第522-524行
        omega_hat_pre = self.omega_hat.copy()  # 保留上一步的omega_hat
        f_a = self.compute_adaptive_model(dt)
        
        # 6. 扰动补偿u_c - 对应C++第526-539行
        max_omega_change = 0.0
        for i in range(self.n_basis):
            max_omega_change = max(abs(self.omega_hat[i] - omega_hat_pre[i]), max_omega_change)
        
        p_t = 0.0
        if max_omega_change > self.omega_star:
            p_t = 0.0  # 使用纯自适应模型
        else:
            p_t = 1.0  # 使用自适应模型+ESO
        
        f_z_hat = f_a + p_t * self.epsilon_hat3
        u_c = -self.quad_mass * f_z_hat
        
        # 7. 计算z轴总控制律 - 对应C++第541-542行
        u_z = u_n + u_f + u_c
        
        # 8. AMESO观测器 - 对应C++第544-545行
        self.update_ameso(dt, u_z, f_z_hat)
        
        # 9. 更新跟踪微分器TD，根据论文公式 - 对应C++第547-548行
        self.update_tracking_differentiator(dt)
        
        # 10. 更新标称系统，根据论文公式 - 对应C++第550-551行
        self.update_nominal_system(dt, u_n)
        
        # ////////////////////////////////////////////////////////////以下计算x,y轴输入，PID控制 - 对应C++第554行注释
        
        # 位置误差和速度误差 - 对应C++第555-557行
        epsilon_pos_PID = self.desired_state_.pos - self.current_state_.pos
        epsilon_vel_PID = self.desired_state_.vel - self.current_state_.vel
        
        # 限制最大误差 - 对应C++第559-564行（注意原代码有变量名错误但保持逻辑）
        pos_error = np.zeros(3)
        vel_error = np.zeros(3)
        for i in range(3):
            pos_error[i] = sat(epsilon_pos_PID[i], 3.0)
            vel_error[i] = sat(epsilon_vel_PID[i], 3.0)
        
        # 积分项计算（仅在小误差时启动积分）- 对应C++第566-589行
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
        
        # PID控制律 - 对应C++第591-592行
        des_acc = (self.desired_state_.acc + 
                   self.k_p * epsilon_pos_PID + 
                   self.k_d * epsilon_vel_PID + 
                   self.k_i * self.integral_term_pid)
        
        # 计算x,y轴控制量 - 对应C++第594-595行
        u_x = des_acc[0] * self.quad_mass
        u_y = des_acc[1] * self.quad_mass
        
        # 关键安全检查 - 对应C++第597-604行
        if abs(u_z) < 0.01:  # 防止除零
            self.logger.error("ADRC: Critical thrust too small! Emergency fallback.")
            u_z = 0.5 * self.quad_mass * 9.8  # 紧急回落到悬停推力
            u_x = 0
            u_y = 0
        
        # 计算总拉力和姿态角 - 对应C++第606-614行
        u_total = math.sqrt(u_x * u_x + u_y * u_y + u_z * u_z)
        Thrust_des = u_total
        
        self.u_att_[2] = self.desired_state_.yaw
        self.u_att_[0] = math.asin((math.sin(self.u_att_[2]) * u_x - math.cos(self.u_att_[2]) * u_y) / (Thrust_des / self.quad_mass))
        self.u_att_[1] = math.atan((math.cos(self.u_att_[2]) * u_x + math.sin(self.u_att_[2]) * u_y) / u_z)
        
        # 油门计算方法选择 - 严格对应C++第616-661行的switch语句
        if self.method_choose == 1:
            # 法一：过原点与点(hov_percent, mg)的线性关系（原方法）
            full_thrust = self.quad_mass * 9.8 / self.hov_percent
            # 油门 = 期望推力/最大推力
            self.u_att_[3] = Thrust_des / full_thrust
        elif self.method_choose == 2:
            # 法二：线性关系
            Thr_x1 = self.hov_percent
            Thr_y1 = self.quad_mass * 9.8
            Thr_x2 = 1.0
            Thr_y2 = 2.5 * self.quad_mass * 9.8
            Thr_k = (Thr_y2 - Thr_y1) / (Thr_x2 - Thr_x1)
            Thr_b = Thr_y2 - Thr_k * Thr_x2
            self.u_att_[3] = (Thrust_des - Thr_b) / Thr_k
        else:
            # 默认方法
            full_thrust = self.quad_mass * 9.8 / self.hov_percent
            self.u_att_[3] = Thrust_des / full_thrust
        
        # 油门限制 - 对应C++第663-673行
        if self.u_att_[3] < 0.3:
            self.u_att_[3] = 0.3
            self.logger.warning("throttle too low")
        
        if self.u_att_[3] > 0.7:
            self.u_att_[3] = 0.7
            self.logger.warning("throttle too high")
        
        return self.u_att_
    
    def printf_result(self):
        """打印控制结果 - 严格对应C++的printf_result函数第678-681行"""
        self.logger.info("AMESO-based ADRC Controller - Roll: %.2f deg, Pitch: %.2f deg, Yaw: %.2f deg, Thrust: %.3f", 
                        self.u_att_[0]*180/math.pi, self.u_att_[1]*180/math.pi, 
                        self.u_att_[2]*180/math.pi, self.u_att_[3])
    
    def compute_basis_functions(self, z: float, z_dot: float) -> List[float]:
        """计算基函数向量 - 严格对应C++函数第684-699行"""
        phi = [0.0] * self.n_basis
        phi[0] = 1.0                          # 常数项
        phi[1] = math.sin(z)                  # sin(z)
        phi[2] = math.sin(z_dot)              # sin(z_dot)
        phi[3] = math.cos(z)                  # cos(z)
        phi[4] = math.cos(z_dot)              # cos(z_dot)
        phi[5] = math.sin(2.0 * z)            # sin(2z)
        phi[6] = math.sin(2.0 * z_dot)        # sin(2z_dot)
        phi[7] = math.cos(2.0 * z)            # cos(2z)
        phi[8] = math.cos(2.0 * z_dot)        # cos(2z_dot)
        return phi
    
    def compute_adaptive_model(self, dt: float) -> float:
        """计算自适应模型输出 - 严格对应C++函数第701-726行"""
        # 计算基函数向量 - 对应C++第704-706行
        z = self.current_state_.pos[2]           # Z轴位置
        z_dot = self.epsilon_bar2 + self.desired_state_.vel[2]  # Z轴速度估计 = TD输出 + 期望速度
        
        phi = self.compute_basis_functions(z, z_dot)
        
        # 计算自适应模型输出 - 对应C++第710-713行
        f_a = 0.0
        for i in range(self.n_basis):
            f_a += self.omega_hat[i] * phi[i]
        
        # 更新自适应权重，根据论文公式 - 对应C++第715-723行
        e_n2_bar = self.epsilon_bar2 - self.epsilon_n2  # 误差项
        
        for i in range(self.n_basis):
            omega_dot = (self.lambda_adapt * e_n2_bar * phi[i] - 
                        self.sigma_adapt * self.lambda_adapt * abs(e_n2_bar) * self.omega_hat[i])
            self.omega_hat[i] += omega_dot * dt  # 使用实际控制周期积分
        
        return f_a
    
    def update_tracking_differentiator(self, dt: float):
        """更新跟踪微分器 - 严格对应C++函数第728-742行"""
        # 计算跟踪误差 - 对应C++第731-732行
        epsilon = self.current_state_.pos - self.desired_state_.pos
        
        # 二阶跟踪微分器更新 - 对应C++第734-741行
        epsilon_bar1_dot = self.epsilon_bar2
        epsilon_bar2_dot = (-(1.0 / (self.t1 * self.t2)) * (self.epsilon_bar1 - epsilon[2]) - 
                           (self.t1 + self.t2) / (self.t1 * self.t2) * self.epsilon_bar2)
        
        # 数值积分 - 对应C++第740-741行
        self.epsilon_bar1 += epsilon_bar1_dot * dt
        self.epsilon_bar2 += epsilon_bar2_dot * dt
    
    def update_nominal_system(self, dt: float, u_n: float):
        """更新标称系统 - 严格对应C++函数第745-757行"""
        # 标称系统状态方程 - 对应C++第748-752行
        epsilon_n1_dot = self.epsilon_n2
        epsilon_n2_dot = -9.8 + (1.0 / self.quad_mass) * u_n - self.desired_state_.acc[2]
        
        # 数值积分更新状态 - 对应C++第754-756行
        self.epsilon_n1 += epsilon_n1_dot * dt
        self.epsilon_n2 += epsilon_n2_dot * dt
    
    def update_ameso(self, dt: float, u_z: float, f_z_hat: float):
        """更新AMESO观测器 - 严格对应C++函数第759-780行"""
        # 计算跟踪误差 - 对应C++第762-763行
        epsilon = self.current_state_.pos - self.desired_state_.pos
        
        # AMESO状态更新方程 - 对应C++第765-777行
        e_e1 = epsilon[2] - self.epsilon_hat1  # 观测误差
        
        # 三阶扩展状态观测器
        epsilon_hat1_dot = self.epsilon_hat2 + 3.0 * self.l_eso * e_e1
        epsilon_hat2_dot = (-9.8 + (1.0 / self.quad_mass) * u_z + 
                           f_z_hat - self.desired_state_.acc[2] + 
                           3.0 * self.l_eso * self.l_eso * e_e1)
        epsilon_hat3_dot = self.l_eso * self.l_eso * self.l_eso * e_e1
        
        # 数值积分更新 - 对应C++第775-778行
        self.epsilon_hat1 += epsilon_hat1_dot * dt
        self.epsilon_hat2 += epsilon_hat2_dot * dt
        self.epsilon_hat3 += epsilon_hat3_dot * dt
    
    def compute_variable_exponent(self, epsilon: float) -> float:
        """计算可变指数 - 严格对应C++函数第782-797行"""
        # 实现论文中的可变指数计算，增加边界检查 - 对应C++第785-787行
        if abs(epsilon) < 1e-6:
            return 1.0  # 返回默认值
        
        # 对应C++第789-791行
        beta_i = 1.0 + min(self.beta_max, pow(abs(epsilon), self.gamma_param)) * (1.0 if abs(epsilon) > 1.0 else -1.0)
        
        # 确保指数在合理范围内 - 对应C++第793-794行
        beta_i = max(0.1, min(beta_i, self.beta_max + 1.0))
        
        return beta_i