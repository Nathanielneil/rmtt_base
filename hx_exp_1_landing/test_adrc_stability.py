#!/usr/bin/env python3
"""
ADRC控制器稳定性测试脚本
专门验证ADRC数值稳定性和控制性能
"""

import sys
import time
import math
import numpy as np

# 添加路径
sys.path.append('.')
sys.path.append('..')

from adrc_controller import ADRC_Controller
from data_structures import DesiredState, CurrentState

def test_adrc_stability():
    """测试ADRC控制器稳定性"""
    print("=== ADRC控制器稳定性测试 ===")
    
    # 1. 创建ADRC控制器
    controller = ADRC_Controller()
    
    # 2. 使用保守参数配置
    conservative_params = {
        "quad_mass": 0.087,          # RMTT实际质量
        "hov_percent": 0.5,
        
        # 保守的滑模参数
        "k": 0.5,
        "k1": -0.1, 
        "k2": -2.0,
        "c1": 1.0,
        "c2": 0.8,
        "lambda_D": 0.5,
        "beta_max": 0.8,
        "gamma": 0.15,
        
        # 保守的自适应参数
        "lambda": 0.5,
        "sigma": 0.95,
        "omega_star": 0.01,
        
        # 增大滤波时间常数
        "t1": 0.05,
        "t2": 0.08,
        
        # 降低ESO增益
        "l": 3.0,
        
        # PID参数
        "kp": 1.5,
        "ki": 0.2, 
        "kd": 1.5,
    }
    
    controller.init(conservative_params)
    print("✓ ADRC控制器初始化成功")
    
    # 3. 测试不同场景的数值稳定性
    test_scenarios = [
        # (目标位置, 当前位置, 目标速度, 当前速度, 描述)
        ([0, 0, 1.0], [0, 0, 0], [0, 0, 0], [0, 0, 0], "起飞到1米"),
        ([0, 0, 1.0], [0, 0, 0.5], [0, 0, 0.2], [0, 0, 0.15], "上升过程"),
        ([0, 0, 1.0], [0, 0, 1.0], [0, 0, 0], [0, 0, 0], "悬停在1米"),
        ([0, 0, 0.5], [0, 0, 1.0], [0, 0, -0.1], [0, 0, 0], "开始降落"),
        ([0, 0, 0.0], [0, 0, 0.5], [0, 0, -0.1], [0, 0, -0.05], "降落过程"),
    ]
    
    dt = 0.02  # 50Hz控制频率
    
    for i, (target_pos, current_pos, target_vel, current_vel, description) in enumerate(test_scenarios):
        print(f"\n--- 场景 {i+1}: {description} ---")
        
        # 设置期望状态
        desired_state = DesiredState()
        desired_state.pos = np.array(target_pos, dtype=float)
        desired_state.vel = np.array(target_vel, dtype=float)
        desired_state.acc = np.array([0, 0, 0], dtype=float)
        desired_state.yaw = 0.0
        
        # 设置当前状态
        current_state = CurrentState()
        current_state.pos = np.array(current_pos, dtype=float)
        current_state.vel = np.array(current_vel, dtype=float)
        current_state.yaw = 0.0
        current_state.q = np.array([1, 0, 0, 0], dtype=float)  # 单位四元数
        
        controller.set_desired_state(desired_state)
        controller.set_current_state(current_state)
        
        # 运行控制器多个步骤
        stable = True
        for step in range(10):
            try:
                control_output = controller.update(dt)
                
                # 检查输出是否合理
                if (math.isnan(control_output[0]) or math.isnan(control_output[1]) or 
                    math.isnan(control_output[2]) or math.isnan(control_output[3])):
                    print(f"  ✗ 步骤 {step}: NaN输出")
                    stable = False
                    break
                
                if abs(control_output[3]) > 1.0 or control_output[3] < 0.0:
                    print(f"  ✗ 步骤 {step}: 推力超范围 {control_output[3]:.3f}")
                    stable = False
                    break
                    
                if step == 0:  # 只打印第一步
                    print(f"  控制输出: Roll={math.degrees(control_output[0]):.1f}°, "
                          f"Pitch={math.degrees(control_output[1]):.1f}°, "
                          f"Yaw={math.degrees(control_output[2]):.1f}°, "
                          f"Thrust={control_output[3]:.3f}")
                
            except Exception as e:
                print(f"  ✗ 步骤 {step}: 异常 {e}")
                stable = False
                break
        
        if stable:
            print(f"  ✓ {description} - 数值稳定")
        else:
            print(f"  ✗ {description} - 数值不稳定")
    
    # 4. 推力范围测试
    print(f"\n--- 推力范围测试 ---")
    test_positions = [0.0, 0.5, 1.0, 1.5, 2.0]  # 不同高度
    
    for height in test_positions:
        desired_state.pos[2] = height
        current_state.pos[2] = height
        controller.set_desired_state(desired_state) 
        controller.set_current_state(current_state)
        
        control_output = controller.update(dt)
        thrust = control_output[3]
        
        print(f"  高度 {height}m: 推力 {thrust:.3f} ({thrust*100:.1f}%)")
    
    print(f"\n=== ADRC稳定性测试完成 ===")

if __name__ == "__main__":
    test_adrc_stability()