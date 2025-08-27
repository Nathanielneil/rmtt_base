"""
控制器测试脚本

用于验证三种控制器的正确性和参数设置
"""

import numpy as np
import time
import sys
from pathlib import Path

# Add the current directory to Python path
sys.path.insert(0, str(Path(__file__).parent))

from data_structures import DesiredState, CurrentState
from pid_controller import PID_Controller
from ude_controller import UDE_Controller
from adrc_controller import ADRC_Controller


def test_controller_creation():
    """测试控制器创建和初始化"""
    print("测试控制器创建...")
    
    # 测试参数（对应原始launch文件）
    params = {
        # PID参数
        "pid_gain/quad_mass": 0.087,  # RMTT实际质量87g
        "pid_gain/hov_percent": 0.5,
        "pid_gain/Kp_xy": 2.0,
        "pid_gain/Kp_z": 2.0,
        "pid_gain/Kv_xy": 2.0,
        "pid_gain/Kv_z": 2.0,
        "pid_gain/Kvi_xy": 0.3,
        "pid_gain/Kvi_z": 0.3,
        "pid_gain/tilt_angle_max": 10.0,
        "pid_gain/pxy_int_max": 0.5,
        "pid_gain/pz_int_max": 0.5,
        
        # UDE参数
        "ude_gain/quad_mass": 0.087,  # RMTT实际质量87g
        "ude_gain/hov_percent": 0.5,
        "ude_gain/Kp_xy": 0.5,
        "ude_gain/Kp_z": 0.5,
        "ude_gain/Kd_xy": 2.0,
        "ude_gain/Kd_z": 2.0,
        "ude_gain/T_ude": 1.0,
        "ude_gain/tilt_angle_max": 20.0,
        "ude_gain/pxy_int_max": 1.0,
        "ude_gain/pz_int_max": 1.0,
        
        # ADRC参数
        "ameso_gain/quad_mass": 0.087,  # RMTT实际质量87g
        "ameso_gain/hov_percent": 0.5,
        "ameso_gain/k": 0.8,
        "ameso_gain/k1": -0.15,
        "ameso_gain/k2": -3.0,
        "ameso_gain/c1": 1.5,
        "ameso_gain/c2": 0.6,
        "ameso_gain/lambda_D": 1.0,
        "ameso_gain/beta_max": 1.0,
        "ameso_gain/gamma": 0.2,
        "ameso_gain/lambda": 0.8,
        "ameso_gain/sigma": 0.9,
        "ameso_gain/omega_star": 0.02,
        "ameso_gain/t1": 0.02,
        "ameso_gain/t2": 0.04,
        "ameso_gain/l": 5.0,
        "ameso_gain/kp": 2.0,
        "ameso_gain/ki": 0.3,
        "ameso_gain/kd": 2.0,
        "ameso_gain/int_max_xy": 0.5,
        "ameso_gain/int_max_z": 0.5,
        "ameso_gain/pxy_int_max": 0.5,
        "ameso_gain/pz_int_max": 0.5,
    }
    
    try:
        # 创建PID控制器
        pid = PID_Controller()
        pid.init(params)
        print("✓ PID控制器创建成功")
        
        # 创建UDE控制器
        ude = UDE_Controller()
        ude.init(params)
        print("✓ UDE控制器创建成功")
        
        # 创建ADRC控制器
        adrc = ADRC_Controller()
        adrc.init(params)
        print("✓ ADRC控制器创建成功")
        
        return pid, ude, adrc
        
    except Exception as e:
        print(f"✗ 控制器创建失败: {e}")
        return None, None, None


def test_controller_update():
    """测试控制器更新功能"""
    print("\n测试控制器更新...")
    
    controllers = test_controller_creation()
    if None in controllers:
        return
    
    pid, ude, adrc = controllers
    
    # 创建测试状态
    desired = DesiredState(
        pos=np.array([0.0, 0.0, 1.0]),  # 目标高度1米
        vel=np.array([0.0, 0.0, 0.0]),
        acc=np.array([0.0, 0.0, 0.0]),
        yaw=0.0
    )
    
    current = CurrentState(
        pos=np.array([0.0, 0.0, 0.5]),  # 当前高度0.5米
        vel=np.array([0.0, 0.0, 0.0]),
        acc=np.array([0.0, 0.0, 0.0]),
        yaw=0.0
    )
    
    dt = 0.02  # 50Hz控制频率
    
    try:
        # 测试PID
        pid.set_desired_state(desired)
        pid.set_current_state(current)
        pid_output = pid.update(dt)
        print(f"✓ PID输出: roll={pid_output[0]:.3f}, pitch={pid_output[1]:.3f}, yaw={pid_output[2]:.3f}, thrust={pid_output[3]:.3f}")
        
        # 测试UDE
        ude.set_desired_state(desired)
        ude.set_current_state(current)
        ude_output = ude.update(dt)
        print(f"✓ UDE输出: roll={ude_output[0]:.3f}, pitch={ude_output[1]:.3f}, yaw={ude_output[2]:.3f}, thrust={ude_output[3]:.3f}")
        
        # 测试ADRC
        adrc.set_desired_state(desired)
        adrc.set_current_state(current)
        adrc_output = adrc.update(dt)
        print(f"✓ ADRC输出: roll={adrc_output[0]:.3f}, pitch={adrc_output[1]:.3f}, yaw={adrc_output[2]:.3f}, thrust={adrc_output[3]:.3f}")
        
    except Exception as e:
        print(f"✗ 控制器更新失败: {e}")


def test_parameter_accuracy():
    """验证参数设置的准确性"""
    print("\n验证参数准确性...")
    
    controllers = test_controller_creation()
    if None in controllers:
        return
    
    pid, ude, adrc = controllers
    
    # 验证关键参数
    print(f"PID quad_mass: {pid.quad_mass} (应为0.087)")
    print(f"PID kp_pos: {pid.kp_pos} (应为[2.0, 2.0, 2.0])")
    
    print(f"UDE quad_mass: {ude.quad_mass} (应为0.087)")
    print(f"UDE filter_param: {ude.filter_param} (应为1.0)")
    
    print(f"ADRC quad_mass: {adrc.quad_mass} (应为0.087)")
    print(f"ADRC k: {adrc.k} (应为0.8)")
    print(f"ADRC c1: {adrc.c1} (应为1.5)")
    print(f"ADRC lambda_adapt: {adrc.lambda_adapt} (应为0.8)")


def main():
    print("HX实验1 控制器测试")
    print("=" * 40)
    
    test_controller_creation()
    test_controller_update() 
    test_parameter_accuracy()
    
    print("\n测试完成!")


if __name__ == "__main__":
    main()