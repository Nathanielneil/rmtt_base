#!/usr/bin/env python3
"""
快速项目检查脚本 - 不依赖外部硬件库
专注于核心代码逻辑检查
"""

import sys
import numpy as np
import traceback

def check_controller_safety():
    """检查控制器安全初始化"""
    print("=== 1. 检查控制器安全初始化 ===")
    
    try:
        from pid_controller import PID_Controller
        from ude_controller import UDE_Controller
        from adrc_controller import ADRC_Controller
        
        controllers = {
            'PID': PID_Controller(),
            'UDE': UDE_Controller(), 
            'ADRC': ADRC_Controller()
        }
        
        # 检查构造函数默认值是否安全
        for name, controller in controllers.items():
            mass = getattr(controller, 'quad_mass', None)
            hov_percent = getattr(controller, 'hov_percent', None)
            
            # 检查质量参数
            if mass is None or mass <= 0.0:
                print(f"✗ {name} - quad_mass不安全: {mass}")
                return False
            else:
                print(f"✓ {name} - quad_mass安全: {mass}kg")
            
            # 检查悬停参数
            if hov_percent is None or hov_percent <= 0.0:
                print(f"✗ {name} - hov_percent不安全: {hov_percent}")
                return False
            else:
                print(f"✓ {name} - hov_percent安全: {hov_percent}")
            
        return True
        
    except Exception as e:
        print(f"✗ 控制器安全检查失败: {e}")
        traceback.print_exc()
        return False

def check_thrust_conversion():
    """检查推力转换逻辑"""
    print("\n=== 2. 检查推力转换逻辑 ===")
    
    try:
        from data_structures import ControlOutput
        
        # 测试推力转换
        test_cases = [
            (0.5, 0, "悬停点"),
            (0.6, 20, "轻微上升"),  
            (0.4, -20, "轻微下降"),
            (1.0, 100, "最大上升"),
            (0.0, -100, "最大下降")
        ]
        
        for thrust, expected_throttle, description in test_cases:
            control_output = ControlOutput(0.0, 0.0, 0.0, thrust)
            _, _, throttle_cmd, _ = control_output.as_rc_command()
            
            # 允许一定误差
            if abs(throttle_cmd - expected_throttle) > 5:
                print(f"✗ {description}: thrust={thrust} -> throttle={throttle_cmd} (期望≈{expected_throttle})")
                return False
            else:
                print(f"✓ {description}: thrust={thrust} -> throttle={throttle_cmd}")
        
        return True
        
    except Exception as e:
        print(f"✗ 推力转换检查失败: {e}")
        traceback.print_exc()
        return False

def check_controller_updates():
    """检查控制器更新功能"""
    print("\n=== 3. 检查控制器更新功能 ===")
    
    try:
        from pid_controller import PID_Controller
        from ude_controller import UDE_Controller
        from adrc_controller import ADRC_Controller
        from data_structures import DesiredState, CurrentState
        
        # 创建测试状态
        desired_state = DesiredState()
        desired_state.pos = np.array([0, 0, 1.0])  # 目标1米高度
        desired_state.vel = np.zeros(3)
        desired_state.acc = np.zeros(3)
        desired_state.yaw = 0.0
        
        current_state = CurrentState()
        current_state.pos = np.array([0, 0, 0.5])  # 当前0.5米
        current_state.vel = np.zeros(3)
        current_state.yaw = 0.0
        # 使用R类创建正确的旋转对象
        try:
            from scipy.spatial.transform import Rotation as R
            current_state.q = R.from_quat([0, 0, 0, 1])  # 单位四元数 [x,y,z,w]
        except ImportError:
            from data_structures import R
            current_state.q = R.from_euler('z', 0.0)  # 零旋转
        
        controllers = [
            ('PID', PID_Controller()),
            ('UDE', UDE_Controller()), 
            ('ADRC', ADRC_Controller())
        ]
        
        # 初始化所有控制器参数
        test_params = {
            "pid_gain/quad_mass": 0.087,
            "pid_gain/hov_percent": 0.5,
            "ude_gain/quad_mass": 0.087,
            "ude_gain/hov_percent": 0.5,
            "ude_gain/T_ude": 1.0,
            "ameso_gain/quad_mass": 0.087,
            "ameso_gain/hov_percent": 0.5,
            "ameso_gain/omega_star": 0.02
        }
        
        dt = 0.02  # 50Hz
        
        for name, controller in controllers:
            try:
                # 初始化控制器参数
                controller.init(test_params)
                controller.set_desired_state(desired_state)
                controller.set_current_state(current_state)
                
                # 测试更新
                control_output = controller.update(dt)
                
                # 检查输出格式
                if not isinstance(control_output, np.ndarray):
                    print(f"✗ {name} - 输出类型错误: {type(control_output)}")
                    return False
                    
                if len(control_output) != 4:
                    print(f"✗ {name} - 输出长度错误: {len(control_output)}")
                    return False
                    
                # 检查数值合理性
                roll, pitch, yaw, thrust = control_output
                
                if np.isnan(control_output).any() or np.isinf(control_output).any():
                    print(f"✗ {name} - 输出包含NaN/Inf")
                    return False
                
                if thrust < 0 or thrust > 1:
                    print(f"✗ {name} - 推力超范围: {thrust}")
                    return False
                    
                print(f"✓ {name} - 更新成功，推力: {thrust:.3f}")
                
            except Exception as e:
                print(f"✗ {name} - 更新失败: {e}")
                return False
        
        return True
        
    except Exception as e:
        print(f"✗ 控制器更新检查失败: {e}")
        traceback.print_exc()
        return False

def check_parameter_consistency():
    """检查参数一致性（简化版）"""
    print("\n=== 4. 检查参数一致性 ===")
    
    # 检查关键参数值
    expected_values = {
        'quad_mass': 0.087,
        'hov_percent': 0.5
    }
    
    try:
        from pid_controller import PID_Controller
        from ude_controller import UDE_Controller
        from adrc_controller import ADRC_Controller
        
        controllers = {
            'PID': PID_Controller(),
            'UDE': UDE_Controller(), 
            'ADRC': ADRC_Controller()
        }
        
        for param, expected_value in expected_values.items():
            values = []
            for name, controller in controllers.items():
                value = getattr(controller, param, None)
                values.append((name, value))
                
            # 检查是否一致
            all_same = all(abs(v[1] - expected_value) < 1e-6 for v in values if v[1] is not None)
            
            if all_same:
                print(f"✓ {param} 参数一致: {expected_value}")
            else:
                print(f"✗ {param} 参数不一致:")
                for name, value in values:
                    print(f"    {name}: {value}")
                return False
        
        return True
        
    except Exception as e:
        print(f"✗ 参数一致性检查失败: {e}")
        return False

def main():
    """主检查函数"""
    print("🔍 快速项目检查 (无硬件依赖版)...")
    print("=" * 50)
    
    checks = [
        check_controller_safety,
        check_thrust_conversion,
        check_controller_updates,
        check_parameter_consistency
    ]
    
    passed = 0
    total = len(checks)
    
    for check_func in checks:
        try:
            if check_func():
                passed += 1
        except Exception as e:
            print(f"⚠️ 检查异常: {check_func.__name__} - {e}")
    
    print("\n" + "=" * 50)
    print(f"🎯 检查结果: {passed}/{total} 项通过")
    
    if passed == total:
        print("✅ 核心代码检查通过! 可以进行实验")
        print("\n📝 建议:")
        print("1. 在配置好的环境中运行完整测试")
        print("2. 先用PID控制器进行基础测试")
        print("3. 确认推力响应正常后再测试ADRC")
        return True
    else:
        print("❌ 发现问题，需要修复")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)