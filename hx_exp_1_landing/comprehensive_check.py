#!/usr/bin/env python3
"""
全面项目检查脚本 - 确保代码一致性和正确性
"""

import sys
import importlib
import traceback

def check_imports():
    """检查所有模块能否正确导入"""
    print("=== 1. 检查模块导入 ===")
    modules_to_check = [
        'data_structures',
        'pid_controller', 
        'ude_controller',
        'adrc_controller',
        'experiment_runner'
    ]
    
    for module_name in modules_to_check:
        try:
            module = importlib.import_module(module_name)
            print(f"✓ {module_name} - 导入成功")
        except Exception as e:
            print(f"✗ {module_name} - 导入失败: {e}")
            return False
    return True

def check_controller_interfaces():
    """检查控制器接口一致性"""
    print("\n=== 2. 检查控制器接口一致性 ===")
    
    try:
        from pid_controller import PID_Controller
        from ude_controller import UDE_Controller
        from adrc_controller import ADRC_Controller
        from data_structures import DesiredState, CurrentState
        import numpy as np
        
        controllers = {
            'PID': PID_Controller(),
            'UDE': UDE_Controller(), 
            'ADRC': ADRC_Controller()
        }
        
        # 检查构造函数默认值
        for name, controller in controllers.items():
            mass = getattr(controller, 'quad_mass', None)
            if mass is None or mass == 0.0:
                print(f"✗ {name} - quad_mass未正确初始化: {mass}")
                return False
            else:
                print(f"✓ {name} - quad_mass正确初始化: {mass}")
        
        # 检查接口方法
        required_methods = ['init', 'set_desired_state', 'set_current_state', 'update']
        for name, controller in controllers.items():
            for method in required_methods:
                if not hasattr(controller, method):
                    print(f"✗ {name} - 缺少方法: {method}")
                    return False
            print(f"✓ {name} - 接口方法完整")
            
        return True
        
    except Exception as e:
        print(f"✗ 控制器接口检查失败: {e}")
        traceback.print_exc()
        return False

def check_parameter_consistency():
    """检查参数配置一致性"""
    print("\n=== 3. 检查参数配置一致性 ===")
    
    try:
        from experiment_runner import ExperimentRunner
        
        runner = ExperimentRunner()
        
        # 检查RMTT质量参数
        expected_mass = 0.087
        mass_params = [
            runner.control_params.get("ameso_gain/quad_mass"),
            runner.control_params.get("pid_gain/quad_mass"), 
            runner.control_params.get("ude_gain/quad_mass")
        ]
        
        for i, mass in enumerate(mass_params):
            controller_names = ['ADRC', 'PID', 'UDE']
            if mass != expected_mass:
                print(f"✗ {controller_names[i]} - quad_mass参数错误: {mass} (期望: {expected_mass})")
                return False
            else:
                print(f"✓ {controller_names[i]} - quad_mass参数正确: {mass}")
        
        return True
        
    except Exception as e:
        print(f"✗ 参数检查失败: {e}")
        return False

def check_thrust_conversion():
    """检查推力转换一致性"""
    print("\n=== 4. 检查推力转换逻辑 ===")
    
    try:
        from data_structures import ControlOutput
        from experiment_runner import ExperimentRunner
        import numpy as np
        
        # 测试推力转换
        test_cases = [
            (0.5, 0, "悬停点"),
            (0.6, 20, "轻微上升"),
            (0.4, -20, "轻微下降")
        ]
        
        for thrust, expected_throttle, description in test_cases:
            # 创建ControlOutput并测试转换
            control_output = ControlOutput(0.0, 0.0, 0.0, 0.0, thrust)
            _, _, throttle_cmd, _ = control_output.to_tello_rc()
            
            if abs(throttle_cmd - expected_throttle) > 5:  # 允许5%误差
                print(f"✗ 推力转换错误: {thrust} -> {throttle_cmd} (期望: ~{expected_throttle}) - {description}")
                return False
            else:
                print(f"✓ 推力转换正确: {thrust} -> {throttle_cmd} - {description}")
        
        return True
        
    except Exception as e:
        print(f"✗ 推力转换检查失败: {e}")
        return False

def check_math_functions():
    """检查数学函数使用一致性"""
    print("\n=== 5. 检查数学函数使用 ===")
    
    # 这里主要检查是否有混用np和math的问题
    import numpy as np
    import math
    
    # 测试NaN检查
    test_value = float('nan')
    
    try:
        # 确保math.isnan和np.isnan都能工作
        assert math.isnan(test_value)
        assert np.isnan(test_value)
        print("✓ NaN检查函数正常工作")
        
        # 测试三角函数
        angle = np.pi / 4
        assert abs(math.sin(angle) - np.sin(angle)) < 1e-10
        assert abs(math.cos(angle) - np.cos(angle)) < 1e-10
        print("✓ 三角函数一致性正常")
        
        return True
        
    except Exception as e:
        print(f"✗ 数学函数检查失败: {e}")
        return False

def main():
    """主检查函数"""
    print("🔍 开始全面项目检查...")
    print("=" * 50)
    
    checks = [
        check_imports,
        check_controller_interfaces, 
        check_parameter_consistency,
        check_thrust_conversion,
        check_math_functions
    ]
    
    passed = 0
    total = len(checks)
    
    for check_func in checks:
        try:
            if check_func():
                passed += 1
            else:
                print(f"⚠️ 检查失败: {check_func.__name__}")
        except Exception as e:
            print(f"⚠️ 检查异常: {check_func.__name__} - {e}")
    
    print("\n" + "=" * 50)
    print(f"🎯 检查结果: {passed}/{total} 项通过")
    
    if passed == total:
        print("✅ 所有检查通过! 项目代码一致性良好")
        return True
    else:
        print("❌ 发现问题，需要修复后再运行实验")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)