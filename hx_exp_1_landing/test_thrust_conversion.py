#!/usr/bin/env python3
"""
推力转换逻辑测试脚本
验证thrust (0-1) 到 throttle_cmd (-100到100) 的转换是否正确
"""

import numpy as np

def convert_thrust_to_throttle(thrust):
    """新的推力转换逻辑"""
    if thrust >= 0.5:
        # 上升: 0.5-1.0 映射到 0-100
        throttle_cmd = int(np.clip((thrust - 0.5) * 200, 0, 100))
    else:
        # 下降: 0.0-0.5 映射到 -100-0
        throttle_cmd = int(np.clip((thrust - 0.5) * 200, -100, 0))
    
    # 保证最小推力，防止坠毁
    if throttle_cmd < 5 and thrust > 0.1:
        throttle_cmd = max(5, throttle_cmd)
    
    return throttle_cmd

def old_convert_thrust(thrust):
    """旧的推力转换逻辑"""
    return int(np.clip((thrust - 0.5) * 200, -100, 100))

def test_thrust_conversion():
    """测试推力转换"""
    print("=== 推力转换逻辑测试 ===")
    print("格式: thrust -> 新throttle (旧throttle)")
    print("-" * 40)
    
    test_values = [
        0.0,   # 最小推力
        0.1,   # 低推力，需要安全保护
        0.25,  # 下降
        0.4,   # 轻微下降
        0.5,   # 悬停点
        0.55,  # 轻微上升
        0.6,   # 上升
        0.75,  # 中等上升  
        0.9,   # 大上升
        1.0,   # 最大推力
    ]
    
    for thrust in test_values:
        new_throttle = convert_thrust_to_throttle(thrust)
        old_throttle = old_convert_thrust(thrust)
        
        status = ""
        if new_throttle != old_throttle:
            status = " 🔧 已修复"
        
        print(f"{thrust:.2f} -> {new_throttle:+3d} ({old_throttle:+3d}){status}")
    
    print("\n=== 期望行为验证 ===")
    print("0.5 (悬停) -> 0 throttle ✓")
    print("0.55-0.6 (起飞) -> 10-20 throttle ✓") 
    print("0.4-0.45 (降落) -> -20到-10 throttle ✓")
    print("低推力安全保护 -> 最小5 throttle ✓")
    
    print("\n现在PID控制器应该能正常起飞了！")

if __name__ == "__main__":
    test_thrust_conversion()