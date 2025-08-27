"""
实验配置文件
在这里修改实验参数，无需改动主实验脚本
"""

# =================实验参数配置=================
# 
# 电池续航优化说明:
# - CYCLE_REST_TIME: 已从5秒减少到1秒，节省电池
# - STABILIZATION_TIME: 已从2秒减少到1秒，减少等待时间
# - 如需更长实验，可考虑减少 EXPERIMENT_CYCLES 或 HOVER_DURATION
#
# =================实验参数配置=================

# 基础实验参数
EXPERIMENT_CYCLES = 3           # 实验周期数量（修改这里改变周期数）
TARGET_HEIGHT = 150             # 目标高度 (cm) - 相对地面绝对高度1.5m
HOVER_DURATION = 1              # 悬停时间 (秒) - 优化电池使用
DESCENT_SPEED = 0.2             # 下降速度 (m/s) - 20cm/s
FINAL_HEIGHT = 30               # 最终高度 (cm) - 相对地面绝对高度0.3m
CYCLE_REST_TIME = 1             # 每周期间隔时间 (秒) - 缩短以节省电池

# 高级参数
STEP_DISTANCE = 20              # 分步下降时每步距离 (cm)
HEIGHT_TOLERANCE = 10           # 高度调整容差 (cm)
STABILIZATION_TIME = 1          # 移动后稳定时间 (秒) - 缩短以节省电池

# 安全参数  
MIN_BATTERY_LEVEL = 25          # 最低电池电量要求 (%)
MAX_EXPERIMENT_TIME = 1800      # 最大实验时间限制 (秒，30分钟)

# 实验元数据
EXPERIMENT_NAME = "altitude_control_experiment"
EXPERIMENT_VERSION = "1.0"
EXPERIMENTER_NAME = ""          # 可选：实验员姓名

# 实验描述（自动生成）
EXPERIMENT_DESCRIPTION = f"""
高度控制实验 - {EXPERIMENT_CYCLES}个周期 (优化版本)
- 目标高度: {TARGET_HEIGHT}cm (相对地面绝对高度{TARGET_HEIGHT/100}m)
- 悬停时间: {HOVER_DURATION}秒 (优化电池使用) 
- 下降速度: {DESCENT_SPEED}m/s ({int(DESCENT_SPEED*100)}cm/s)
- 最终高度: {FINAL_HEIGHT}cm (相对地面绝对高度{FINAL_HEIGHT/100}m)
- 周期间隔: {CYCLE_REST_TIME}秒 (节省电池)
- 稳定等待: {STABILIZATION_TIME}秒 (优化)
"""

# ==============================================

# 参数验证
def validate_parameters():
    """验证实验参数的合理性"""
    errors = []
    
    if EXPERIMENT_CYCLES <= 0:
        errors.append("实验周期数必须大于0")
    
    if TARGET_HEIGHT <= FINAL_HEIGHT:
        errors.append("目标高度必须大于最终高度")
    
    if DESCENT_SPEED <= 0:
        errors.append("下降速度必须大于0")
    
    if HOVER_DURATION < 0:
        errors.append("悬停时间不能为负数")
    
    if TARGET_HEIGHT > 500:  # Tello最大飞行高度约5米
        errors.append("目标高度不应超过500cm以确保安全")
    
    if FINAL_HEIGHT < 5:
        errors.append("最终高度不应低于5cm以确保安全")
    
    return errors

# 获取实验配置摘要
def get_experiment_summary():
    """获取实验配置摘要"""
    return {
        'cycles': EXPERIMENT_CYCLES,
        'target_height_m': TARGET_HEIGHT / 100,
        'final_height_m': FINAL_HEIGHT / 100,
        'hover_duration_s': HOVER_DURATION,
        'descent_speed_ms': DESCENT_SPEED,
        'cycle_rest_time_s': CYCLE_REST_TIME,
        'estimated_cycle_time': (
            (TARGET_HEIGHT - FINAL_HEIGHT) / (DESCENT_SPEED * 100) + 
            HOVER_DURATION + 
            STABILIZATION_TIME * 2
        ),
        'total_estimated_time': (
            (TARGET_HEIGHT - FINAL_HEIGHT) / (DESCENT_SPEED * 100) + 
            HOVER_DURATION + 
            STABILIZATION_TIME * 2
        ) * EXPERIMENT_CYCLES + CYCLE_REST_TIME * (EXPERIMENT_CYCLES - 1)
    }

if __name__ == "__main__":
    # 测试配置参数
    errors = validate_parameters()
    if errors:
        print("⚠️  参数配置错误:")
        for error in errors:
            print(f"  - {error}")
    else:
        print("✅ 参数配置验证通过")
        
        summary = get_experiment_summary()
        print(f"\n📋 实验配置摘要:")
        print(f"  实验周期: {summary['cycles']}")
        print(f"  高度范围: {summary['final_height_m']:.1f}m → {summary['target_height_m']:.1f}m")
        print(f"  下降速度: {summary['descent_speed_ms']:.1f}m/s")
        print(f"  预估单周期时间: {summary['estimated_cycle_time']:.1f}秒")
        print(f"  预估总实验时间: {summary['total_estimated_time']:.1f}秒 ({summary['total_estimated_time']/60:.1f}分钟)")