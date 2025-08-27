# RMTT自定义控制器系统

## 概述

RMTT自定义控制器系统提供了一个强大而灵活的框架，用于实现各种无人机控制算法，完全替代或增强传统的PID控制系统。

## 系统架构

```
controllers/
├── base_controller.py              # 基础控制器抽象类
├── controller_manager.py           # 控制器管理器
├── attitude_control/               # 姿态控制模块
│   ├── __init__.py
│   └── attitude_controller.py      # 姿态角控制器
├── position_control/               # 位置控制模块
│   ├── __init__.py
│   ├── position_controller.py      # 3D位置控制器
│   ├── velocity_controller.py      # 速度控制器
│   └── trajectory_controller.py    # 轨迹跟踪控制器
├── example_usage.py                # 使用示例
├── quick_start.py                  # 快速启动示例
└── README.md                       # 本文档
```

## 核心特性

### 分层控制架构
- **姿态控制层**: 直接控制Roll、Pitch、Yaw角度
- **位置控制层**: 基于姿态控制实现3D位置控制
- **轨迹控制层**: 基于位置控制实现复杂轨迹跟踪

### 模块化设计
- 每个控制器都继承自`BaseController`抽象类
- 统一的接口和生命周期管理
- 可插拔的控制算法

### 安全监控
- 多层安全检查机制
- 自动故障检测和恢复
- 紧急停止和降落功能

### 实时数据融合
- 高频传感器数据处理（50Hz）
- RMTT双重定高传感器融合
- 实时性能监控和统计

## 控制器类型

### 1. 姿态控制器 (AttitudeController)

**功能**: 精确控制无人机的姿态角度
**应用场景**: 姿态稳定、特殊飞行动作、视觉伺服

```python
from controllers import AttitudeController

# 创建姿态控制器
attitude_controller = AttitudeController(
    config={
        'roll_gains': {'kp': 1.5, 'ki': 0.1, 'kd': 0.2},
        'pitch_gains': {'kp': 1.5, 'ki': 0.1, 'kd': 0.2},
        'yaw_gains': {'kp': 2.0, 'ki': 0.15, 'kd': 0.3},
        'altitude_hold_enabled': True
    }
)

# 设置目标姿态
attitude_controller.set_target({
    'roll_deg': 10.0,    # 右倾10度
    'pitch_deg': -5.0,   # 前倾5度
    'yaw_deg': 45.0      # 右转45度
})
```

### 2. 位置控制器 (PositionController)

**功能**: 控制无人机在3D空间中的位置
**应用场景**: 定点悬停、位置跟踪、自动降落

```python
from controllers import PositionController

# 创建位置控制器
position_controller = PositionController(
    config={
        'altitude_gains': {'kp': 0.8, 'ki': 0.1, 'kd': 0.3},
        'position_gains': {'kp': 0.6, 'ki': 0.05, 'kd': 0.25}
    }
)

# 设置目标位置
position_controller.set_target({
    'height_cm': 100.0,  # 高度100cm
    'x_cm': 50.0,        # 前进50cm
    'y_cm': 30.0,        # 右移30cm
    'yaw_deg': 0.0       # 保持朝向
})
```

### 3. 速度控制器 (VelocityController)

**功能**: 直接控制无人机的线速度和角速度
**应用场景**: 动态跟踪、速度控制、平滑运动

```python
from controllers import VelocityController

# 创建速度控制器
velocity_controller = VelocityController(
    config={
        'max_vertical_speed': 50.0,
        'max_horizontal_speed': 80.0,
        'velocity_filter_alpha': 0.7
    }
)

# 设置目标速度
velocity_controller.set_target({
    'vx_cm_s': 30.0,     # 前进速度30cm/s
    'vy_cm_s': 0.0,      # 左右速度0
    'vz_cm_s': 10.0,     # 上升速度10cm/s
    'vyaw_deg_s': 0.0    # 不旋转
})
```

### 4. 轨迹控制器 (TrajectoryController)

**功能**: 跟踪预定义的飞行轨迹
**应用场景**: 航点导航、几何轨迹、自动巡检

```python
from controllers import TrajectoryController, TrajectoryType

# 创建轨迹控制器
trajectory_controller = TrajectoryController(
    config={
        'max_speed_cm_s': 50.0,
        'feedforward_gain': 0.8,
        'tracking_tolerance_cm': 8.0
    }
)

# 圆形轨迹
trajectory_controller.set_target({
    'trajectory_type': TrajectoryType.CIRCULAR.value,
    'parameters': {
        'radius_cm': 100.0,
        'height_cm': 120.0,
        'speed_cm_s': 40.0,
        'num_laps': 2.0
    }
})

# 航点轨迹
waypoints = [
    {'x_cm': 0, 'y_cm': 0, 'height_cm': 80, 'yaw_deg': 0, 'speed_cm_s': 30, 'hold_time_s': 2},
    {'x_cm': 100, 'y_cm': 0, 'height_cm': 100, 'yaw_deg': 90, 'speed_cm_s': 25, 'hold_time_s': 3},
    {'x_cm': 100, 'y_cm': 100, 'height_cm': 120, 'yaw_deg': 180, 'speed_cm_s': 20, 'hold_time_s': 2}
]

trajectory_controller.set_target({
    'trajectory_type': TrajectoryType.WAYPOINT.value,
    'waypoints': waypoints
})
```

## 控制器管理器 (ControllerManager)

控制器管理器是整个系统的核心，负责统一管理所有控制器。

### 基本使用

```python
from controllers import ControllerManager, ControllerManagerConfig, ControlMode

# 创建管理器
config = ControllerManagerConfig(
    update_frequency_hz=50.0,
    safety_check_enabled=True,
    auto_fallback_enabled=True
)
manager = ControllerManager(config)

# 注册控制器
manager.register_controller("attitude", attitude_controller)
manager.register_controller("position", position_controller)
manager.register_controller("velocity", velocity_controller)

# 启动控制循环
manager.start_control_loop()

# 切换控制模式
manager.switch_control_mode(ControlMode.POSITION)

# 更新传感器数据
sensor_data = SensorData(...)  # 从RMTT获取
manager.update_sensor_data(sensor_data)

# 获取控制指令
control_command = manager.get_control_command()
```

### 控制模式

- `MANUAL`: 手动控制模式
- `ATTITUDE`: 姿态控制模式
- `POSITION`: 位置控制模式  
- `VELOCITY`: 速度控制模式
- `TRAJECTORY`: 轨迹控制模式
- `EMERGENCY`: 紧急模式

### 安全特性

- **电池监控**: 电量低于10%自动紧急降落
- **连接监控**: WiFi断开时自动切换到紧急模式
- **姿态监控**: 异常姿态角自动触发安全措施
- **控制器故障**: 自动回退到备用控制器
- **超时保护**: 控制器响应超时自动处理

## 快速开始

### 1. 基础使用示例

```python
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent.parent))

from core.connection import ConnectionManager
from controllers import (
    ControllerManager, ControlMode, 
    PositionController, SensorData
)

# 初始化
connection_manager = ConnectionManager()
connection_manager.connect()

controller_manager = ControllerManager()

# 创建位置控制器
position_controller = PositionController()
controller_manager.register_controller("position", position_controller)

# 启动系统
controller_manager.start_control_loop()
controller_manager.switch_control_mode(ControlMode.POSITION)

# 起飞并悬停
tello = connection_manager.get_tello()
tello.takeoff()

position_controller.set_target({
    'height_cm': 100.0,
    'x_cm': 0.0,
    'y_cm': 0.0, 
    'yaw_deg': 0.0
})

# 主循环
while True:
    # 获取传感器数据
    sensor_data = SensorData(
        timestamp=time.time(),
        height_cm=tello.get_height(),
        tof_distance_cm=tello.get_distance_tof(),
        # ... 其他传感器数据
    )
    
    # 更新控制器
    controller_manager.update_sensor_data(sensor_data)
    
    # 获取并发送控制指令
    command = controller_manager.get_control_command()
    if command:
        tello.send_rc_control(
            int(command.roll), int(command.pitch), 
            int(command.throttle), int(command.yaw)
        )
    
    time.sleep(0.02)  # 50Hz控制频率
```

### 2. 运行完整演示

```bash
cd /home/daniel/NGW/github_projects/rmtt
python controllers/example_usage.py
```

演示程序将展示：
- 姿态控制演示
- 位置控制演示  
- 速度控制演示
- 轨迹控制演示
- 航点导航演示

## 高级功能

### 自定义控制器

创建自定义控制器只需继承`BaseController`类：

```python
from controllers.base_controller import BaseController, SensorData, ControlCommand

class MyCustomController(BaseController):
    def __init__(self, name="custom", config=None):
        super().__init__(name, config)
        # 初始化自定义参数
        
    def set_target(self, target):
        # 设置控制目标
        self.target_setpoint = target
        return True
        
    def compute_control(self, sensor_data, target):
        # 实现控制算法
        # 返回ControlCommand对象
        return ControlCommand(
            timestamp=time.time(),
            roll=0.0, pitch=0.0, 
            throttle=0.0, yaw=0.0
        )
```

### PID参数调优

所有控制器都支持在线PID参数调整：

```python
# 运行时调整PID参数
controller.altitude_pid.gains.kp = 1.2
controller.altitude_pid.gains.ki = 0.15
controller.altitude_pid.gains.kd = 0.25

# 重置PID内部状态
controller.altitude_pid.reset()
```

### 数据记录与分析

系统自动记录所有控制数据：

```python
# 获取控制器性能统计
status = controller.get_status()
performance = status['performance']

print(f"平均控制时间: {performance['avg_control_time_ms']:.2f}ms")
print(f"最大控制时间: {performance['max_control_time_ms']:.2f}ms")
print(f"总控制周期: {performance['total_iterations']}")

# 获取控制误差
if hasattr(controller, 'get_control_errors'):
    errors = controller.get_control_errors()
    print(f"高度误差: {errors.get('height_error_cm', 0):.1f}cm")
```

## 故障排除

### 常见问题

1. **控制器无响应**
   - 检查控制器是否已激活：`controller.state == ControllerState.ACTIVE`
   - 检查是否设置了目标：`controller.target_setpoint is not None`
   - 检查传感器数据是否更新：`controller.current_sensor_data is not None`

2. **控制振荡**
   - 降低PID增益，特别是比例增益Kp
   - 增加微分增益Kd以提高阻尼
   - 检查传感器数据质量和噪声

3. **控制精度不足**
   - 增加比例增益Kp
   - 适当增加积分增益Ki消除静态误差
   - 检查控制频率是否足够高

4. **系统崩溃**
   - 检查日志文件了解详细错误信息
   - 验证传感器数据的有效性
   - 确保安全检查机制正常工作

### 调试技巧

```python
# 启用详细日志
import logging
logging.getLogger("controllers").setLevel(logging.DEBUG)

# 监控控制器状态
def monitor_controller(controller):
    status = controller.get_status()
    print(f"状态: {status['state']}")
    print(f"目标: {status['target_setpoint']}")
    if hasattr(controller, 'get_control_errors'):
        errors = controller.get_control_errors()
        print(f"误差: {errors}")

# 性能分析
def analyze_performance(manager):
    status = manager.get_status()
    perf = status['performance']
    print(f"控制频率: {1000/perf['avg_cycle_time_ms']:.1f} Hz")
    print(f"错误次数: {status['error_count']}")
```

## 注意事项

### 安全须知

1. **首次使用**：建议在空旷安全区域进行测试
2. **参数调整**：小幅度调整PID参数，避免剧烈振荡
3. **电池监控**：始终关注电池电量，低电量时及时降落
4. **紧急停止**：熟悉紧急停止操作，遇到异常立即停止
5. **备份控制**：保留手动遥控器作为备用控制手段

### 性能优化

1. **控制频率**：推荐50Hz控制频率，平衡性能和计算负载
2. **传感器融合**：充分利用RMTT的TOF和气压计双重定高优势
3. **预测控制**：在轨迹控制中使用前瞻算法提高跟踪精度
4. **滤波处理**：对传感器数据进行适当滤波减少噪声影响

## 更新日志

### v1.0.0 (2024-12-xx)
- 初始版本发布
- 实现基础控制器架构
- 支持姿态、位置、速度、轨迹四种控制模式
- 集成安全监控和故障处理
- 提供完整使用示例和文档

---

## 技术支持

如有问题或建议，请：
1. 查看日志文件获取详细错误信息
2. 参考示例代码和文档
3. 在GitHub项目页面提交Issue

**享受自定义控制器带来的强大功能！**