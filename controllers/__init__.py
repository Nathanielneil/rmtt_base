"""
RMTT自定义控制器模块

这个模块提供了一个灵活的框架，用于实现自定义的无人机控制器，
替代或增强RMTT自带的PID控制系统。

主要特性：
- 模块化控制器架构
- 分层控制结构（姿态控制 + 位置控制）
- 支持位置、速度、姿态等多种控制模式
- 实时传感器数据融合
- 可配置的控制参数
- 安全监控和故障处理

模块结构：
├── base_controller.py          # 基础控制器抽象类
├── controller_manager.py       # 控制器管理器
├── attitude_control/           # 姿态控制模块
│   └── attitude_controller.py  # 姿态角控制器
└── position_control/           # 位置控制模块
    ├── position_controller.py  # 位置控制器
    ├── velocity_controller.py  # 速度控制器
    └── trajectory_controller.py # 轨迹控制器
"""

from .base_controller import BaseController, ControllerState, ControlCommand, SensorData
from .controller_manager import ControllerManager

# 导入姿态控制模块
from .attitude_control import AttitudeController

# 导入位置控制模块  
from .position_control import (
    PositionController, PIDController, PIDGains,
    VelocityController,
    TrajectoryController, TrajectoryType, Waypoint, TrajectoryPoint
)

__all__ = [
    # 基础组件
    'BaseController', 'ControllerState', 'ControlCommand', 'SensorData',
    'ControllerManager',
    
    # 姿态控制
    'AttitudeController',
    
    # 位置控制
    'PositionController', 'PIDController', 'PIDGains',
    'VelocityController', 
    'TrajectoryController', 'TrajectoryType', 'Waypoint', 'TrajectoryPoint'
]

__version__ = "1.0.0"