"""
姿态控制模块

包含用于直接控制无人机姿态角的控制器：
- 横滚角(Roll)控制
- 俯仰角(Pitch)控制  
- 偏航角(Yaw)控制
- 角速度控制
- 姿态稳定控制

这些控制器提供低层次的姿态控制能力，是位置控制的基础。
"""

from .attitude_controller import AttitudeController

__all__ = [
    'AttitudeController'
]