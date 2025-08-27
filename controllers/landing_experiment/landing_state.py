"""
降落实验状态定义模块

定义与原始C++代码完全相同的状态结构
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional
import time


@dataclass
class DesiredState:
    """期望状态结构体 - 对应C++的Desired_State"""
    pos: np.ndarray        # 位置 [x, y, z] (米)
    vel: np.ndarray        # 速度 [vx, vy, vz] (m/s)
    acc: np.ndarray        # 加速度 [ax, ay, az] (m/s²)
    yaw: float             # 偏航角 (弧度)
    q: Optional[np.ndarray] = None  # 四元数 [w, x, y, z] (可选)
    
    def __post_init__(self):
        """初始化后处理"""
        if isinstance(self.pos, (list, tuple)):
            self.pos = np.array(self.pos, dtype=np.float64)
        if isinstance(self.vel, (list, tuple)):
            self.vel = np.array(self.vel, dtype=np.float64)
        if isinstance(self.acc, (list, tuple)):
            self.acc = np.array(self.acc, dtype=np.float64)


@dataclass
class CurrentState:
    """当前状态结构体 - 对应C++的Current_State"""
    pos: np.ndarray        # 位置 [x, y, z] (米)
    vel: np.ndarray        # 速度 [vx, vy, vz] (m/s)
    acc: np.ndarray        # 加速度 [ax, ay, az] (m/s²)
    yaw: float             # 偏航角 (弧度)
    q: Optional[np.ndarray] = None  # 四元数 [w, x, y, z] (可选)
    
    def __post_init__(self):
        """初始化后处理"""
        if isinstance(self.pos, (list, tuple)):
            self.pos = np.array(self.pos, dtype=np.float64)
        if isinstance(self.vel, (list, tuple)):
            self.vel = np.array(self.vel, dtype=np.float64)
        if isinstance(self.acc, (list, tuple)):
            self.acc = np.array(self.acc, dtype=np.float64)


@dataclass
class ControlOutput:
    """控制输出结构 - 对应C++的u_att"""
    timestamp: float       # 时间戳
    roll: float           # 横滚角指令 (弧度)
    pitch: float          # 俯仰角指令 (弧度) 
    yaw: float            # 偏航角指令 (弧度)
    thrust: float         # 推力指令 (归一化 0-1)
    
    def to_tello_rc(self) -> tuple:
        """转换为Tello RC控制指令"""
        # 转换角度到RC控制范围 [-100, 100]
        roll_rc = int(np.clip(self.roll * 180 / np.pi * 2, -100, 100))
        pitch_rc = int(np.clip(self.pitch * 180 / np.pi * 2, -100, 100))
        yaw_rc = int(np.clip(self.yaw * 180 / np.pi * 2, -100, 100))
        throttle_rc = int(np.clip((self.thrust - 0.5) * 200, -100, 100))
        
        return roll_rc, pitch_rc, throttle_rc, yaw_rc


def quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    """四元数转旋转矩阵"""
    if q is None or len(q) != 4:
        return np.eye(3)
    
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
        [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]
    ])


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """欧拉角转四元数"""
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return np.array([w, x, y, z])


def sign(x: float) -> float:
    """符号函数"""
    return 1.0 if x > 0.0 else (-1.0 if x < 0.0 else 0.0)


def sat(x: float, limit: float) -> float:
    """饱和函数"""
    return max(-limit, min(limit, x))