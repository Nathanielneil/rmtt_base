"""
数据结构定义

移植自原始C++代码的数据结构，保持完全一致的参数和结构
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple
import time
try:
    from scipy.spatial.transform import Rotation as R
except ImportError:
    # 如果没有scipy，使用简化的四元数处理
    class R:
        @staticmethod
        def from_euler(seq, angles):
            return SimpleRotation(angles)

class SimpleRotation:
    def __init__(self, angles):
        self.angles = angles if hasattr(angles, '__len__') else [0, 0, angles]
    
    def as_matrix(self):
        # 简化的旋转矩阵（仅考虑yaw）
        yaw = self.angles[2] if len(self.angles) > 2 else self.angles
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        return np.array([
            [cos_yaw, -sin_yaw, 0],
            [sin_yaw, cos_yaw, 0],
            [0, 0, 1]
        ])


@dataclass
class DesiredState:
    """期望状态结构体 - 完全对应原始C++结构"""
    pos: np.ndarray       # 3D位置 [x, y, z] 
    vel: np.ndarray       # 3D速度 [vx, vy, vz]
    acc: np.ndarray       # 3D加速度 [ax, ay, az]
    yaw: float            # 偏航角 (弧度)
    q: R                  # 四元数姿态

    def __init__(self, pos=None, vel=None, acc=None, yaw=0.0, q=None):
        self.pos = np.array(pos) if pos is not None else np.zeros(3)
        self.vel = np.array(vel) if vel is not None else np.zeros(3)  
        self.acc = np.array(acc) if acc is not None else np.zeros(3)
        self.yaw = yaw
        self.q = q if q is not None else R.from_euler('z', yaw)


@dataclass  
class CurrentState:
    """当前状态结构体 - 完全对应原始C++结构"""
    pos: np.ndarray       # 当前3D位置
    vel: np.ndarray       # 当前3D速度
    acc: np.ndarray       # 当前3D加速度 
    yaw: float            # 当前偏航角
    q: R                  # 当前四元数姿态

    def __init__(self, pos=None, vel=None, acc=None, yaw=0.0, q=None):
        self.pos = np.array(pos) if pos is not None else np.zeros(3)
        self.vel = np.array(vel) if vel is not None else np.zeros(3)
        self.acc = np.array(acc) if acc is not None else np.zeros(3)
        self.yaw = yaw
        self.q = q if q is not None else R.from_euler('z', yaw)


class ControlOutput:
    """控制输出 - 对应原始C++的Vector4d输出"""
    def __init__(self, roll=0.0, pitch=0.0, yaw=0.0, thrust=0.0):
        self.roll = roll      # 横滚角 (弧度)
        self.pitch = pitch    # 俯仰角 (弧度) 
        self.yaw = yaw        # 偏航角 (弧度)
        self.thrust = thrust  # 推力 (0-1)
        self.timestamp = time.time()
    
    def as_array(self) -> np.ndarray:
        """返回为numpy数组，对应Eigen::Vector4d"""
        return np.array([self.roll, self.pitch, self.yaw, self.thrust])
    
    def as_rc_command(self) -> Tuple[int, int, int, int]:
        """转换为Tello RC控制指令 [roll, pitch, throttle, yaw]"""
        # 转换为度并缩放到Tello范围(-100到100)
        roll_cmd = int(np.clip(self.roll * 180 / np.pi * 2.5, -100, 100))
        pitch_cmd = int(np.clip(self.pitch * 180 / np.pi * 2.5, -100, 100))
        yaw_cmd = int(np.clip(self.yaw * 180 / np.pi * 2.5, -100, 100))
        
        # 推力转换为throttle (-100到100)
        throttle_cmd = int(np.clip((self.thrust - 0.5) * 200, -100, 100))
        
        return roll_cmd, pitch_cmd, throttle_cmd, yaw_cmd


def sign(x: float) -> float:
    """符号函数 - 对应原始C++实现"""
    return 1.0 if x > 0.0 else (-1.0 if x < 0.0 else 0.0)


def sat(x: float, limit: float) -> float:
    """饱和函数 - 对应原始C++实现"""
    return limit if x > limit else (-limit if x < -limit else x)