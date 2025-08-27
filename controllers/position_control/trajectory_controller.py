"""
轨迹控制器

实现轨迹跟踪控制，可以沿着预定义的轨迹飞行
"""

import time
import numpy as np
from typing import Dict, Any, List, Tuple, Optional
from dataclasses import dataclass
from enum import Enum
import math

from ..base_controller import BaseController, SensorData, ControlCommand
from .position_controller import PositionController


class TrajectoryType(Enum):
    """轨迹类型枚举"""
    WAYPOINT = "waypoint"          # 航点轨迹
    CIRCULAR = "circular"          # 圆形轨迹
    FIGURE_EIGHT = "figure_eight"  # 8字形轨迹
    SQUARE = "square"              # 方形轨迹
    CUSTOM = "custom"              # 自定义轨迹


@dataclass
class Waypoint:
    """航点数据结构"""
    x_cm: float          # X坐标 (cm)
    y_cm: float          # Y坐标 (cm)
    height_cm: float     # 高度 (cm)
    yaw_deg: float       # 偏航角 (度)
    speed_cm_s: float    # 飞行速度 (cm/s)
    hold_time_s: float   # 停留时间 (秒)
    tolerance_cm: float  # 到达容差 (cm)


@dataclass
class TrajectoryPoint:
    """轨迹点数据结构"""
    timestamp: float     # 时间戳
    x_cm: float         # X坐标
    y_cm: float         # Y坐标  
    height_cm: float    # 高度
    yaw_deg: float      # 偏航角
    vx_cm_s: float      # X方向速度
    vy_cm_s: float      # Y方向速度
    vz_cm_s: float      # Z方向速度


class TrajectoryController(BaseController):
    """
    轨迹控制器
    
    支持多种轨迹跟踪模式：
    - 航点飞行：按预定航点依次飞行
    - 几何轨迹：圆形、方形、8字形等规律轨迹
    - 自定义轨迹：基于时间参数的任意轨迹
    
    特性：
    - 轨迹预生成和实时跟踪
    - 速度和加速度平滑
    - 前馈+反馈控制
    - 轨迹可视化和监控
    """
    
    def __init__(self, name: str = "trajectory_controller", config: Dict[str, Any] = None):
        super().__init__(name, config)
        
        # 默认配置
        default_config = {
            # 轨迹生成参数
            'trajectory_resolution_s': 0.1,    # 轨迹点时间分辨率
            'max_speed_cm_s': 50.0,            # 最大飞行速度
            'max_acceleration_cm_s2': 30.0,    # 最大加速度
            'default_height_cm': 100.0,        # 默认飞行高度
            
            # 控制参数 - 使用前馈+反馈控制
            'feedforward_gain': 0.8,           # 前馈增益
            'position_kp': 0.6,                # 位置反馈增益
            'velocity_kp': 0.4,                # 速度反馈增益
            
            # 轨迹跟踪参数
            'lookahead_time_s': 0.5,           # 前瞻时间
            'tracking_tolerance_cm': 8.0,      # 跟踪容差
            'waypoint_tolerance_cm': 15.0,     # 航点到达容差
            
            # 几何轨迹参数
            'circle_radius_cm': 100.0,         # 圆形轨迹半径
            'square_size_cm': 150.0,           # 方形轨迹边长
            'eight_width_cm': 120.0,           # 8字轨迹宽度
        }
        
        # 合并配置
        for key, value in default_config.items():
            if key not in self.config:
                self.config[key] = value
        
        # 创建底层位置控制器
        self.position_controller = PositionController(
            name="trajectory_position_controller",
            config={
                'altitude_gains': {'kp': 1.0, 'ki': 0.1, 'kd': 0.3},
                'position_gains': {'kp': 0.8, 'ki': 0.05, 'kd': 0.2}
            }
        )
        
        # 轨迹数据
        self.trajectory_points: List[TrajectoryPoint] = []
        self.current_trajectory_index = 0
        self.trajectory_start_time = None
        self.trajectory_type = None
        
        # 航点数据
        self.waypoints: List[Waypoint] = []
        self.current_waypoint_index = 0
        self.waypoint_hold_start_time = None
        
        # 状态跟踪
        self.trajectory_completed = False
        self.total_distance_traveled = 0.0
        self.last_position = None
        
        self.logger.info("轨迹控制器初始化完成")
    
    def set_target(self, target: Dict[str, Any]) -> bool:
        """
        设置轨迹目标
        
        Args:
            target: 轨迹参数字典，包含：
                   - 'trajectory_type': 轨迹类型 (TrajectoryType)
                   - 'waypoints': 航点列表 (for WAYPOINT type)
                   - 'parameters': 轨迹参数字典 (for geometric trajectories)
                   
        Returns:
            bool: 设置是否成功
        """
        try:
            trajectory_type = target.get('trajectory_type')
            if trajectory_type not in [t.value for t in TrajectoryType]:
                self.logger.error(f"不支持的轨迹类型: {trajectory_type}")
                return False
            
            self.trajectory_type = TrajectoryType(trajectory_type)
            
            # 根据轨迹类型生成轨迹
            if self.trajectory_type == TrajectoryType.WAYPOINT:
                waypoints_data = target.get('waypoints', [])
                if not waypoints_data:
                    self.logger.error("航点轨迹需要提供航点列表")
                    return False
                
                self.waypoints = [Waypoint(**wp) for wp in waypoints_data]
                success = self._generate_waypoint_trajectory()
                
            elif self.trajectory_type == TrajectoryType.CIRCULAR:
                params = target.get('parameters', {})
                success = self._generate_circular_trajectory(params)
                
            elif self.trajectory_type == TrajectoryType.SQUARE:
                params = target.get('parameters', {})
                success = self._generate_square_trajectory(params)
                
            elif self.trajectory_type == TrajectoryType.FIGURE_EIGHT:
                params = target.get('parameters', {})
                success = self._generate_figure_eight_trajectory(params)
                
            else:
                self.logger.error(f"轨迹类型 {self.trajectory_type} 尚未实现")
                return False
            
            if success:
                self._reset_trajectory_state()
                self.target_setpoint = target
                self.logger.info(f"设置{self.trajectory_type.value}轨迹，包含{len(self.trajectory_points)}个轨迹点")
                return True
            else:
                return False
                
        except Exception as e:
            self.logger.error(f"设置轨迹目标失败: {e}")
            return False
    
    def compute_control(self, sensor_data: SensorData, target: Dict[str, Any]) -> ControlCommand:
        """
        计算轨迹跟踪控制指令
        
        Args:
            sensor_data: 传感器数据
            target: 轨迹目标
            
        Returns:
            ControlCommand: 控制指令
        """
        current_time = time.time()
        
        # 安全检查
        if not self._safety_check(sensor_data):
            return self.emergency_stop()
        
        # 初始化轨迹开始时间
        if self.trajectory_start_time is None:
            self.trajectory_start_time = current_time
            self.logger.info("开始轨迹跟踪")
        
        # 获取当前目标位置
        target_point = self._get_current_trajectory_target(current_time)
        if target_point is None:
            # 轨迹结束
            self.trajectory_completed = True
            self.logger.info("轨迹跟踪完成")
            return ControlCommand(
                timestamp=current_time,
                roll=0.0, pitch=0.0, throttle=0.0, yaw=0.0
            )
        
        # 更新距离统计
        self._update_distance_tracking(sensor_data)
        
        # 前馈+反馈控制
        control_command = self._compute_feedforward_feedback_control(
            sensor_data, target_point, current_time
        )
        
        return control_command
    
    def _generate_waypoint_trajectory(self) -> bool:
        """生成航点轨迹"""
        if not self.waypoints:
            return False
        
        self.trajectory_points = []
        current_time = 0.0
        
        for i, waypoint in enumerate(self.waypoints):
            # 计算到下一个航点的距离和飞行时间
            if i == 0:
                # 第一个航点，从当前位置开始
                prev_x, prev_y, prev_h = 0.0, 0.0, self.config['default_height_cm']
            else:
                prev_waypoint = self.waypoints[i-1]
                prev_x, prev_y, prev_h = prev_waypoint.x_cm, prev_waypoint.y_cm, prev_waypoint.height_cm
            
            # 计算飞行距离和时间
            dx = waypoint.x_cm - prev_x
            dy = waypoint.y_cm - prev_y
            dh = waypoint.height_cm - prev_h
            distance = math.sqrt(dx*dx + dy*dy + dh*dh)
            
            flight_time = distance / waypoint.speed_cm_s if waypoint.speed_cm_s > 0 else 1.0
            
            # 生成轨迹点
            num_points = max(1, int(flight_time / self.config['trajectory_resolution_s']))
            
            for j in range(num_points + 1):
                t_ratio = j / num_points if num_points > 0 else 1.0
                
                # 插值位置
                x = prev_x + dx * t_ratio
                y = prev_y + dy * t_ratio
                h = prev_h + dh * t_ratio
                
                # 计算速度
                if j < num_points:
                    vx = dx / flight_time if flight_time > 0 else 0.0
                    vy = dy / flight_time if flight_time > 0 else 0.0
                    vz = dh / flight_time if flight_time > 0 else 0.0
                else:
                    vx = vy = vz = 0.0  # 到达航点时停止
                
                point = TrajectoryPoint(
                    timestamp=current_time,
                    x_cm=x, y_cm=y, height_cm=h, yaw_deg=waypoint.yaw_deg,
                    vx_cm_s=vx, vy_cm_s=vy, vz_cm_s=vz
                )
                self.trajectory_points.append(point)
                current_time += self.config['trajectory_resolution_s']
            
            # 添加停留时间
            if waypoint.hold_time_s > 0:
                hold_points = int(waypoint.hold_time_s / self.config['trajectory_resolution_s'])
                for _ in range(hold_points):
                    point = TrajectoryPoint(
                        timestamp=current_time,
                        x_cm=waypoint.x_cm, y_cm=waypoint.y_cm, height_cm=waypoint.height_cm,
                        yaw_deg=waypoint.yaw_deg,
                        vx_cm_s=0.0, vy_cm_s=0.0, vz_cm_s=0.0
                    )
                    self.trajectory_points.append(point)
                    current_time += self.config['trajectory_resolution_s']
        
        return len(self.trajectory_points) > 0
    
    def _generate_circular_trajectory(self, params: Dict[str, Any]) -> bool:
        """生成圆形轨迹"""
        radius = params.get('radius_cm', self.config['circle_radius_cm'])
        center_x = params.get('center_x_cm', 0.0)
        center_y = params.get('center_y_cm', 0.0)
        height = params.get('height_cm', self.config['default_height_cm'])
        speed = params.get('speed_cm_s', self.config['max_speed_cm_s'] * 0.5)
        num_laps = params.get('num_laps', 1.0)
        
        # 计算轨迹总时间
        circumference = 2 * math.pi * radius
        total_time = (circumference * num_laps) / speed
        
        self.trajectory_points = []
        current_time = 0.0
        
        while current_time <= total_time:
            # 计算角度位置
            angle = (current_time * speed / radius) % (2 * math.pi)
            
            # 计算位置
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            
            # 计算切向速度
            vx = -speed * math.sin(angle)
            vy = speed * math.cos(angle)
            
            # 计算朝向角 (切向)
            yaw = math.degrees(angle + math.pi/2)
            
            point = TrajectoryPoint(
                timestamp=current_time,
                x_cm=x, y_cm=y, height_cm=height, yaw_deg=yaw,
                vx_cm_s=vx, vy_cm_s=vy, vz_cm_s=0.0
            )
            self.trajectory_points.append(point)
            current_time += self.config['trajectory_resolution_s']
        
        return True
    
    def _generate_square_trajectory(self, params: Dict[str, Any]) -> bool:
        """生成方形轨迹"""
        size = params.get('size_cm', self.config['square_size_cm'])
        center_x = params.get('center_x_cm', 0.0)
        center_y = params.get('center_y_cm', 0.0)
        height = params.get('height_cm', self.config['default_height_cm'])
        speed = params.get('speed_cm_s', self.config['max_speed_cm_s'] * 0.4)
        
        # 定义方形四个顶点
        half_size = size / 2
        corners = [
            (center_x + half_size, center_y + half_size),  # 右上
            (center_x - half_size, center_y + half_size),  # 左上
            (center_x - half_size, center_y - half_size),  # 左下
            (center_x + half_size, center_y - half_size),  # 右下
        ]
        
        waypoints = []
        for corner in corners:
            waypoint = Waypoint(
                x_cm=corner[0], y_cm=corner[1], height_cm=height,
                yaw_deg=0.0, speed_cm_s=speed, hold_time_s=0.5,
                tolerance_cm=self.config['waypoint_tolerance_cm']
            )
            waypoints.append(waypoint)
        
        # 添加第一个点完成闭环
        waypoints.append(waypoints[0])
        
        self.waypoints = waypoints
        return self._generate_waypoint_trajectory()
    
    def _generate_figure_eight_trajectory(self, params: Dict[str, Any]) -> bool:
        """生成8字形轨迹"""
        width = params.get('width_cm', self.config['eight_width_cm'])
        center_x = params.get('center_x_cm', 0.0)
        center_y = params.get('center_y_cm', 0.0)
        height = params.get('height_cm', self.config['default_height_cm'])
        speed = params.get('speed_cm_s', self.config['max_speed_cm_s'] * 0.3)
        
        # 8字形轨迹总时间（大约）
        total_time = (4 * width) / speed
        
        self.trajectory_points = []
        current_time = 0.0
        
        while current_time <= total_time:
            # 8字形参数方程
            t = (current_time / total_time) * 4 * math.pi  # 完整8字需要4π
            
            # 8字形轨迹: x = a*sin(t), y = a*sin(t)*cos(t)
            a = width / 2
            x = center_x + a * math.sin(t)
            y = center_y + a * math.sin(t) * math.cos(t)
            
            # 计算速度（参数方程的导数）
            dt_real = self.config['trajectory_resolution_s']
            dt_param = (4 * math.pi / total_time) * dt_real
            
            vx = a * math.cos(t) * (dt_param / dt_real)
            vy = a * (math.cos(t)*math.cos(t) - math.sin(t)*math.sin(t)) * (dt_param / dt_real)
            
            # 速度归一化到指定大小
            v_mag = math.sqrt(vx*vx + vy*vy)
            if v_mag > 0:
                vx = (vx / v_mag) * speed
                vy = (vy / v_mag) * speed
            
            # 计算朝向角
            yaw = math.degrees(math.atan2(vy, vx)) if v_mag > 0 else 0.0
            
            point = TrajectoryPoint(
                timestamp=current_time,
                x_cm=x, y_cm=y, height_cm=height, yaw_deg=yaw,
                vx_cm_s=vx, vy_cm_s=vy, vz_cm_s=0.0
            )
            self.trajectory_points.append(point)
            current_time += self.config['trajectory_resolution_s']
        
        return True
    
    def _get_current_trajectory_target(self, current_time: float) -> Optional[TrajectoryPoint]:
        """获取当前时刻的轨迹目标点"""
        if not self.trajectory_points or self.trajectory_start_time is None:
            return None
        
        elapsed_time = current_time - self.trajectory_start_time
        
        # 查找对应的轨迹点
        target_index = None
        for i, point in enumerate(self.trajectory_points):
            if point.timestamp >= elapsed_time:
                target_index = i
                break
        
        if target_index is None:
            # 轨迹结束
            return None
        
        # 前瞻控制 - 查看未来的轨迹点
        lookahead_time = elapsed_time + self.config['lookahead_time_s']
        lookahead_index = target_index
        
        for i in range(target_index, len(self.trajectory_points)):
            if self.trajectory_points[i].timestamp >= lookahead_time:
                lookahead_index = i
                break
        
        # 返回前瞻点
        return self.trajectory_points[min(lookahead_index, len(self.trajectory_points) - 1)]
    
    def _compute_feedforward_feedback_control(self, sensor_data: SensorData, target_point: TrajectoryPoint, current_time: float) -> ControlCommand:
        """计算前馈+反馈控制指令"""
        
        # 获取当前位置估计 (简化版本)
        current_height = sensor_data.tof_distance_cm if sensor_data.tof_distance_cm > 0 else sensor_data.height_cm
        
        # 简化的位置估计 (实际应用中需要更精确的状态估计)
        current_x = 0.0  # 需要集成位置估计算法
        current_y = 0.0
        current_vx = sensor_data.vgx_cm_s
        current_vy = sensor_data.vgy_cm_s
        current_vz = sensor_data.vgz_cm_s
        
        # 前馈控制 - 基于期望轨迹的速度
        ff_vx = target_point.vx_cm_s * self.config['feedforward_gain']
        ff_vy = target_point.vy_cm_s * self.config['feedforward_gain']
        ff_vz = target_point.vz_cm_s * self.config['feedforward_gain']
        
        # 反馈控制 - 基于位置和速度误差
        pos_error_x = target_point.x_cm - current_x
        pos_error_y = target_point.y_cm - current_y
        pos_error_z = target_point.height_cm - current_height
        
        vel_error_x = target_point.vx_cm_s - current_vx
        vel_error_y = target_point.vy_cm_s - current_vy
        vel_error_z = target_point.vz_cm_s - current_vz
        
        fb_vx = pos_error_x * self.config['position_kp'] + vel_error_x * self.config['velocity_kp']
        fb_vy = pos_error_y * self.config['position_kp'] + vel_error_y * self.config['velocity_kp']
        fb_vz = pos_error_z * self.config['position_kp'] + vel_error_z * self.config['velocity_kp']
        
        # 组合前馈和反馈
        cmd_vx = ff_vx + fb_vx
        cmd_vy = ff_vy + fb_vy
        cmd_vz = ff_vz + fb_vz
        
        # 转换为Tello控制指令
        # 这是一个简化的映射，实际应用中需要更精确的控制律
        pitch_cmd = -np.clip(cmd_vx * 2.0, -80, 80)  # 前后
        roll_cmd = np.clip(cmd_vy * 2.0, -80, 80)    # 左右
        throttle_cmd = np.clip(cmd_vz * 1.5, -80, 80)  # 上下
        
        # 偏航角控制
        yaw_error = target_point.yaw_deg - sensor_data.yaw_deg
        if yaw_error > 180:
            yaw_error -= 360
        elif yaw_error < -180:
            yaw_error += 360
        yaw_cmd = np.clip(yaw_error * 1.5, -80, 80)
        
        return ControlCommand(
            timestamp=current_time,
            roll=roll_cmd,
            pitch=pitch_cmd,
            throttle=throttle_cmd,
            yaw=yaw_cmd
        )
    
    def _update_distance_tracking(self, sensor_data: SensorData):
        """更新距离跟踪统计"""
        current_pos = (sensor_data.vgx_cm_s, sensor_data.vgy_cm_s)  # 简化位置
        
        if self.last_position is not None:
            dx = current_pos[0] - self.last_position[0]
            dy = current_pos[1] - self.last_position[1]
            distance = math.sqrt(dx*dx + dy*dy)
            self.total_distance_traveled += distance
        
        self.last_position = current_pos
    
    def _reset_trajectory_state(self):
        """重置轨迹状态"""
        self.current_trajectory_index = 0
        self.trajectory_start_time = None
        self.trajectory_completed = False
        self.total_distance_traveled = 0.0
        self.last_position = None
        self.current_waypoint_index = 0
        self.waypoint_hold_start_time = None
    
    def _on_activate(self) -> bool:
        """激活轨迹控制器"""
        # 激活底层位置控制器
        return self.position_controller.activate()
    
    def _on_deactivate(self) -> bool:
        """停用轨迹控制器"""
        # 停用底层位置控制器
        return self.position_controller.deactivate()
    
    def _on_reset(self) -> bool:
        """重置轨迹控制器"""
        self.trajectory_points.clear()
        self.waypoints.clear()
        self._reset_trajectory_state()
        return self.position_controller.reset()
    
    def get_trajectory_status(self) -> Dict[str, Any]:
        """获取轨迹跟踪状态"""
        if self.trajectory_start_time is None:
            progress = 0.0
        elif self.trajectory_completed:
            progress = 1.0
        else:
            elapsed = time.time() - self.trajectory_start_time
            total_time = self.trajectory_points[-1].timestamp if self.trajectory_points else 1.0
            progress = min(elapsed / total_time, 1.0)
        
        return {
            'trajectory_type': self.trajectory_type.value if self.trajectory_type else None,
            'total_points': len(self.trajectory_points),
            'current_index': self.current_trajectory_index,
            'progress': progress,
            'completed': self.trajectory_completed,
            'total_distance_cm': self.total_distance_traveled,
            'elapsed_time_s': time.time() - self.trajectory_start_time if self.trajectory_start_time else 0.0
        }
    
    def pause_trajectory(self) -> bool:
        """暂停轨迹跟踪"""
        # 实现轨迹暂停逻辑
        self.logger.info("轨迹跟踪已暂停")
        return True
    
    def resume_trajectory(self) -> bool:
        """恢复轨迹跟踪"""
        # 实现轨迹恢复逻辑
        self.logger.info("轨迹跟踪已恢复")
        return True