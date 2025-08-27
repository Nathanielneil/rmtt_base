import time
from djitellopy import Tello
from utils.logger import Logger
from utils.safety import SafetyManager
from utils.exceptions import TelloControlError, TelloConnectionError
from config.settings import *


class TelloController:
    def __init__(self, connection_manager):
        self.logger = Logger("TelloController")
        self.connection_manager = connection_manager
        self.safety_manager = SafetyManager(connection_manager)
        self.in_flight = False
        self.flight_start_time = None
    
    def takeoff(self):
        try:
            if not self.connection_manager.is_connected():
                raise TelloConnectionError("未连接到Tello")
            
            if self.in_flight:
                self.logger.warning("无人机已经在飞行中")
                return
            
            tello = self.connection_manager.get_tello()
            battery = tello.get_battery()
            
            self.logger.info(f"准备起飞，电池电量: {battery}%")
            
            if not self.safety_manager.check_flight_safety('takeoff'):
                raise TelloControlError("安全检查失败，取消起飞")
            
            self.safety_manager.start_safety_monitoring(self._on_emergency)
            self.connection_manager.start_monitoring(self._on_connection_lost)
            
            tello.takeoff()
            self.in_flight = True
            self.flight_start_time = time.time()
            
            self.logger.flight_log("起飞", f"电池: {battery}%")
            
        except Exception as e:
            self.logger.error(f"起飞失败: {str(e)}")
            raise TelloControlError(f"起飞失败: {str(e)}")
    
    def land(self):
        try:
            if not self.in_flight:
                self.logger.warning("无人机未在飞行中")
                return
            
            tello = self.connection_manager.get_tello()
            tello.land()
            
            self.in_flight = False
            flight_time = int(time.time() - self.flight_start_time) if self.flight_start_time else 0
            battery = tello.get_battery()
            
            self.safety_manager.stop_safety_monitoring()
            self.connection_manager.stop_monitoring()
            
            self.logger.flight_log("降落", f"飞行时间: {flight_time}秒, 剩余电量: {battery}%")
            
        except Exception as e:
            self.logger.error(f"降落失败: {str(e)}")
            raise TelloControlError(f"降落失败: {str(e)}")
    
    def move_up(self, distance=DEFAULT_MOVEMENT_DISTANCE):
        self._execute_movement('up', distance, f"上升 {distance}cm")
    
    def move_down(self, distance=DEFAULT_MOVEMENT_DISTANCE):
        self._execute_movement('down', distance, f"下降 {distance}cm")
    
    def move_left(self, distance=DEFAULT_MOVEMENT_DISTANCE):
        self._execute_movement('left', distance, f"左移 {distance}cm")
    
    def move_right(self, distance=DEFAULT_MOVEMENT_DISTANCE):
        self._execute_movement('right', distance, f"右移 {distance}cm")
    
    def move_forward(self, distance=DEFAULT_MOVEMENT_DISTANCE):
        self._execute_movement('forward', distance, f"前进 {distance}cm")
    
    def move_back(self, distance=DEFAULT_MOVEMENT_DISTANCE):
        self._execute_movement('back', distance, f"后退 {distance}cm")
    
    def rotate_clockwise(self, degrees=DEFAULT_ROTATION_ANGLE):
        self._execute_movement('cw', degrees, f"顺时针旋转 {degrees}度")
    
    def rotate_counter_clockwise(self, degrees=DEFAULT_ROTATION_ANGLE):
        self._execute_movement('ccw', degrees, f"逆时针旋转 {degrees}度")
    
    def flip(self, direction='f'):
        try:
            if not self.in_flight:
                raise TelloControlError("必须在飞行中才能翻滚")
            
            if direction not in ['f', 'b', 'l', 'r']:
                raise TelloControlError("翻滚方向必须是 f(前), b(后), l(左), r(右)")
            
            if not self.safety_manager.check_flight_safety('flip'):
                raise TelloControlError("安全检查失败，取消翻滚")
            
            tello = self.connection_manager.get_tello()
            tello.flip(direction)
            
            direction_map = {'f': '前', 'b': '后', 'l': '左', 'r': '右'}
            self.logger.flight_log("翻滚", f"方向: {direction_map[direction]}")
            
        except Exception as e:
            self.logger.error(f"翻滚失败: {str(e)}")
            raise TelloControlError(f"翻滚失败: {str(e)}")
    
    def hover(self, duration=3):
        self.logger.flight_log("悬停", f"时长: {duration}秒")
        time.sleep(duration)
    
    def _execute_movement(self, command, value, description):
        try:
            if not self.in_flight:
                raise TelloControlError("必须在飞行中才能移动")
            
            if not self.safety_manager.check_flight_safety(command, value):
                raise TelloControlError(f"安全检查失败: {description}")
            
            tello = self.connection_manager.get_tello()
            
            if command == 'up':
                tello.move_up(value)
            elif command == 'down':
                tello.move_down(value)
            elif command == 'left':
                tello.move_left(value)
            elif command == 'right':
                tello.move_right(value)
            elif command == 'forward':
                tello.move_forward(value)
            elif command == 'back':
                tello.move_back(value)
            elif command == 'cw':
                tello.rotate_clockwise(value)
            elif command == 'ccw':
                tello.rotate_counter_clockwise(value)
            
            self.logger.flight_log("移动", description)
            
        except Exception as e:
            self.logger.error(f"移动失败: {str(e)}")
            raise TelloControlError(f"移动失败: {str(e)}")
    
    def get_status(self):
        try:
            tello = self.connection_manager.get_tello()
            battery = tello.get_battery()
            height = tello.get_height()
            temperature = tello.get_temperature()
            
            status = {
                'connected': self.connection_manager.is_connected(),
                'in_flight': self.in_flight,
                'battery': battery,
                'height': height,
                'temperature': temperature
            }
            
            if self.flight_start_time:
                status['flight_time'] = int(time.time() - self.flight_start_time)
            
            return status
            
        except Exception as e:
            self.logger.error(f"获取状态失败: {str(e)}")
            return None
    
    def emergency_stop(self):
        try:
            self.logger.critical("执行紧急停止")
            tello = self.connection_manager.get_tello()
            tello.emergency()
            self.in_flight = False
            self.safety_manager.stop_safety_monitoring()
            self.connection_manager.stop_monitoring()
            
        except Exception as e:
            self.logger.error(f"紧急停止失败: {str(e)}")
    
    def _on_emergency(self, reason):
        self.logger.critical(f"触发紧急事件: {reason}")
        self.in_flight = False
    
    def _on_connection_lost(self):
        self.logger.critical("连接丢失回调")
        self.in_flight = False