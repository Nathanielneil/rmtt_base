import threading
import time
from utils.logger import Logger
from utils.exceptions import TelloSafetyError
from config.settings import *


class SafetyManager:
    def __init__(self, connection_manager):
        self.logger = Logger("SafetyManager")
        self.connection_manager = connection_manager
        self.monitoring = False
        self.emergency_landed = False
        self._monitor_thread = None
        self.emergency_callback = None
    
    def start_safety_monitoring(self, emergency_callback=None):
        if not ENABLE_SAFETY_CHECKS:
            self.logger.info("安全检查已禁用")
            return
        
        if self.monitoring:
            return
        
        self.emergency_callback = emergency_callback
        self.monitoring = True
        self._monitor_thread = threading.Thread(target=self._safety_monitor)
        self._monitor_thread.daemon = True
        self._monitor_thread.start()
        self.logger.info("安全监控已启动")
    
    def stop_safety_monitoring(self):
        self.monitoring = False
        if self._monitor_thread:
            self._monitor_thread.join(timeout=1)
        self.logger.info("安全监控已停止")
    
    def _safety_monitor(self):
        while self.monitoring:
            try:
                if not self.connection_manager.is_connected():
                    if AUTO_LAND_ON_CONNECTION_LOST and not self.emergency_landed:
                        self.logger.critical("连接丢失，触发紧急降落")
                        self._trigger_emergency_landing("连接丢失")
                    break
                
                tello = self.connection_manager.get_tello()
                battery = tello.get_battery()
                
                if battery <= BATTERY_CRITICAL_THRESHOLD and not self.emergency_landed:
                    self.logger.critical(f"电池电量危急: {battery}%，触发紧急降落")
                    self._trigger_emergency_landing(f"电池电量危急: {battery}%")
                
                elif battery <= BATTERY_WARNING_THRESHOLD:
                    self.logger.warning(f"电池电量警告: {battery}%")
                
                time.sleep(1)
                
            except Exception as e:
                self.logger.error(f"安全监控出错: {str(e)}")
                time.sleep(2)
    
    def _trigger_emergency_landing(self, reason):
        try:
            self.emergency_landed = True
            self.logger.critical(f"执行紧急降落: {reason}")
            
            if self.connection_manager.is_connected():
                tello = self.connection_manager.get_tello()
                tello.emergency()
            
            if self.emergency_callback:
                self.emergency_callback(reason)
                
        except Exception as e:
            self.logger.error(f"紧急降落失败: {str(e)}")
    
    def check_flight_safety(self, command, value=None):
        if not ENABLE_SAFETY_CHECKS:
            return True
        
        try:
            tello = self.connection_manager.get_tello()
            
            if command in ['up', 'down', 'left', 'right', 'forward', 'back']:
                if value and value > SPEED_LIMIT:
                    raise TelloSafetyError(f"移动距离 {value} 超过安全限制 {SPEED_LIMIT}")
            
            elif command in ['cw', 'ccw']:
                if value and abs(value) > 360:
                    raise TelloSafetyError(f"旋转角度 {value} 超过安全限制")
            
            elif command == 'up':
                if value and value > FLIGHT_ALTITUDE_LIMIT:
                    raise TelloSafetyError(f"飞行高度 {value} 超过安全限制 {FLIGHT_ALTITUDE_LIMIT}")
            
            battery = tello.get_battery()
            if battery <= BATTERY_CRITICAL_THRESHOLD:
                raise TelloSafetyError(f"电池电量过低 ({battery}%)，拒绝执行飞行命令")
            
            return True
            
        except TelloSafetyError:
            raise
        except Exception as e:
            self.logger.warning(f"安全检查时出错: {str(e)}")
            return False
    
    def force_emergency_land(self):
        self._trigger_emergency_landing("用户手动触发")