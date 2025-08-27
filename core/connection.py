import time
import socket
import threading
from djitellopy import Tello
from utils.logger import Logger
from utils.exceptions import TelloConnectionError
from config.settings import *


class ConnectionManager:
    def __init__(self):
        self.logger = Logger("ConnectionManager")
        self.tello = None
        self.connected = False
        self.monitoring = False
        self._monitor_thread = None
        self.connection_lost_callback = None
    
    def connect(self):
        try:
            self.logger.info("正在连接Tello无人机...")
            self.tello = Tello()
            
            self.tello.connect()
            
            if not self.tello.get_battery():
                raise TelloConnectionError("无法获取电池信息，连接可能失败")
            
            battery = self.tello.get_battery()
            self.logger.info(f"成功连接到Tello，电池电量: {battery}%")
            
            if battery < BATTERY_WARNING_THRESHOLD:
                self.logger.warning(f"电池电量较低: {battery}%")
            
            self.connected = True
            return True
            
        except Exception as e:
            self.logger.error(f"连接失败: {str(e)}")
            raise TelloConnectionError(f"连接失败: {str(e)}")
    
    def disconnect(self):
        try:
            if self.tello and self.connected:
                self.stop_monitoring()
                self.logger.info("正在断开Tello连接...")
                self.tello.end()
                self.connected = False
                self.logger.info("已断开连接")
        except Exception as e:
            self.logger.error(f"断开连接时出错: {str(e)}")
    
    def is_connected(self):
        if not self.connected or not self.tello:
            return False
        
        try:
            battery = self.tello.get_battery()
            return battery is not None
        except:
            return False
    
    def start_monitoring(self, callback=None):
        if self.monitoring:
            return
        
        self.connection_lost_callback = callback
        self.monitoring = True
        self._monitor_thread = threading.Thread(target=self._monitor_connection)
        self._monitor_thread.daemon = True
        self._monitor_thread.start()
        self.logger.info("开始监控连接状态")
    
    def stop_monitoring(self):
        self.monitoring = False
        if self._monitor_thread:
            self._monitor_thread.join(timeout=1)
        self.logger.info("停止监控连接状态")
    
    def _monitor_connection(self):
        consecutive_failures = 0
        
        while self.monitoring:
            try:
                if not self.is_connected():
                    consecutive_failures += 1
                    self.logger.warning(f"连接检查失败 ({consecutive_failures}/3)")
                    
                    if consecutive_failures >= 3:
                        self.logger.critical("检测到连接丢失!")
                        self.connected = False
                        if self.connection_lost_callback:
                            self.connection_lost_callback()
                        break
                else:
                    consecutive_failures = 0
                
                time.sleep(2)
                
            except Exception as e:
                self.logger.error(f"连接监控出错: {str(e)}")
                consecutive_failures += 1
    
    def get_tello(self):
        if not self.connected:
            raise TelloConnectionError("未连接到Tello")
        return self.tello