import csv
import time
import threading
from datetime import datetime
from pathlib import Path
from utils.logger import Logger
from config.settings import *


class FlightDataRecorder:
    def __init__(self, connection_manager):
        self.logger = Logger("FlightDataRecorder")
        self.connection_manager = connection_manager
        self.recording = False
        self.record_thread = None
        self.csv_file = None
        self.csv_writer = None
        self.csv_file_path = None
        self.record_start_time = None
        self.data_points_recorded = 0
        
        # CSV字段定义 - 专注核心飞行数据
        self.csv_headers = [
            'timestamp',          # 时间戳
            'relative_time',      # 相对开始时间(秒)
            'x_cm',              # X轴位置坐标(cm)
            'y_cm',              # Y轴位置坐标(cm)
            'z_cm',              # Z轴位置坐标(cm)
            'pitch_deg',         # 俯仰角(度)
            'roll_deg',          # 翻滚角(度)
            'yaw_deg',           # 偏航角(度)
            'vgx_cm_s',          # X轴速度分量(cm/s)
            'vgy_cm_s',          # Y轴速度分量(cm/s)  
            'vgz_cm_s',          # Z轴速度分量(cm/s)
            'agx_0001g',         # X轴加速度分量(0.001g)
            'agy_0001g',         # Y轴加速度分量(0.001g)
            'agz_0001g',         # Z轴加速度分量(0.001g)
        ]
    
    def start_recording(self, session_name=None):
        if not ENABLE_FLIGHT_DATA_RECORDING:
            self.logger.info("飞行数据记录已禁用")
            return False
            
        if self.recording:
            self.logger.warning("数据记录已在进行中")
            return False
            
        if not self.connection_manager.is_connected():
            self.logger.error("未连接到无人机，无法开始数据记录")
            return False
        
        try:
            # 创建CSV文件
            self._create_csv_file(session_name)
            
            # 启动记录线程
            self.recording = True
            self.record_start_time = time.time()
            self.data_points_recorded = 0
            
            self.record_thread = threading.Thread(target=self._record_worker)
            self.record_thread.daemon = True
            self.record_thread.start()
            
            self.logger.info(f"飞行数据记录已开始，文件: {self.csv_file_path}")
            return True
            
        except Exception as e:
            self.logger.error(f"开始数据记录失败: {e}")
            return False
    
    def stop_recording(self):
        if not self.recording:
            return
        
        try:
            self.recording = False
            
            if self.record_thread:
                self.record_thread.join(timeout=2)
            
            if self.csv_file:
                self.csv_file.close()
                self.csv_file = None
                self.csv_writer = None
            
            record_duration = time.time() - self.record_start_time if self.record_start_time else 0
            
            self.logger.info(f"数据记录已停止")
            self.logger.info(f"记录时长: {record_duration:.2f}秒, 数据点: {self.data_points_recorded}")
            self.logger.info(f"文件保存至: {self.csv_file_path}")
            
        except Exception as e:
            self.logger.error(f"停止数据记录失败: {e}")
    
    def _create_csv_file(self, session_name=None):
        # 确保目录存在
        FLIGHT_RECORDS_DIR.mkdir(parents=True, exist_ok=True)
        
        # 生成文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        if session_name:
            filename = f"{timestamp}_{session_name}_drone_data.csv"
        else:
            filename = f"{timestamp}_drone_data.csv"
        
        self.csv_file_path = FLIGHT_RECORDS_DIR / filename
        
        # 创建CSV文件并写入头部
        self.csv_file = open(self.csv_file_path, 'w', newline='', encoding=FLIGHT_DATA_CSV_ENCODING)
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(self.csv_headers)
        self.csv_file.flush()
    
    def _record_worker(self):
        while self.recording:
            try:
                if not self.connection_manager.is_connected():
                    self.logger.warning("连接丢失，停止数据记录")
                    break
                
                # 获取无人机数据
                data_row = self._collect_drone_data()
                
                # 写入CSV文件
                if data_row and self.csv_writer:
                    self.csv_writer.writerow(data_row)
                    self.csv_file.flush()
                    self.data_points_recorded += 1
                
                # 按配置间隔等待
                time.sleep(FLIGHT_DATA_RECORDING_INTERVAL)
                
            except Exception as e:
                self.logger.error(f"数据记录过程出错: {e}")
                time.sleep(1)
    
    def _collect_drone_data(self):
        try:
            tello = self.connection_manager.get_tello()
            current_time = time.time()
            
            # 获取完整状态字符串
            state = tello.get_current_state()
            
            data_row = []
            
            # 时间戳数据
            data_row.append(datetime.now().isoformat())
            data_row.append(current_time - self.record_start_time)
            
            # 位置坐标 (x, y, z) - 从状态字符串解析
            try:
                if state and isinstance(state, str):
                    position_data = self._parse_state_data(state, ['x', 'y', 'z'])
                    data_row.extend(position_data)
                else:
                    data_row.extend([None, None, None])
            except:
                data_row.extend([None, None, None])
            
            # 姿态角度 (pitch, roll, yaw)
            try:
                data_row.append(tello.get_pitch())
                data_row.append(tello.get_roll())
                data_row.append(tello.get_yaw())
            except:
                data_row.extend([None, None, None])
            
            # 速度分量 (vgx, vgy, vgz) - 从状态字符串解析
            try:
                if state and isinstance(state, str):
                    velocity_data = self._parse_state_data(state, ['vgx', 'vgy', 'vgz'])
                    data_row.extend(velocity_data)
                else:
                    data_row.extend([None, None, None])
            except:
                data_row.extend([None, None, None])
            
            # 加速度分量 (agx, agy, agz) - 从状态字符串解析
            try:
                if state and isinstance(state, str):
                    acceleration_data = self._parse_state_data(state, ['agx', 'agy', 'agz'])
                    data_row.extend(acceleration_data)
                else:
                    data_row.extend([None, None, None])
            except:
                data_row.extend([None, None, None])
            
            return data_row
            
        except Exception as e:
            self.logger.error(f"收集无人机数据失败: {e}")
            return None
    
    def _parse_state_data(self, state_string, fields):
        results = []
        for field in fields:
            try:
                # 解析状态字符串，格式类似: "field:value;"
                pattern = f"{field}:([^;]+)"
                import re
                match = re.search(pattern, state_string)
                if match:
                    value = match.group(1)
                    # 尝试转换为数字
                    try:
                        results.append(float(value))
                    except:
                        results.append(value)
                else:
                    results.append(None)
            except:
                results.append(None)
        return results
    
    def add_command_log(self, command, success=True, error_msg=""):
        """添加命令执行日志 - 暂时禁用以保持数据纯净性"""
        # 暂时禁用命令日志记录，保持数据文件的纯净性
        # 所有命令记录将通过程序日志文件记录
        pass
    
    def get_recording_status(self):
        if self.recording and self.record_start_time:
            duration = time.time() - self.record_start_time
            return {
                'recording': True,
                'duration': duration,
                'data_points': self.data_points_recorded,
                'file_path': str(self.csv_file_path),
                'interval': FLIGHT_DATA_RECORDING_INTERVAL
            }
        return {
            'recording': False,
            'duration': 0,
            'data_points': 0,
            'file_path': None,
            'interval': FLIGHT_DATA_RECORDING_INTERVAL
        }