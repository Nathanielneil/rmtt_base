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
        
        # CSV字段定义 - 针对RoboMaster TT ESP32-D2WD主控优化
        self.csv_headers = [
            'timestamp',          # 时间戳
            'relative_time',      # 相对实验开始时间(秒)
            'height_cm',         # 主要高度数据(cm) - 融合传感器
            'battery_percent',    # 电池电量(%)
            'temperature_deg',    # 温度(°C)
            'pitch_deg',         # 俯仰角(度)
            'roll_deg',          # 翻滚角(度)
            'yaw_deg',           # 偏航角(度)
            'tof_distance_cm',   # 红外TOF距离传感器(cm) - 近距离精确
            'barometer_cm',      # 气压计高度(cm) - 绝对高度
            'height_diff_cm',    # TOF与气压计高度差(cm) - 地面检测
            'vgx_cm_s',          # X轴速度分量(cm/s)
            'vgy_cm_s',          # Y轴速度分量(cm/s)  
            'vgz_cm_s',          # Z轴速度分量(cm/s)
            'agx_0001g',         # X轴加速度分量(0.001g)
            'agy_0001g',         # Y轴加速度分量(0.001g)
            'agz_0001g',         # Z轴加速度分量(0.001g)
            'wifi_snr',          # WiFi信号强度 - ESP32网络质量
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
            
            data_row = []
            
            # 时间戳数据
            data_row.append(datetime.now().isoformat())
            data_row.append(round(current_time - self.record_start_time, 3))  # 相对时间(秒)，保留3位小数
            
            # 高度数据 - 优先使用TOF传感器提供厘米级精度
            try:
                # 优先使用TOF传感器（更精确）
                tof_height = tello.get_distance_tof()
                if tof_height is not None and tof_height > 0:
                    data_row.append(tof_height)  # TOF传感器厘米级精度
                else:
                    # TOF无效时使用API高度
                    api_height = tello.get_height()
                    data_row.append(api_height if api_height is not None else 0)
            except Exception as e:
                if self.data_points_recorded < 5:
                    self.logger.warning(f"获取高度失败: {e}")
                data_row.append(0)
            
            # 电池电量
            try:
                battery = tello.get_battery()
                data_row.append(battery if battery is not None else 0)
            except:
                data_row.append(0)
            
            # 温度
            try:
                temp = tello.get_temperature()
                data_row.append(temp if temp is not None else 20)  # 默认室温
            except:
                data_row.append(20)
            
            # 获取完整状态字符串并解析所有传感器数据
            tof_distance = 0
            baro_height = 0
            
            try:
                # 使用原始UDP状态数据而非get_current_state()
                state = None
                try:
                    # 直接访问Tello的状态数据
                    if hasattr(tello, 'get_current_state'):
                        state = tello.get_current_state()
                    elif hasattr(tello, 'state'):
                        # 尝试访问内部状态
                        state_dict = tello.state
                        if state_dict:
                            # 构建状态字符串
                            state_parts = []
                            for key, value in state_dict.items():
                                state_parts.append(f"{key}:{value}")
                            state = ";".join(state_parts) + ";"
                except:
                    state = None
                
                if state and isinstance(state, str) and len(state) > 20:
                    # 仅在第一次记录时显示ESP32状态字符串示例
                    if self.data_points_recorded == 0:
                        self.logger.info(f"RoboMaster TT ESP32状态字符串: {state[:200]}...")
                    
                    # 解析姿态角度 (pitch, roll, yaw)
                    attitude_data = self._parse_state_data(state, ['pitch', 'roll', 'yaw'])
                    pitch_val = attitude_data[0] if attitude_data[0] is not None else 0.0
                    roll_val = attitude_data[1] if attitude_data[1] is not None else 0.0
                    yaw_val = attitude_data[2] if attitude_data[2] is not None else 0.0
                    data_row.extend([pitch_val, roll_val, yaw_val])
                    
                    # 解析传感器数据 - RoboMaster TT特有的红外定高和气压计
                    sensor_data = self._parse_state_data(state, ['tof', 'baro'])
                    tof_distance = sensor_data[0] if sensor_data[0] is not None else 0
                    baro_height = sensor_data[1] if sensor_data[1] is not None else 0
                    data_row.extend([tof_distance, baro_height])
                    
                    # 计算TOF与气压计高度差
                    height_diff = abs(tof_distance - baro_height) if (tof_distance > 0 and baro_height > 0) else 0
                    data_row.append(height_diff)
                    
                    # 解析ESP32提供的速度数据
                    velocity_data = self._parse_state_data(state, ['vgx', 'vgy', 'vgz'])
                    vgx = velocity_data[0] if velocity_data[0] is not None else 0.0
                    vgy = velocity_data[1] if velocity_data[1] is not None else 0.0
                    vgz = velocity_data[2] if velocity_data[2] is not None else 0.0
                    data_row.extend([vgx, vgy, vgz])
                    
                    # 解析ESP32提供的加速度数据
                    acceleration_data = self._parse_state_data(state, ['agx', 'agy', 'agz'])
                    agx = acceleration_data[0] if acceleration_data[0] is not None else 0
                    agy = acceleration_data[1] if acceleration_data[1] is not None else 0
                    agz = acceleration_data[2] if acceleration_data[2] is not None else 0
                    data_row.extend([agx, agy, agz])
                    
                    # 尝试解析WiFi信号质量（如果存在）
                    wifi_data = self._parse_state_data(state, ['wifi_snr', 'wifi', 'snr'])
                    wifi_snr = -1
                    for val in wifi_data:
                        if val is not None and val != -1:
                            wifi_snr = val
                            break
                    data_row.append(wifi_snr)
                    
                else:
                    # 状态字符串无效或为空时使用API调用备用方案
                    if self.data_points_recorded < 3:
                        self.logger.warning(f"ESP32状态字符串无效或为空: '{state}', 使用API备用方案")
                    
                    # 姿态角度 - 直接API调用
                    try:
                        pitch = float(tello.get_pitch() or 0.0)
                        roll = float(tello.get_roll() or 0.0)
                        yaw = float(tello.get_yaw() or 0.0)
                        data_row.extend([pitch, roll, yaw])
                    except Exception as api_error:
                        if self.data_points_recorded < 3:
                            self.logger.debug(f"姿态角度API调用失败: {api_error}")
                        data_row.extend([0.0, 0.0, 0.0])
                    
                    # 传感器数据
                    try:
                        tof_distance = int(tello.get_distance_tof() or 0)
                        baro_height = float(tello.get_barometer() or 0)
                        height_diff = abs(tof_distance - baro_height) if (tof_distance > 0 and baro_height > 0) else 0
                        data_row.extend([tof_distance, baro_height, height_diff])
                    except Exception as sensor_error:
                        if self.data_points_recorded < 3:
                            self.logger.debug(f"传感器API调用失败: {sensor_error}")
                        data_row.extend([0, 0, 0])
                    
                    # 速度和加速度数据无法通过单独API获取
                    data_row.extend([0.0, 0.0, 0.0])  # vgx, vgy, vgz
                    data_row.extend([0, 0, 0])        # agx, agy, agz
                    data_row.append(-1)               # wifi_snr 未知
                    
            except Exception as e:
                if self.data_points_recorded < 5:
                    self.logger.error(f"ESP32数据解析失败: {e}")
                # 填充默认值确保数据完整性
                data_row.extend([0.0, 0.0, 0.0])  # pitch, roll, yaw
                data_row.extend([0, 0, 0])        # tof, baro, height_diff
                data_row.extend([0.0, 0.0, 0.0])  # vgx, vgy, vgz
                data_row.extend([0, 0, 0])        # agx, agy, agz
                data_row.append(-1)               # wifi_snr
            
            return data_row
            
        except Exception as e:
            self.logger.error(f"收集无人机数据失败: {e}")
            return None
    
    def _parse_state_data(self, state_string, fields):
        """解析RoboMaster TT状态字符串中的数据字段"""
        results = []
        import re
        
        for field in fields:
            try:
                # RoboMaster TT状态字符串格式通常为: "field:value;"
                # 支持多种可能的分隔符和格式
                patterns = [
                    f"{field}:([^;,\\s]+)",  # field:value; 或 field:value,
                    f"{field}=([^;,\\s]+)",  # field=value; 或 field=value,
                    f"{field}\\s+([^;,\\s]+)",  # field value
                ]
                
                value = None
                for pattern in patterns:
                    match = re.search(pattern, state_string, re.IGNORECASE)
                    if match:
                        raw_value = match.group(1).strip()
                        try:
                            # 尝试转换为浮点数
                            value = float(raw_value)
                            break
                        except ValueError:
                            try:
                                # 尝试转换为整数
                                value = int(raw_value)
                                break
                            except ValueError:
                                # 如果都失败，保留字符串值
                                value = raw_value
                                break
                
                results.append(value)
                
            except Exception as e:
                if self.data_points_recorded < 3:  # 只在开始时记录解析错误
                    self.logger.debug(f"解析字段 '{field}' 失败: {e}")
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