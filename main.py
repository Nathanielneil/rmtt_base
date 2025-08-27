#!/usr/bin/env python3

import sys
import argparse
import signal
import atexit
import time
from datetime import datetime
from pathlib import Path
from colorama import init, Fore, Style

sys.path.append(str(Path(__file__).parent))

from core.connection import ConnectionManager
from core.tello_controller import TelloController
from media.video_stream import VideoStreamHandler
from media.media_saver import MediaSaver
from data.flight_data_recorder import FlightDataRecorder
from utils.logger import Logger
from utils.exceptions import TelloConnectionError, TelloControlError

init()


class TelloCommander:
    def __init__(self):
        self.logger = Logger("TelloCommander")
        self.connection_manager = ConnectionManager()
        self.controller = None
        self.video_handler = None
        self.media_saver = MediaSaver()
        self.flight_recorder = None
        self.running = True
        self.session_start_time = datetime.now()
        self.session_stats = {
            'photos_taken': 0,
            'videos_recorded': 0,
            'flight_time': 0,
            'commands_executed': 0
        }
        
        signal.signal(signal.SIGINT, self._signal_handler)
        atexit.register(self._cleanup_on_exit)
    
    def _signal_handler(self, signum, frame):
        print(f"\n{Fore.YELLOW}接收到中断信号，正在安全退出...{Style.RESET_ALL}")
        self.shutdown()
        sys.exit(0)
    
    def _cleanup_on_exit(self):
        if self.running:
            self.logger.info("程序退出时执行清理操作...")
            self._auto_save_media_data()
    
    def _auto_save_media_data(self):
        from config.settings import AUTO_SAVE_ON_EXIT, CREATE_SESSION_BACKUP, GENERATE_SESSION_REPORT
        
        if not AUTO_SAVE_ON_EXIT:
            return
            
        try:
            self.logger.info("开始自动保存媒体数据...")
            
            # 1. 停止任何正在进行的录制
            if self.video_handler and self.video_handler.recording:
                self.logger.info("检测到正在录制，停止录制并保存视频...")
                self.video_handler.stop_recording()
                self.session_stats['videos_recorded'] += 1
            
            # 2. 停止视频流
            if self.video_handler and self.video_handler.streaming:
                self.video_handler.stop_stream()
            
            # 3. 创建媒体备份 (可配置)
            if CREATE_SESSION_BACKUP:
                backup_path = self.media_saver.create_media_backup()
                if backup_path:
                    self.logger.info(f"媒体备份已创建: {backup_path}")
            
            # 4. 生成会话报告 (可配置)
            if GENERATE_SESSION_REPORT:
                self._generate_session_report()
            
            # 5. 获取并显示统计信息
            stats = self.media_saver.get_media_stats()
            if stats:
                print(f"\n{Fore.CYAN}=== 本次会话媒体统计 ==={Style.RESET_ALL}")
                print(f"拍摄照片: {self.session_stats['photos_taken']}张")
                print(f"录制视频: {self.session_stats['videos_recorded']}个") 
                print(f"总图片数: {stats['images']['count']}张 ({stats['images']['size_mb']}MB)")
                print(f"总视频数: {stats['videos']['count']}个 ({stats['videos']['size_mb']}MB)")
                print(f"媒体总大小: {stats['total_size_mb']}MB")
                print(f"执行命令: {self.session_stats['commands_executed']}次")
                print(f"飞行时间: {self.session_stats['flight_time']}秒")
            
            self.logger.info("媒体数据自动保存完成")
            
        except Exception as e:
            self.logger.error(f"自动保存媒体数据失败: {e}")
    
    def _generate_session_report(self):
        try:
            session_duration = datetime.now() - self.session_start_time
            report_content = f"""
=== RMTT 会话报告 ===
会话开始时间: {self.session_start_time.strftime('%Y-%m-%d %H:%M:%S')}
会话结束时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
会话总时长: {str(session_duration).split('.')[0]}
拍摄照片数: {self.session_stats['photos_taken']}
录制视频数: {self.session_stats['videos_recorded']}
飞行时间: {self.session_stats['flight_time']}秒
执行命令数: {self.session_stats['commands_executed']}
"""
            
            # 保存报告到logs目录
            from config.settings import LOGS_DIR
            LOGS_DIR.mkdir(exist_ok=True)
            
            report_filename = f"session_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
            report_path = LOGS_DIR / report_filename
            
            with open(report_path, 'w', encoding='utf-8') as f:
                f.write(report_content)
            
            self.logger.info(f"会话报告已保存: {report_path}")
            
        except Exception as e:
            self.logger.error(f"生成会话报告失败: {e}")
    
    def connect(self):
        try:
            self.connection_manager.connect()
            self.controller = TelloController(self.connection_manager)
            self.video_handler = VideoStreamHandler(self.connection_manager)
            self.flight_recorder = FlightDataRecorder(self.connection_manager)
            self.logger.info("无人机控制系统已就绪")
            return True
        except Exception as e:
            self.logger.error(f"连接失败: {e}")
            return False
    
    def disconnect(self):
        if self.flight_recorder:
            self.flight_recorder.stop_recording()
        if self.video_handler:
            self.video_handler.stop_stream()
        if self.controller and self.controller.in_flight:
            try:
                self.controller.land()
            except:
                pass
        self.connection_manager.disconnect()
    
    def shutdown(self):
        self.running = False
        self._auto_save_media_data()  # 确保退出时保存数据
        self.disconnect()
    
    def interactive_mode(self):
        from config.settings import AUTO_SAVE_ON_EXIT
        
        print(f"{Fore.CYAN}=== Tello无人机控制系统 ==={Style.RESET_ALL}")
        print("输入 'help' 查看可用命令，输入 'quit' 退出")
        
        if AUTO_SAVE_ON_EXIT:
            print(f"{Fore.GREEN}自动保存已启用：程序退出时将自动保存视频和生成报告{Style.RESET_ALL}")
        
        print()
        
        while self.running:
            try:
                command = input(f"{Fore.GREEN}Tello> {Style.RESET_ALL}").strip().lower()
                
                if not command:
                    continue
                
                if command == 'quit' or command == 'exit':
                    break
                
                self._execute_command(command)
                
            except KeyboardInterrupt:
                break
            except Exception as e:
                self.logger.error(f"命令执行出错: {e}")
    
    def _execute_command(self, command_line):
        parts = command_line.split()
        command = parts[0]
        args = parts[1:] if len(parts) > 1 else []
        
        try:
            # 记录命令执行次数
            self.session_stats['commands_executed'] += 1
            if command == 'help':
                self._show_help()
            
            elif command == 'status':
                self._show_status()
            
            elif command == 'takeoff':
                if self.controller:
                    flight_start = time.time()
                    self.controller.takeoff()
                    self.session_stats['flight_start_time'] = flight_start
                    # 自动开始飞行数据记录
                    if self.flight_recorder:
                        self.flight_recorder.start_recording()
                        self.flight_recorder.add_command_log('takeoff', True)
            
            elif command == 'land':
                if self.controller:
                    if 'flight_start_time' in self.session_stats:
                        flight_duration = int(time.time() - self.session_stats['flight_start_time'])
                        self.session_stats['flight_time'] += flight_duration
                    self.controller.land()
                    # 停止飞行数据记录
                    if self.flight_recorder:
                        self.flight_recorder.add_command_log('land', True)
                        self.flight_recorder.stop_recording()
            
            elif command == 'emergency':
                if self.controller:
                    self.controller.emergency_stop()
            
            elif command in ['up', 'down', 'left', 'right', 'forward', 'back']:
                distance = int(args[0]) if args else 50
                self._move_drone(command, distance)
            
            elif command in ['cw', 'ccw']:
                angle = int(args[0]) if args else 90
                self._rotate_drone(command, angle)
            
            elif command == 'flip':
                direction = args[0] if args else 'f'
                if self.controller:
                    self.controller.flip(direction)
            
            elif command == 'hover':
                duration = int(args[0]) if args else 3
                if self.controller:
                    self.controller.hover(duration)
            
            elif command == 'stream':
                action = args[0] if args else 'start'
                self._handle_stream(action)
            
            elif command == 'record':
                action = args[0] if args else 'start'
                if action == 'start':
                    self.session_stats['videos_recorded'] += 1
                self._handle_recording(action)
            
            elif command == 'photo':
                self.session_stats['photos_taken'] += 1
                self._take_photo()
            
            elif command == 'media':
                self._show_media_info()
            
            elif command == 'record_data':
                action = args[0] if args else 'status'
                self._handle_data_recording(action)
            
            else:
                print(f"{Fore.RED}未知命令: {command}{Style.RESET_ALL}")
                
        except Exception as e:
            self.logger.error(f"执行命令 '{command}' 失败: {e}")
    
    def _move_drone(self, direction, distance):
        if not self.controller:
            return
        
        direction_map = {
            'up': self.controller.move_up,
            'down': self.controller.move_down,
            'left': self.controller.move_left,
            'right': self.controller.move_right,
            'forward': self.controller.move_forward,
            'back': self.controller.move_back
        }
        
        if direction in direction_map:
            direction_map[direction](distance)
            # 记录移动命令
            if self.flight_recorder:
                self.flight_recorder.add_command_log(f'{direction} {distance}', True)
    
    def _rotate_drone(self, direction, angle):
        if not self.controller:
            return
        
        if direction == 'cw':
            self.controller.rotate_clockwise(angle)
            if self.flight_recorder:
                self.flight_recorder.add_command_log(f'cw {angle}', True)
        elif direction == 'ccw':
            self.controller.rotate_counter_clockwise(angle)
            if self.flight_recorder:
                self.flight_recorder.add_command_log(f'ccw {angle}', True)
    
    def _handle_stream(self, action):
        if not self.video_handler:
            return
        
        if action == 'start':
            self.video_handler.start_stream()
        elif action == 'stop':
            self.video_handler.stop_stream()
    
    def _handle_recording(self, action):
        if not self.video_handler:
            return
        
        if action == 'start':
            self.video_handler.start_recording()
        elif action == 'stop':
            self.video_handler.stop_recording()
    
    def _take_photo(self):
        if self.video_handler and self.video_handler.streaming:
            self.video_handler.capture_image()
            if self.flight_recorder:
                self.flight_recorder.add_command_log('photo', True)
        else:
            print(f"{Fore.YELLOW}请先启动视频流 (stream start){Style.RESET_ALL}")
    
    def _handle_data_recording(self, action):
        if not self.flight_recorder:
            print(f"{Fore.RED}数据记录器未初始化{Style.RESET_ALL}")
            return
        
        if action == 'start':
            session_name = input("输入会话名称 (可选，直接回车跳过): ").strip()
            if session_name:
                success = self.flight_recorder.start_recording(session_name)
            else:
                success = self.flight_recorder.start_recording()
            
            if success:
                print(f"{Fore.GREEN}飞行数据记录已开始{Style.RESET_ALL}")
            else:
                print(f"{Fore.RED}启动数据记录失败{Style.RESET_ALL}")
        
        elif action == 'stop':
            self.flight_recorder.stop_recording()
            print(f"{Fore.GREEN}飞行数据记录已停止{Style.RESET_ALL}")
        
        elif action == 'status':
            status = self.flight_recorder.get_recording_status()
            if status['recording']:
                print(f"{Fore.CYAN}=== 数据记录状态 ==={Style.RESET_ALL}")
                print(f"记录中: 是")
                print(f"记录时长: {status['duration']:.1f}秒")
                print(f"数据点数: {status['data_points']}")
                print(f"采样频率: {1/status['interval']:.1f}Hz")
                print(f"文件路径: {status['file_path']}")
            else:
                print(f"{Fore.YELLOW}数据记录未启动{Style.RESET_ALL}")
        
        else:
            print(f"{Fore.RED}未知操作: {action}，支持: start/stop/status{Style.RESET_ALL}")
    
    def _show_status(self):
        if self.controller:
            status = self.controller.get_status()
            if status:
                print(f"{Fore.CYAN}=== 无人机状态 ==={Style.RESET_ALL}")
                print(f"连接状态: {'已连接' if status['connected'] else '未连接'}")
                print(f"飞行状态: {'飞行中' if status['in_flight'] else '地面'}")
                print(f"电池电量: {status['battery']}%")
                print(f"飞行高度: {status['height']}cm")
                print(f"温度: {status['temperature']}°C")
                if 'flight_time' in status:
                    print(f"飞行时间: {status['flight_time']}秒")
    
    def _show_media_info(self):
        stats = self.media_saver.get_media_stats()
        if stats:
            print(f"{Fore.CYAN}=== 媒体文件统计 ==={Style.RESET_ALL}")
            print(f"图片: {stats['images']['count']} 个, {stats['images']['size_mb']} MB")
            print(f"视频: {stats['videos']['count']} 个, {stats['videos']['size_mb']} MB")
            print(f"总大小: {stats['total_size_mb']} MB")
    
    def _show_help(self):
        help_text = f"""
{Fore.CYAN}=== 可用命令 ==={Style.RESET_ALL}

{Fore.YELLOW}连接和状态:{Style.RESET_ALL}
  status              - 显示无人机状态
  help                - 显示此帮助信息
  quit/exit           - 退出程序 (自动保存数据)

{Fore.YELLOW}飞行控制:{Style.RESET_ALL}
  takeoff             - 起飞
  land                - 降落
  emergency           - 紧急停止
  hover [时长]        - 悬停 (默认3秒)

{Fore.YELLOW}移动控制:{Style.RESET_ALL}
  up [距离]           - 上升 (默认50cm)
  down [距离]         - 下降 (默认50cm)
  left [距离]         - 左移 (默认50cm)
  right [距离]        - 右移 (默认50cm)
  forward [距离]      - 前进 (默认50cm)
  back [距离]         - 后退 (默认50cm)
  cw [角度]           - 顺时针旋转 (默认90度)
  ccw [角度]          - 逆时针旋转 (默认90度)
  flip [方向]         - 翻滚 (f/b/l/r, 默认f)

{Fore.YELLOW}视频和拍照:{Style.RESET_ALL}
  stream start/stop   - 启动/停止视频流
  record start/stop   - 开始/停止录制
  photo               - 拍照 (自动计入统计)
  media               - 显示媒体文件统计

{Fore.YELLOW}飞行数据记录:{Style.RESET_ALL}
  record_data start   - 开始记录飞行数据 (50Hz频率)
  record_data stop    - 停止记录飞行数据
  record_data status  - 查看记录状态
  注: 起飞时自动开始记录，降落时自动停止

{Fore.YELLOW}自动保存功能:{Style.RESET_ALL}
  程序退出时自动执行:
     - 停止进行中的录制和数据记录
     - 创建媒体文件备份
     - 生成详细会话报告
     - 显示统计信息

{Fore.YELLOW}示例:{Style.RESET_ALL}
  takeoff             - 起飞 (自动开始数据记录)
  stream start        - 开启视频流
  record start        - 开始录制视频
  up 100              - 上升100cm
  photo               - 拍照
  record_data status  - 查看数据记录状态
  land                - 降落 (自动停止数据记录)
  quit                - 退出 (自动保存所有数据)
"""
        print(help_text)


def main():
    parser = argparse.ArgumentParser(description="Tello无人机控制程序")
    parser.add_argument('--auto-connect', action='store_true', help='自动连接到无人机')
    args = parser.parse_args()
    
    commander = TelloCommander()
    
    try:
        if args.auto_connect or input("是否连接到Tello无人机? (y/n): ").lower() == 'y':
            if commander.connect():
                commander.interactive_mode()
            else:
                print("连接失败，程序退出")
        else:
            print("未连接到无人机，程序退出")
    
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    
    finally:
        commander.shutdown()
        print("程序已退出")


if __name__ == "__main__":
    main()