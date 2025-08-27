#!/usr/bin/env python3

import sys
import argparse
import signal
from pathlib import Path
from colorama import init, Fore, Style

sys.path.append(str(Path(__file__).parent))

from core.connection import ConnectionManager
from core.tello_controller import TelloController
from media.video_stream import VideoStreamHandler
from media.media_saver import MediaSaver
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
        self.running = True
        
        signal.signal(signal.SIGINT, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        print(f"\n{Fore.YELLOW}接收到中断信号，正在安全退出...{Style.RESET_ALL}")
        self.shutdown()
        sys.exit(0)
    
    def connect(self):
        try:
            self.connection_manager.connect()
            self.controller = TelloController(self.connection_manager)
            self.video_handler = VideoStreamHandler(self.connection_manager)
            self.logger.info("无人机控制系统已就绪")
            return True
        except Exception as e:
            self.logger.error(f"连接失败: {e}")
            return False
    
    def disconnect(self):
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
        self.disconnect()
    
    def interactive_mode(self):
        print(f"{Fore.CYAN}=== Tello无人机控制系统 ==={Style.RESET_ALL}")
        print("输入 'help' 查看可用命令，输入 'quit' 退出")
        
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
            if command == 'help':
                self._show_help()
            
            elif command == 'status':
                self._show_status()
            
            elif command == 'takeoff':
                if self.controller:
                    self.controller.takeoff()
            
            elif command == 'land':
                if self.controller:
                    self.controller.land()
            
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
                self._handle_recording(action)
            
            elif command == 'photo':
                self._take_photo()
            
            elif command == 'media':
                self._show_media_info()
            
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
    
    def _rotate_drone(self, direction, angle):
        if not self.controller:
            return
        
        if direction == 'cw':
            self.controller.rotate_clockwise(angle)
        elif direction == 'ccw':
            self.controller.rotate_counter_clockwise(angle)
    
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
        else:
            print(f"{Fore.YELLOW}请先启动视频流 (stream start){Style.RESET_ALL}")
    
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
  quit/exit           - 退出程序

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
  photo               - 拍照
  media               - 显示媒体文件统计

{Fore.YELLOW}示例:{Style.RESET_ALL}
  takeoff             - 起飞
  up 100              - 上升100cm
  cw 180              - 顺时针旋转180度
  photo               - 拍照
  land                - 降落
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