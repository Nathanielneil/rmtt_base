"""
降落实验主程序

实现与原始launch文件相同的降落实验轨迹和参数
完全对应原始实验设置
"""

import sys
import time
import numpy as np
import logging
from pathlib import Path

# 添加RMTT项目路径
sys.path.append(str(Path(__file__).parent.parent.parent))

from core.connection import ConnectionManager
from utils.logger import Logger
from data.flight_data_recorder import FlightDataRecorder

from .rmtt_adapter import RMTTAdapter, ControllerType
from .landing_state import DesiredState


class LandingExperiment:
    """降落实验类 - 完全对应原始实验设置"""
    
    def __init__(self, controller_type: ControllerType = ControllerType.ADRC):
        self.logger = Logger("LandingExperiment")
        self.controller_type = controller_type
        
        # RMTT基础组件
        self.connection_manager = ConnectionManager()
        self.data_recorder = FlightDataRecorder(self.connection_manager)
        
        # 控制器适配器
        self.adapter = RMTTAdapter(controller_type)
        
        # 飞行轨迹参数 - 完全对应launch文件设置
        self.takeoff_height = 1.0      # 起飞高度1.0m - 安全修正
        self.takeoff_speed = 0.2       # 起飞速度0.2m/s - 安全起飞
        self.descend_speed_1 = 0.3     # 1.6m→1.0m下降速度
        self.descend_speed_2 = 0.2     # 1.0m→0.5m下降速度
        self.land_speed = 0.1          # 0.5m→地面降落速度
        
        # 控制参数 - 完全对应launch文件参数
        self.control_params = {
            "quad_mass": 0.087,         # RMTT实际质量87g
            "hov_percent": 0.5,         # 悬停油门百分比
            
            # 滑模控制参数，来自论文表1
            "k": 0.8,                   # 滑模增益
            "k1": -0.15,                # 反馈增益k1
            "k2": -3.0,                 # 反馈增益k2
            "c1": 1.5,                  # 滑模面参数c1
            "c2": 0.6,                  # 滑模面参数c2
            "lambda_D": 1.0,            # 积分参数
            "beta_max": 1.0,            # 最大指数值
            "gamma": 0.2,               # 指数参数
            
            # 自适应模型参数，来自论文表1
            "lambda": 0.8,              # 自适应学习率
            "sigma": 0.9,               # 收缩因子
            "omega_star": 0.02,         # 阈值参数
            
            # 跟踪微分器参数，来自论文表1
            "t1": 0.02,                 # TD时间常数1
            "t2": 0.04,                 # TD时间常数2
            
            # AMESO观测器参数
            "l": 5.0,                   # ESO增益参数
            
            # PID参数
            "kp": 2.0,                  # PID增益参数
            "ki": 0.3,                  # PID增益参数
            "kd": 2.0,                  # PID增益参数
            
            # 安全限制参数
            "pxy_int_max": 0.5,         # XY轴积分限制
            "pz_int_max": 0.5           # Z轴积分限制
        }
        
        # 实验状态
        self.experiment_running = False
        self.control_dt = 0.02  # 50Hz控制频率
        
        self.logger.info(f"Landing Experiment initialized with {controller_type.name} controller")
    
    def initialize_system(self) -> bool:
        """初始化实验系统"""
        try:
            # 连接无人机
            self.logger.info("连接到Tello无人机...")
            self.connection_manager.connect()
            
            # 初始化控制器
            self.logger.info(f"初始化{self.controller_type.name}控制器...")
            self.adapter.init_controller(self.control_params)
            
            # 启动数据记录
            self.data_recorder.start_recording(f"landing_experiment_{self.controller_type.name.lower()}")
            
            self.logger.info("实验系统初始化完成")
            return True
            
        except Exception as e:
            self.logger.error(f"系统初始化失败: {e}")
            return False
    
    def get_tello_state(self) -> dict:
        """获取Tello状态数据"""
        tello = self.connection_manager.get_tello()
        
        return {
            'height_cm': tello.get_height() or 0,
            'tof_distance_cm': tello.get_distance_tof() or 0,
            'barometer_cm': tello.get_barometer() or 0,
            'roll_deg': tello.get_roll() or 0,
            'pitch_deg': tello.get_pitch() or 0,
            'yaw_deg': tello.get_yaw() or 0,
            'vgx_cm_s': 0,  # 简化版本，实际需要从状态获取
            'vgy_cm_s': 0,
            'vgz_cm_s': 0,
            'battery_percent': tello.get_battery() or 0,
            'temperature_deg': tello.get_temperature() or 20
        }
    
    def send_control_command(self, roll_rc: int, pitch_rc: int, throttle_rc: int, yaw_rc: int):
        """发送控制指令到Tello"""
        try:
            tello = self.connection_manager.get_tello()
            tello.send_rc_control(roll_rc, pitch_rc, throttle_rc, yaw_rc)
        except Exception as e:
            self.logger.error(f"发送控制指令失败: {e}")
    
    def execute_landing_experiment(self) -> bool:
        """执行降落实验 - 完全对应原始实验流程"""
        
        if not self.initialize_system():
            return False
        
        try:
            self.experiment_running = True
            tello = self.connection_manager.get_tello()
            
            # ==========================================
            # 阶段1：起飞到目标高度
            # ==========================================
            self.logger.info("阶段1：起飞...")
            tello.takeoff()
            time.sleep(3)  # 等待起飞稳定
            
            # 设置起飞目标高度
            target_height = self.takeoff_height
            self.logger.info(f"上升到目标高度 {target_height}m")
            
            self._hover_at_height(target_height, duration=5.0)
            
            # ==========================================
            # 阶段2：第一段下降 (1.0m → 0.7m)
            # ==========================================
            self.logger.info("阶段2：第一段下降...")
            target_height = 0.7
            self._descend_to_height(target_height, self.descend_speed_1)
            self._hover_at_height(target_height, duration=3.0)
            
            # ==========================================
            # 阶段3：第二段下降 (0.7m → 0.5m)  
            # ==========================================
            self.logger.info("阶段3：第二段下降...")
            target_height = 0.5
            self._descend_to_height(target_height, self.descend_speed_2)
            self._hover_at_height(target_height, duration=3.0)
            
            # ==========================================
            # 阶段4：最终降落 (0.5m → 地面)
            # ==========================================
            self.logger.info("阶段4：最终降落...")
            target_height = 0.1
            self._descend_to_height(target_height, self.land_speed)
            
            # 执行降落
            self.logger.info("执行自动降落...")
            tello.land()
            time.sleep(5)
            
            self.logger.info("降落实验成功完成！")
            return True
            
        except Exception as e:
            self.logger.error(f"实验执行失败: {e}")
            return False
        finally:
            self.experiment_running = False
            self._cleanup()
    
    def _hover_at_height(self, target_height: float, duration: float):
        """在指定高度悬停"""
        self.logger.info(f"在{target_height}m高度悬停{duration}秒")
        
        start_time = time.time()
        last_control_time = start_time
        
        while time.time() - start_time < duration and self.experiment_running:
            current_time = time.time()
            dt = current_time - last_control_time
            
            if dt >= self.control_dt:  # 50Hz控制频率
                # 获取当前状态
                tello_state = self.get_tello_state()
                current_state = self.adapter.rmtt_to_current_state(tello_state)
                
                # 创建期望状态 - 悬停
                desired_state = self.adapter.create_desired_state(
                    target_pos=np.array([0.0, 0.0, target_height]),
                    target_vel=np.zeros(3),
                    target_acc=np.zeros(3),
                    target_yaw=0.0
                )
                
                # 计算控制输出
                control_output = self.adapter.update_control(current_state, desired_state, dt)
                
                # 发送控制指令
                roll_rc, pitch_rc, throttle_rc, yaw_rc = self.adapter.control_output_to_tello_rc(control_output)
                self.send_control_command(roll_rc, pitch_rc, throttle_rc, yaw_rc)
                
                # 记录调试信息（每1秒一次）
                if int(current_time - start_time) % 1 == 0:
                    self.logger.info(f"高度: {current_state.pos[2]:.3f}m, 目标: {target_height}m")
                
                last_control_time = current_time
            
            time.sleep(0.005)  # 短暂休眠
    
    def _descend_to_height(self, target_height: float, descent_speed: float):
        """下降到指定高度"""
        self.logger.info(f"以{descent_speed}m/s速度下降到{target_height}m")
        
        # 获取当前高度
        tello_state = self.get_tello_state()
        current_state = self.adapter.rmtt_to_current_state(tello_state)
        start_height = current_state.pos[2]
        
        self.logger.info(f"当前高度: {start_height:.3f}m，目标高度: {target_height}m")
        
        if start_height <= target_height:
            self.logger.info("已在目标高度或更低，跳过下降")
            return
        
        start_time = time.time()
        last_control_time = start_time
        
        while self.experiment_running:
            current_time = time.time()
            dt = current_time - last_control_time
            
            if dt >= self.control_dt:  # 50Hz控制频率
                # 获取当前状态
                tello_state = self.get_tello_state()
                current_state = self.adapter.rmtt_to_current_state(tello_state)
                current_height = current_state.pos[2]
                
                # 检查是否到达目标高度
                if current_height <= target_height + 0.05:  # 5cm容差
                    self.logger.info(f"到达目标高度 {target_height}m")
                    break
                
                # 计算期望高度（线性下降）
                elapsed_time = current_time - start_time
                expected_height = start_height - descent_speed * elapsed_time
                expected_height = max(expected_height, target_height)
                
                # 创建期望状态
                desired_state = self.adapter.create_desired_state(
                    target_pos=np.array([0.0, 0.0, expected_height]),
                    target_vel=np.array([0.0, 0.0, -descent_speed]),
                    target_acc=np.zeros(3),
                    target_yaw=0.0
                )
                
                # 计算控制输出
                control_output = self.adapter.update_control(current_state, desired_state, dt)
                
                # 发送控制指令
                roll_rc, pitch_rc, throttle_rc, yaw_rc = self.adapter.control_output_to_tello_rc(control_output)
                self.send_control_command(roll_rc, pitch_rc, throttle_rc, yaw_rc)
                
                # 记录调试信息（每1秒一次）
                if int(elapsed_time) % 1 == 0:
                    self.logger.info(f"当前高度: {current_height:.3f}m, 期望高度: {expected_height:.3f}m")
                
                last_control_time = current_time
            
            time.sleep(0.005)  # 短暂休眠
            
            # 安全超时
            if time.time() - start_time > 30.0:
                self.logger.warning("下降超时，停止下降")
                break
    
    def _cleanup(self):
        """清理资源"""
        try:
            # 停止数据记录
            self.data_recorder.stop_recording()
            
            # 断开连接
            self.connection_manager.disconnect()
            
            self.logger.info("资源清理完成")
        except Exception as e:
            self.logger.error(f"资源清理失败: {e}")
    
    def emergency_stop(self):
        """紧急停止"""
        self.logger.warning("执行紧急停止")
        self.experiment_running = False
        try:
            tello = self.connection_manager.get_tello()
            tello.emergency()
        except:
            pass


def main():
    """主函数"""
    print("RMTT降落实验控制器")
    print("=" * 50)
    print("1. PID控制器")
    print("2. UDE控制器")  
    print("3. ADRC控制器")
    print("=" * 50)
    
    try:
        choice = input("请选择控制器类型 (1/2/3): ").strip()
        
        if choice == "1":
            controller_type = ControllerType.PID
        elif choice == "2":
            controller_type = ControllerType.UDE
        elif choice == "3":
            controller_type = ControllerType.ADRC
        else:
            print("无效选择，使用默认ADRC控制器")
            controller_type = ControllerType.ADRC
        
        # 创建实验实例
        experiment = LandingExperiment(controller_type)
        
        print(f"\n开始执行降落实验，使用{controller_type.name}控制器...")
        print("注意：确保在安全的飞行环境中执行实验！")
        
        # 等待用户确认
        input("按回车键开始实验...")
        
        # 执行实验
        success = experiment.execute_landing_experiment()
        
        if success:
            print("\n实验成功完成！")
        else:
            print("\n实验执行失败！")
            
    except KeyboardInterrupt:
        print("\n用户中断实验")
        if 'experiment' in locals():
            experiment.emergency_stop()
    except Exception as e:
        print(f"\n实验异常终止: {e}")
        if 'experiment' in locals():
            experiment.emergency_stop()


if __name__ == "__main__":
    main()