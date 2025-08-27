"""
降落实验运行器 - 适配RMTT系统

严格按照原始launch文件参数进行实验运行
"""

import sys
import time
import logging
import numpy as np
from pathlib import Path
from typing import Dict, Any, Optional
from enum import Enum

# 添加RMTT项目路径
sys.path.append(str(Path(__file__).parent.parent))

from core.connection import ConnectionManager
from utils.logger import Logger
from data.flight_data_recorder import FlightDataRecorder

from data_structures import DesiredState, CurrentState, ControlOutput, R
from pid_controller import PID_Controller
from ude_controller import UDE_Controller  
from adrc_controller import ADRC_Controller


class ControllerType(Enum):
    """控制器类型枚举 - 对应原始launch文件"""
    PID = 0    # PID控制器
    UDE = 1    # UDE控制器
    ADRC = 2   # AMESO-based ADRC控制器


class ExperimentRunner:
    """实验运行器 - 完整实现降落实验"""
    
    def __init__(self):
        self.logger = Logger("HX_Landing_Experiment")
        
        # RMTT系统组件
        self.connection_manager = None
        self.data_recorder = None
        self.tello = None
        
        # 控制器
        self.controller = None
        self.controller_type = ControllerType.ADRC  # 默认使用ADRC
        
        # 实验参数 - 修改为简化降落流程
        self.experiment_params = {
            # 飞行轨迹参数 - 简化为起飞到1.0m，然后直接降落
            "takeoff_height": 1.0,      # 起飞高度1.0m
            "takeoff_speed": 0.2,       # 起飞速度0.2m/s  
            "descend_speed": 0.1,       # 1.0m→地面直接降落速度0.1m/s
            
            # 控制器参数 - 严格对应launch文件第15-50行
            "ameso_gain/quad_mass": 0.087,  # RMTT实际质量87g
            "ameso_gain/hov_percent": 0.5,
            
            # 滑模控制参数
            "ameso_gain/k": 0.8,
            "ameso_gain/k1": -0.15,
            "ameso_gain/k2": -3.0,
            "ameso_gain/c1": 1.5,
            "ameso_gain/c2": 0.6,
            "ameso_gain/lambda_D": 1.0,
            "ameso_gain/beta_max": 1.0,
            "ameso_gain/gamma": 0.2,
            
            # 自适应模型参数
            "ameso_gain/lambda": 0.8,
            "ameso_gain/sigma": 0.9,
            "ameso_gain/omega_star": 0.02,
            
            # 跟踪微分器参数
            "ameso_gain/t1": 0.02,
            "ameso_gain/t2": 0.04,
            
            # AMESO观测器参数
            "ameso_gain/l": 5.0,
            
            # PID参数
            "ameso_gain/kp": 2.0,
            "ameso_gain/ki": 0.3,
            "ameso_gain/kd": 2.0,
            "ameso_gain/int_max_xy": 0.5,
            "ameso_gain/int_max_z": 0.5,
            
            # 安全限制参数
            "ameso_gain/pxy_int_max": 0.5,
            "ameso_gain/pz_int_max": 0.5,
        }
        
        # 添加PID和UDE特有参数
        self.experiment_params.update({
            # PID控制器参数
            "pid_gain/quad_mass": 0.087,   # RMTT实际质量87g
            "pid_gain/hov_percent": 0.5,
            "pid_gain/Kp_xy": 2.0,
            "pid_gain/Kp_z": 2.0,
            "pid_gain/Kv_xy": 2.0,
            "pid_gain/Kv_z": 2.0,
            "pid_gain/Kvi_xy": 0.3,
            "pid_gain/Kvi_z": 0.3,
            "pid_gain/tilt_angle_max": 10.0,
            "pid_gain/pxy_int_max": 0.5,
            "pid_gain/pz_int_max": 0.5,
            
            # UDE控制器参数
            "ude_gain/quad_mass": 0.087,   # RMTT实际质量87g
            "ude_gain/hov_percent": 0.5,
            "ude_gain/Kp_xy": 0.5,
            "ude_gain/Kp_z": 0.5,
            "ude_gain/Kd_xy": 2.0,
            "ude_gain/Kd_z": 2.0,
            "ude_gain/T_ude": 1.0,
            "ude_gain/tilt_angle_max": 20.0,
            "ude_gain/pxy_int_max": 1.0,
            "ude_gain/pz_int_max": 1.0,
        })
        
        # 实验状态
        self.experiment_running = False
        self.start_time = None
        
    def initialize_system(self, controller_type: ControllerType = ControllerType.ADRC) -> bool:
        """初始化实验系统"""
        try:
            self.logger.info("初始化HX降落实验系统...")
            self.controller_type = controller_type
            
            # 初始化RMTT连接
            self.connection_manager = ConnectionManager()
            self.connection_manager.connect()
            self.tello = self.connection_manager.get_tello()
            
            # 初始化数据记录器
            self.data_recorder = FlightDataRecorder(self.connection_manager)
            
            # 创建控制器
            self._create_controller()
            
            self.logger.info("系统初始化完成")
            return True
            
        except Exception as e:
            self.logger.error(f"系统初始化失败: {e}")
            return False
    
    def _create_controller(self):
        """创建指定类型的控制器"""
        if self.controller_type == ControllerType.PID:
            self.controller = PID_Controller()
            self.controller.init(self.experiment_params)
            self.logger.info("创建PID控制器")
            
        elif self.controller_type == ControllerType.UDE:
            self.controller = UDE_Controller()
            self.controller.init(self.experiment_params)
            self.logger.info("创建UDE控制器")
            
        elif self.controller_type == ControllerType.ADRC:
            self.controller = ADRC_Controller()
            self.controller.init(self.experiment_params)
            self.logger.info("创建AMESO-based ADRC控制器")
            
        else:
            raise ValueError(f"不支持的控制器类型: {self.controller_type}")
    
    def run_landing_experiment(self) -> bool:
        """运行完整降落实验 - 严格按照原始实验设计"""
        try:
            self.logger.info("开始HX降落实验")
            self.experiment_running = True
            self.start_time = time.time()
            
            # 开始数据记录
            self.data_recorder.start_recording(f"hx_landing_{self.controller_type.name.lower()}")
            
            # 实验阶段1: 起飞到指定高度
            if not self._phase_takeoff():
                return False
                
            # 实验阶段2: 直接降落
            if not self._phase_landing():
                return False
            
            self.logger.info("降落实验成功完成")
            return True
            
        except Exception as e:
            self.logger.error(f"实验过程异常: {e}")
            return False
        finally:
            self._cleanup()
    
    def _phase_takeoff(self) -> bool:
        """起飞阶段"""
        self.logger.info("阶段1: 起飞到目标高度")
        
        try:
            # 发送起飞指令
            self.tello.takeoff()
            time.sleep(3)  # 等待起飞稳定
            
            # 设置起飞目标状态
            target_height = self.experiment_params["takeoff_height"]  # 1.0m
            takeoff_speed = self.experiment_params["takeoff_speed"]    # 0.2m/s
            
            desired_state = DesiredState(
                pos=np.array([0.0, 0.0, target_height]),
                vel=np.array([0.0, 0.0, 0.0]),
                acc=np.array([0.0, 0.0, 0.0]),
                yaw=0.0
            )
            
            # 控制器稳定到目标高度
            return self._controlled_flight_to_target(desired_state, max_time=15.0, tolerance=0.1)
            
        except Exception as e:
            self.logger.error(f"起飞阶段失败: {e}")
            return False
    
    def _phase_landing(self) -> bool:
        """直接降落阶段"""
        self.logger.info("阶段2: 直接降落 (0.1m/s速度)")
        
        try:
            # 从1.0m直接降落到地面
            desired_state = DesiredState(
                pos=np.array([0.0, 0.0, 0.1]),  # 目标到10cm低空
                vel=np.array([0.0, 0.0, -self.experiment_params["descend_speed"]]),  # 0.1m/s下降
                acc=np.array([0.0, 0.0, 0.0]),
                yaw=0.0
            )
            
            # 控制降落到低空 (从1.0m到0.1m需要9秒)
            if not self._controlled_flight_to_target(desired_state, max_time=12.0, tolerance=0.05):
                self.logger.warning("控制降落未完全到达目标高度，继续执行着陆")
            
            # 发送着陆指令
            self.logger.info("执行最终着陆指令")
            self.tello.land()
            time.sleep(3)
            
            self.logger.info("直接降落完成")
            return True
            
        except Exception as e:
            self.logger.error(f"直接降落失败: {e}")
            return False
    
    def _controlled_flight_to_target(self, desired_state: DesiredState, max_time: float = 10.0, tolerance: float = 0.1) -> bool:
        """使用控制器控制飞行到目标状态"""
        start_time = time.time()
        control_dt = 0.02  # 50Hz控制频率
        
        while time.time() - start_time < max_time:
            try:
                # 获取当前状态
                current_state = self._get_current_state()
                
                # 设置控制器状态
                self.controller.set_desired_state(desired_state)
                self.controller.set_current_state(current_state)
                
                # 计算控制指令
                control_output = self.controller.update(control_dt)
                
                # 发送控制指令 - 所有控制器都返回numpy数组 [roll, pitch, yaw, thrust]
                roll_cmd = int(np.clip(control_output[0] * 180/np.pi * 2.5, -100, 100))
                pitch_cmd = int(np.clip(control_output[1] * 180/np.pi * 2.5, -100, 100))
                yaw_cmd = int(np.clip(control_output[2] * 180/np.pi * 2.5, -100, 100))
                throttle_cmd = int(np.clip((control_output[3] - 0.5) * 200, -100, 100))
                
                self.tello.send_rc_control(roll_cmd, pitch_cmd, throttle_cmd, yaw_cmd)
                
                # 检查是否到达目标
                pos_error = np.linalg.norm(current_state.pos - desired_state.pos)
                if pos_error < tolerance:
                    self.logger.info(f"到达目标位置，误差: {pos_error:.3f}m")
                    return True
                
                # 打印控制状态（每2秒一次）
                if int(time.time() - start_time) % 2 == 0:
                    self.controller.printf_result()
                
                time.sleep(control_dt)
                
            except Exception as e:
                self.logger.error(f"控制循环异常: {e}")
                time.sleep(control_dt)
                continue
        
        self.logger.warning(f"控制超时，未能到达目标位置")
        return False
    
    def _get_current_state(self) -> CurrentState:
        """获取当前无人机状态"""
        # 从RMTT获取状态数据
        height_cm = self.tello.get_height() or 0
        tof_cm = self.tello.get_distance_tof() or 0
        barometer_cm = self.tello.get_barometer() or 0
        
        # 优先使用TOF数据，如果无效则使用高度数据
        current_height_m = (tof_cm if tof_cm > 0 else height_cm) / 100.0
        
        # 获取姿态数据
        roll_deg = self.tello.get_roll() or 0
        pitch_deg = self.tello.get_pitch() or 0
        yaw_deg = self.tello.get_yaw() or 0
        
        # 转换为弧度
        roll_rad = roll_deg * np.pi / 180.0
        pitch_rad = pitch_deg * np.pi / 180.0
        yaw_rad = yaw_deg * np.pi / 180.0
        
        # 构造当前状态（简化版本，实际位置采用相对坐标）
        current_state = CurrentState(
            pos=np.array([0.0, 0.0, current_height_m]),  # 假设无人机在原点上方
            vel=np.array([0.0, 0.0, 0.0]),               # 简化速度估计
            acc=np.array([0.0, 0.0, 0.0]),               # 简化加速度
            yaw=yaw_rad,
            q=R.from_euler('xyz', [roll_rad, pitch_rad, yaw_rad])
        )
        
        return current_state
    
    def _cleanup(self):
        """清理资源"""
        try:
            self.experiment_running = False
            
            if self.data_recorder:
                self.data_recorder.stop_recording()
            
            if self.connection_manager:
                self.connection_manager.disconnect()
            
            self.logger.info("实验资源清理完成")
        except Exception as e:
            self.logger.error(f"资源清理失败: {e}")
    
    def run_controller_comparison(self):
        """运行三种控制器对比实验"""
        controllers = [ControllerType.PID, ControllerType.UDE, ControllerType.ADRC]
        results = {}
        
        for controller_type in controllers:
            self.logger.info(f"开始测试 {controller_type.name} 控制器")
            
            # 重新初始化系统
            if self.initialize_system(controller_type):
                success = self.run_landing_experiment()
                results[controller_type.name] = "成功" if success else "失败"
            else:
                results[controller_type.name] = "初始化失败"
            
            # 等待系统稳定
            time.sleep(5)
        
        # 打印对比结果
        self.logger.info("控制器对比实验结果:")
        for controller, result in results.items():
            self.logger.info(f"  {controller}: {result}")
        
        return results


def main():
    """主函数"""
    print("HX实验1: 降落控制器实验")
    print("=" * 50)
    print("0. 运行ADRC控制器实验")
    print("1. 运行PID控制器实验") 
    print("2. 运行UDE控制器实验")
    print("3. 运行三种控制器对比实验")
    print("=" * 50)
    
    try:
        choice = input("请选择实验类型 (0-3): ").strip()
        
        runner = ExperimentRunner()
        
        if choice == "0":
            if runner.initialize_system(ControllerType.ADRC):
                runner.run_landing_experiment()
        elif choice == "1":
            if runner.initialize_system(ControllerType.PID):
                runner.run_landing_experiment()
        elif choice == "2":
            if runner.initialize_system(ControllerType.UDE):
                runner.run_landing_experiment()
        elif choice == "3":
            runner.run_controller_comparison()
        else:
            print("无效选择，运行默认ADRC实验")
            if runner.initialize_system(ControllerType.ADRC):
                runner.run_landing_experiment()
                
    except KeyboardInterrupt:
        print("\n用户中断实验")
    except Exception as e:
        print(f"实验异常: {e}")


if __name__ == "__main__":
    main()