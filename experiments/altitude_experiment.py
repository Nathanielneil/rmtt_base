#!/usr/bin/env python3
"""
高度控制实验脚本
实验设计：无人机起飞至1.5m高度，悬停3秒，然后以0.1m/s速度缓慢降落到0.1m高度
这个过程为一个试验周期，可设置多个周期进行重复实验
"""

import sys
import time
import math
from datetime import datetime
from pathlib import Path

# 添加项目根目录到路径
sys.path.append(str(Path(__file__).parent.parent))
sys.path.append(str(Path(__file__).parent))

from core.connection import ConnectionManager
from core.tello_controller import TelloController
from data.flight_data_recorder import FlightDataRecorder
from utils.logger import Logger

# 导入实验配置
from experiment_config import *


class AltitudeExperiment:
    def __init__(self):
        self.logger = Logger("AltitudeExperiment")
        self.connection_manager = ConnectionManager()
        self.controller = None
        self.data_recorder = None
        self.experiment_start_time = None
        self.current_cycle = 0
        
    def initialize_systems(self):
        """初始化所有系统"""
        try:
            # 验证实验参数
            self.logger.info("验证实验参数...")
            config_errors = validate_parameters()
            if config_errors:
                for error in config_errors:
                    self.logger.error(f"参数配置错误: {error}")
                return False
            
            self.logger.info("初始化实验系统...")
            
            # 连接无人机
            self.logger.info("连接到Tello无人机...")
            self.connection_manager.connect()
            
            # 初始化控制器和数据记录器
            self.controller = TelloController(self.connection_manager)
            self.data_recorder = FlightDataRecorder(self.connection_manager)
            
            # 获取初始状态
            initial_battery = self.controller.get_status()
            if initial_battery:
                battery_level = initial_battery.get('battery', 0)
                self.logger.info(f"无人机电池电量: {battery_level}%")
                
                if battery_level < MIN_BATTERY_LEVEL:
                    self.logger.warning(f"电池电量过低 ({battery_level}%)，需要 {MIN_BATTERY_LEVEL}% 以上")
                    return False
            
            self.logger.info("系统初始化完成")
            return True
            
        except Exception as e:
            self.logger.error(f"系统初始化失败: {e}")
            return False
    
    def start_experiment(self):
        """开始实验"""
        try:
            self.experiment_start_time = datetime.now()
            experiment_session_name = f"{EXPERIMENT_NAME}_{self.experiment_start_time.strftime('%Y%m%d_%H%M%S')}"
            
            self.logger.info("="*60)
            self.logger.info("开始高度控制实验")
            self.logger.info(f"实验名称: {EXPERIMENT_NAME}")
            self.logger.info(f"实验周期: {EXPERIMENT_CYCLES}次")
            self.logger.info(f"实验开始时间: {self.experiment_start_time}")
            self.logger.info("="*60)
            
            # 开始数据记录
            self.logger.info("启动数据记录系统...")
            if not self.data_recorder.start_recording(experiment_session_name):
                self.logger.error("数据记录启动失败")
                return False
            
            # 起飞
            self.logger.info("执行起飞...")
            self.controller.takeoff()
            time.sleep(3)  # 等待稳定
            
            # 执行实验周期
            for cycle in range(1, EXPERIMENT_CYCLES + 1):
                self.current_cycle = cycle
                self.logger.info(f"\n--- 开始第 {cycle}/{EXPERIMENT_CYCLES} 个实验周期 ---")
                
                success = self.execute_single_cycle(cycle)
                if not success:
                    self.logger.error(f"第 {cycle} 个周期执行失败，终止实验")
                    break
                
                # 周期间隔（除了最后一个周期）
                if cycle < EXPERIMENT_CYCLES:
                    self.logger.info(f"周期间隔休息 {CYCLE_REST_TIME} 秒...")
                    time.sleep(CYCLE_REST_TIME)
            
            # 最终降落
            self.logger.info("\n实验周期完成，执行最终降落...")
            self.controller.land()
            time.sleep(2)
            
            self.logger.info("实验执行完成!")
            return True
            
        except Exception as e:
            self.logger.error(f"实验执行失败: {e}")
            return False
    
    def execute_single_cycle(self, cycle_num):
        """执行单个实验周期"""
        try:
            cycle_start_time = time.time()
            
            # 阶段1: 上升到目标高度
            self.logger.info(f"[周期 {cycle_num}] 阶段1: 上升到 {TARGET_HEIGHT}cm...")
            current_height = self.get_current_height()
            
            if current_height is not None:
                height_diff = TARGET_HEIGHT - current_height
                if abs(height_diff) > HEIGHT_TOLERANCE:  # 如果高度差超过容差
                    # 分段移动以避免超过安全限制
                    remaining_distance = int(abs(height_diff))
                    max_single_move = 150  # 单次最大移动距离
                    
                    while remaining_distance > HEIGHT_TOLERANCE:
                        move_distance = min(remaining_distance, max_single_move)
                        
                        if height_diff > 0:
                            self.controller.move_up(move_distance)
                        else:
                            self.controller.move_down(move_distance)
                        
                        remaining_distance -= move_distance
                        time.sleep(STABILIZATION_TIME)  # 等待移动完成
                        
                        # 更新当前高度
                        new_height = self.get_current_height()
                        if new_height is not None:
                            height_diff = TARGET_HEIGHT - new_height
            
            # 阶段2: 悬停
            self.logger.info(f"[周期 {cycle_num}] 阶段2: 在 {TARGET_HEIGHT}cm 悬停 {HOVER_DURATION}秒...")
            self.controller.hover(HOVER_DURATION)
            
            # 阶段3: 缓慢下降
            self.logger.info(f"[周期 {cycle_num}] 阶段3: 以 {DESCENT_SPEED}m/s 下降到 {FINAL_HEIGHT}cm...")
            self.execute_controlled_descent()
            
            cycle_duration = time.time() - cycle_start_time
            self.logger.info(f"[周期 {cycle_num}] 完成，耗时: {cycle_duration:.1f}秒")
            
            return True
            
        except Exception as e:
            self.logger.error(f"执行周期 {cycle_num} 失败: {e}")
            return False
    
    def execute_controlled_descent(self):
        """执行受控下降 - 优化版本，避免分步下降"""
        try:
            # 计算下降参数
            height_diff = TARGET_HEIGHT - FINAL_HEIGHT  # 需要下降的高度 (cm)
            descent_speed_cm_s = DESCENT_SPEED * 100    # 转换为 cm/s
            
            # 计算理论下降时间
            total_descent_time = height_diff / descent_speed_cm_s
            
            self.logger.info(f"优化下降控制: 总距离={height_diff}cm, 速度={descent_speed_cm_s}cm/s ({DESCENT_SPEED}m/s), 预计时间={total_descent_time:.1f}秒")
            
            # 直接下降到目标高度，避免分步停顿
            # 使用Tello的速度控制命令实现连续下降
            try:
                # 方案1: 使用rc命令实现连续下降
                tello = self.connection_manager.get_tello()
                
                self.logger.info(f"开始连续下降 {height_diff}cm...")
                
                # 计算rc命令的下降速度参数 (Tello rc命令范围: -100到100)
                # 负值表示下降，速度映射到-100到0范围
                rc_speed = min(-10, -int(descent_speed_cm_s * 0.5))  # 保守映射，确保不会太快
                
                # 开始连续下降
                start_time = time.time()
                tello.send_rc_control(0, 0, rc_speed, 0)  # left_right, forward_backward, up_down, yaw
                
                # 监控下降过程
                while time.time() - start_time < total_descent_time:
                    current_height = self.get_current_height()
                    if current_height is not None and current_height <= FINAL_HEIGHT + 5:
                        # 接近目标高度，停止下降
                        break
                    time.sleep(0.1)  # 短暂检查间隔
                
                # 停止下降
                tello.send_rc_control(0, 0, 0, 0)
                time.sleep(0.5)  # 等待稳定
                
            except Exception as rc_error:
                self.logger.warning(f"RC连续下降失败，使用备用方案: {rc_error}")
                
                # 方案2: 单次大距离下降
                self.logger.info(f"使用单次下降命令: 下降 {height_diff}cm")
                self.controller.move_down(height_diff)
                
                # 等待下降完成
                time.sleep(total_descent_time + 1)  # 额外1秒缓冲
            
            # 验证最终高度
            final_height = self.get_current_height()
            if final_height is not None:
                height_error = abs(final_height - FINAL_HEIGHT)
                if height_error > HEIGHT_TOLERANCE:
                    self.logger.warning(f"高度偏差较大: 当前{final_height}cm, 目标{FINAL_HEIGHT}cm, 偏差{height_error}cm")
                else:
                    self.logger.info(f"下降完成，当前高度: {final_height}cm (目标: {FINAL_HEIGHT}cm, 偏差: {height_error}cm)")
            
        except Exception as e:
            self.logger.error(f"优化下降控制失败: {e}")
            # 紧急情况，使用传统下降方法
            self.logger.info("使用紧急下降方案...")
            self.controller.move_down(TARGET_HEIGHT - FINAL_HEIGHT)
    
    def get_current_height(self):
        """获取当前高度"""
        try:
            status = self.controller.get_status()
            if status:
                return status.get('height')
            return None
        except:
            return None
    
    def finalize_experiment(self):
        """结束实验并保存数据"""
        try:
            experiment_end_time = datetime.now()
            total_duration = experiment_end_time - self.experiment_start_time
            
            self.logger.info("\n" + "="*60)
            self.logger.info("实验结束")
            self.logger.info(f"结束时间: {experiment_end_time}")
            self.logger.info(f"总实验时长: {str(total_duration).split('.')[0]}")
            self.logger.info(f"完成周期: {self.current_cycle}/{EXPERIMENT_CYCLES}")
            
            # 停止数据记录
            if self.data_recorder:
                self.logger.info("保存实验数据...")
                self.data_recorder.stop_recording()
                
                # 获取记录状态
                status = self.data_recorder.get_recording_status()
                if status.get('file_path'):
                    self.logger.info(f"实验数据已保存至: {status['file_path']}")
                    self.logger.info(f"记录数据点: {status.get('data_points', 0)}")
                    self.logger.info(f"记录时长: {status.get('duration', 0):.1f}秒")
            
            # 生成实验报告
            self.generate_experiment_report(experiment_end_time, total_duration)
            
            self.logger.info("="*60)
            
        except Exception as e:
            self.logger.error(f"实验收尾失败: {e}")
    
    def generate_experiment_report(self, end_time, duration):
        """生成实验报告"""
        try:
            from config.settings import LOGS_DIR
            LOGS_DIR.mkdir(exist_ok=True)
            
            report_filename = f"altitude_experiment_report_{self.experiment_start_time.strftime('%Y%m%d_%H%M%S')}.txt"
            report_path = LOGS_DIR / report_filename
            
            report_content = f"""
高度控制实验报告
===========================================

实验基本信息:
- 实验名称: {EXPERIMENT_NAME}
- 实验描述: {EXPERIMENT_DESCRIPTION.strip()}
- 开始时间: {self.experiment_start_time.strftime('%Y-%m-%d %H:%M:%S')}
- 结束时间: {end_time.strftime('%Y-%m-%d %H:%M:%S')}
- 总时长: {str(duration).split('.')[0]}

实验参数:
- 计划周期数: {EXPERIMENT_CYCLES}
- 实际完成周期数: {self.current_cycle}
- 目标高度: {TARGET_HEIGHT}cm (1.5m)
- 悬停时间: {HOVER_DURATION}秒
- 下降速度: {DESCENT_SPEED}m/s
- 最终高度: {FINAL_HEIGHT}cm (0.1m)
- 周期间隔: {CYCLE_REST_TIME}秒

实验结果:
- 完成率: {(self.current_cycle/EXPERIMENT_CYCLES*100):.1f}%
- 平均周期时长: {(duration.total_seconds()/max(self.current_cycle,1)):.1f}秒

数据文件:
- 飞行数据: 请查看 data/flight_records/ 目录下对应的CSV文件
- 程序日志: 请查看 logs/ 目录下对应的日志文件

===========================================
实验报告生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
"""
            
            with open(report_path, 'w', encoding='utf-8') as f:
                f.write(report_content)
            
            self.logger.info(f"实验报告已生成: {report_path}")
            
        except Exception as e:
            self.logger.error(f"生成实验报告失败: {e}")
    
    def cleanup(self):
        """清理资源"""
        try:
            if self.data_recorder:
                self.data_recorder.stop_recording()
            
            if self.controller and self.controller.in_flight:
                self.logger.info("紧急降落...")
                self.controller.emergency_stop()
            
            if self.connection_manager:
                self.connection_manager.disconnect()
                
        except Exception as e:
            self.logger.error(f"资源清理失败: {e}")


def main():
    """主函数"""
    experiment = AltitudeExperiment()
    
    try:
        print("\n" + "="*80)
        print("高度控制实验程序")
        print("="*80)
        
        # 显示实验配置摘要
        summary = get_experiment_summary()
        print(f"实验配置:")
        print(f"  - 实验周期: {EXPERIMENT_CYCLES} 次")
        print(f"  - 目标高度: {TARGET_HEIGHT}cm ({summary['target_height_m']}m)")
        print(f"  - 最终高度: {FINAL_HEIGHT}cm ({summary['final_height_m']}m)")
        print(f"  - 悬停时间: {HOVER_DURATION}秒")
        print(f"  - 下降速度: {DESCENT_SPEED}m/s")
        print(f"  - 周期间隔: {CYCLE_REST_TIME}秒")
        print(f"  - 预估单周期时间: {summary['estimated_cycle_time']:.1f}秒")
        print(f"  - 预估总时间: {summary['total_estimated_time']:.1f}秒 ({summary['total_estimated_time']/60:.1f}分钟)")
        print("="*80)
        
        # 安全确认
        user_input = input("\n请确认实验参数正确，是否开始实验? (y/N): ")
        if user_input.lower() != 'y':
            print("实验已取消")
            return
        
        # 初始化系统
        if not experiment.initialize_systems():
            print("系统初始化失败，实验终止")
            return
        
        # 开始实验
        success = experiment.start_experiment()
        
        if success:
            print("\n✅ 实验成功完成!")
        else:
            print("\n❌ 实验执行失败!")
            
    except KeyboardInterrupt:
        print("\n\n⚠️  用户中断实验")
        experiment.logger.warning("用户中断实验")
        
    except Exception as e:
        print(f"\n❌ 实验异常终止: {e}")
        experiment.logger.error(f"实验异常终止: {e}")
        
    finally:
        # 实验收尾和资源清理
        experiment.finalize_experiment()
        experiment.cleanup()
        print("\n实验程序已退出")


if __name__ == "__main__":
    main()