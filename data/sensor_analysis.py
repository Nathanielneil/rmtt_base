#!/usr/bin/env python3
"""
RoboMaster TT传感器数据分析工具
分析红外定高、气压计定高、ESP32数据质量等
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import argparse

def analyze_height_sensors(csv_file):
    """分析红外TOF和气压计高度传感器数据"""
    try:
        # 读取CSV数据
        data = pd.read_csv(csv_file)
        
        print(f"分析文件: {csv_file}")
        print(f"数据点数量: {len(data)}")
        print(f"数据采集时长: {data['relative_time'].max():.1f}秒")
        
        # 基本统计信息
        print("\n=== 高度传感器对比分析 ===")
        
        # 主高度数据（融合后）
        main_height = data['height_cm'].dropna()
        print(f"主高度数据 - 范围: {main_height.min():.1f}~{main_height.max():.1f}cm, 平均: {main_height.mean():.1f}cm")
        
        # TOF红外距离传感器（近距离精确）
        tof_data = data['tof_distance_cm'].dropna()
        if len(tof_data) > 0:
            print(f"TOF红外传感器 - 范围: {tof_data.min():.1f}~{tof_data.max():.1f}cm, 平均: {tof_data.mean():.1f}cm")
            print(f"TOF有效数据点: {len(tof_data)}/{len(data)} ({len(tof_data)/len(data)*100:.1f}%)")
        
        # 气压计高度（绝对高度）
        baro_data = data['barometer_cm'].dropna()
        if len(baro_data) > 0:
            print(f"气压计传感器 - 范围: {baro_data.min():.1f}~{baro_data.max():.1f}cm, 平均: {baro_data.mean():.1f}cm")
            print(f"气压计有效数据点: {len(baro_data)}/{len(data)} ({len(baro_data)/len(data)*100:.1f}%)")
        
        # 高度差分析（传感器一致性）
        if 'height_diff_cm' in data.columns:
            height_diff = data['height_diff_cm'].dropna()
            if len(height_diff) > 0:
                print(f"传感器高度差 - 平均: {height_diff.mean():.1f}cm, 最大: {height_diff.max():.1f}cm")
        
        return True
        
    except Exception as e:
        print(f"数据分析失败: {e}")
        return False

def plot_sensor_comparison(csv_file, output_dir=None):
    """绘制传感器对比图表"""
    try:
        data = pd.read_csv(csv_file)
        
        # 创建多子图
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle('RoboMaster TT ESP32传感器数据分析', fontsize=16)
        
        # 子图1: 高度传感器对比
        ax1 = axes[0, 0]
        if 'height_cm' in data.columns:
            ax1.plot(data['relative_time'], data['height_cm'], 'b-', linewidth=2, label='主高度')
        if 'tof_distance_cm' in data.columns:
            ax1.plot(data['relative_time'], data['tof_distance_cm'], 'r--', alpha=0.7, label='TOF红外')
        if 'barometer_cm' in data.columns:
            ax1.plot(data['relative_time'], data['barometer_cm'], 'g:', alpha=0.7, label='气压计')
        
        ax1.set_xlabel('时间 (秒)')
        ax1.set_ylabel('高度 (cm)')
        ax1.set_title('高度传感器对比')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # 子图2: 传感器高度差
        ax2 = axes[0, 1]
        if 'height_diff_cm' in data.columns:
            ax2.plot(data['relative_time'], data['height_diff_cm'], 'purple', linewidth=1.5)
            ax2.fill_between(data['relative_time'], 0, data['height_diff_cm'], alpha=0.3, color='purple')
        
        ax2.set_xlabel('时间 (秒)')
        ax2.set_ylabel('高度差 (cm)')
        ax2.set_title('TOF与气压计高度差')
        ax2.grid(True, alpha=0.3)
        
        # 子图3: 三轴速度
        ax3 = axes[1, 0]
        if 'vgx_cm_s' in data.columns and 'vgy_cm_s' in data.columns and 'vgz_cm_s' in data.columns:
            ax3.plot(data['relative_time'], data['vgx_cm_s'], 'r-', alpha=0.7, label='X轴速度')
            ax3.plot(data['relative_time'], data['vgy_cm_s'], 'g-', alpha=0.7, label='Y轴速度') 
            ax3.plot(data['relative_time'], data['vgz_cm_s'], 'b-', linewidth=2, label='Z轴速度')
        
        ax3.set_xlabel('时间 (秒)')
        ax3.set_ylabel('速度 (cm/s)')
        ax3.set_title('ESP32三轴速度数据')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # 子图4: WiFi信号质量和电池
        ax4 = axes[1, 1]
        ax4_twin = ax4.twinx()
        
        # 电池电量
        if 'battery_percent' in data.columns:
            line1 = ax4.plot(data['relative_time'], data['battery_percent'], 'orange', linewidth=2, label='电池电量')
            ax4.set_ylabel('电池电量 (%)', color='orange')
        
        # WiFi信号强度
        if 'wifi_snr' in data.columns:
            wifi_data = data['wifi_snr'][data['wifi_snr'] > -1]  # 过滤无效值
            if len(wifi_data) > 0:
                line2 = ax4_twin.plot(data['relative_time'], data['wifi_snr'], 'cyan', alpha=0.7, label='WiFi信号')
                ax4_twin.set_ylabel('WiFi SNR', color='cyan')
        
        ax4.set_xlabel('时间 (秒)')
        ax4.set_title('ESP32系统状态')
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # 保存图表
        if output_dir:
            output_path = Path(output_dir)
            output_path.mkdir(parents=True, exist_ok=True)
            
            timestamp = Path(csv_file).stem.split('_')[0]  # 从文件名提取时间戳
            plot_filename = output_path / f"sensor_analysis_{timestamp}.png"
            plt.savefig(plot_filename, dpi=150, bbox_inches='tight')
            print(f"图表已保存到: {plot_filename}")
        
        plt.show()
        return True
        
    except Exception as e:
        print(f"绘图失败: {e}")
        return False

def analyze_esp32_performance(csv_file):
    """分析ESP32性能和WiFi连接质量"""
    try:
        data = pd.read_csv(csv_file)
        
        print("\n=== ESP32-D2WD性能分析 ===")
        
        # 数据采集性能
        total_time = data['relative_time'].max()
        total_points = len(data)
        actual_frequency = total_points / total_time if total_time > 0 else 0
        
        print(f"数据采集频率: {actual_frequency:.1f} Hz (目标: 50 Hz)")
        if actual_frequency < 45:
            print("⚠️  采集频率低于预期，可能存在性能问题")
        else:
            print("✅ 采集频率正常")
        
        # WiFi连接质量
        if 'wifi_snr' in data.columns:
            valid_wifi = data['wifi_snr'][data['wifi_snr'] > -1]
            if len(valid_wifi) > 0:
                avg_snr = valid_wifi.mean()
                print(f"WiFi信号质量: 平均SNR = {avg_snr:.1f}")
                if avg_snr > 20:
                    print("✅ WiFi信号强")
                elif avg_snr > 10:
                    print("⚠️  WiFi信号中等")
                else:
                    print("❌ WiFi信号弱")
        
        # 传感器数据完整性
        sensors = ['height_cm', 'tof_distance_cm', 'barometer_cm', 'vgx_cm_s', 'vgy_cm_s', 'vgz_cm_s']
        print("\n传感器数据完整性:")
        for sensor in sensors:
            if sensor in data.columns:
                valid_count = data[sensor].dropna().count()
                completeness = valid_count / len(data) * 100
                status = "✅" if completeness > 95 else "⚠️ " if completeness > 80 else "❌"
                print(f"  {sensor}: {completeness:.1f}% {status}")
        
        return True
        
    except Exception as e:
        print(f"ESP32性能分析失败: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description='RoboMaster TT传感器数据分析工具')
    parser.add_argument('csv_file', help='CSV数据文件路径')
    parser.add_argument('--plot', action='store_true', help='生成图表')
    parser.add_argument('--output', help='图表输出目录')
    
    args = parser.parse_args()
    
    csv_path = Path(args.csv_file)
    if not csv_path.exists():
        print(f"错误: 文件不存在 {csv_path}")
        return
    
    print("RoboMaster TT ESP32-D2WD传感器数据分析")
    print("=" * 60)
    
    # 基本分析
    analyze_height_sensors(csv_path)
    analyze_esp32_performance(csv_path)
    
    # 生成图表
    if args.plot:
        plot_sensor_comparison(csv_path, args.output)

if __name__ == "__main__":
    main()