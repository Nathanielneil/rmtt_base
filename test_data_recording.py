#!/usr/bin/env python3
"""
数据记录功能测试脚本
测试修复后的FlightDataRecorder是否能正确采集和记录所有数据列
"""

import sys
import time
from pathlib import Path

# 添加项目根目录到路径
sys.path.append(str(Path(__file__).parent))

from core.connection import ConnectionManager
from data.flight_data_recorder import FlightDataRecorder
from utils.logger import Logger

def test_data_recording():
    """测试数据记录功能"""
    logger = Logger("DataRecordingTest")
    
    try:
        print("=" * 60)
        print("RoboMaster TT 数据记录功能测试")
        print("=" * 60)
        
        # 初始化连接管理器
        logger.info("初始化连接管理器...")
        connection_manager = ConnectionManager()
        
        # 连接到无人机
        logger.info("连接到RoboMaster TT...")
        connection_manager.connect()
        
        # 创建数据记录器
        logger.info("创建数据记录器...")
        data_recorder = FlightDataRecorder(connection_manager)
        
        # 开始记录
        logger.info("开始数据记录测试 (记录10秒)...")
        success = data_recorder.start_recording("recording_test")
        
        if not success:
            logger.error("数据记录启动失败")
            return False
        
        # 记录10秒钟的数据
        test_duration = 10
        logger.info(f"记录 {test_duration} 秒数据...")
        
        for i in range(test_duration):
            time.sleep(1)
            status = data_recorder.get_recording_status()
            print(f"记录进度: {i+1}/{test_duration}秒, 数据点: {status.get('data_points', 0)}")
        
        # 停止记录
        logger.info("停止数据记录...")
        data_recorder.stop_recording()
        
        # 获取最终状态
        final_status = data_recorder.get_recording_status()
        
        print("\n" + "=" * 60)
        print("测试结果:")
        print(f"- 记录文件: {final_status.get('file_path', 'N/A')}")
        print(f"- 数据点数量: {final_status.get('data_points', 0)}")
        print(f"- 记录时长: {final_status.get('duration', 0):.1f}秒")
        print(f"- 采样频率: {final_status.get('data_points', 0) / max(final_status.get('duration', 1), 1):.1f} Hz")
        
        # 验证CSV文件内容
        csv_path = final_status.get('file_path')
        if csv_path and Path(csv_path).exists():
            logger.info("验证CSV文件内容...")
            with open(csv_path, 'r', encoding='utf-8-sig') as f:
                lines = f.readlines()
                
            print(f"- CSV行数: {len(lines)} (包括头部)")
            if len(lines) > 1:
                headers = lines[0].strip().split(',')
                print(f"- 数据列数: {len(headers)}")
                print(f"- CSV头部: {', '.join(headers[:5])}...")
                
                # 检查第一行数据
                if len(lines) > 1:
                    first_data_row = lines[1].strip().split(',')
                    print(f"- 第一行数据: {', '.join(first_data_row[:5])}...")
                    
                    # 检查是否有空数据列
                    empty_columns = [i for i, val in enumerate(first_data_row) if val.strip() == '' or val.strip() == 'None']
                    if empty_columns:
                        print(f"⚠️  发现空数据列: {empty_columns}")
                    else:
                        print("✅ 所有数据列都有值")
        
        print("=" * 60)
        
        # 断开连接
        connection_manager.disconnect()
        
        logger.info("数据记录测试完成")
        return True
        
    except Exception as e:
        logger.error(f"测试失败: {e}")
        return False

if __name__ == "__main__":
    success = test_data_recording()
    if success:
        print("\n✅ 数据记录测试通过!")
    else:
        print("\n❌ 数据记录测试失败!")
        sys.exit(1)