# 实验脚本使用说明

## 高度控制实验

### 文件说明

- `altitude_experiment.py` - 主实验脚本
- `experiment_config.py` - 实验参数配置文件
- `README.md` - 使用说明（本文件）

### 实验设计

实验设计为多周期的高度控制测试：

1. **起飞阶段**: 无人机起飞到指定高度（默认1.5m）
2. **悬停阶段**: 在目标高度悬停指定时间（默认3秒）
3. **下降阶段**: 以恒定速度缓慢下降到最终高度（默认0.1m）
4. **周期重复**: 重复上述过程指定次数

### 快速开始

1. **修改实验参数**（可选）
   ```bash
   # 编辑配置文件
   nano experiment_config.py
   
   # 主要参数：
   EXPERIMENT_CYCLES = 3      # 实验周期数量
   TARGET_HEIGHT = 150        # 目标高度(cm)
   DESCENT_SPEED = 0.1        # 下降速度(m/s)
   ```

2. **验证参数配置**
   ```bash
   python experiment_config.py
   ```

3. **运行实验**
   ```bash
   python altitude_experiment.py
   ```

### 配置参数说明

#### 基础参数
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `EXPERIMENT_CYCLES` | 3 | 实验周期数量 |
| `TARGET_HEIGHT` | 150cm | 目标悬停高度 |
| `HOVER_DURATION` | 3秒 | 悬停持续时间 |
| `DESCENT_SPEED` | 0.1m/s | 下降速度 |
| `FINAL_HEIGHT` | 10cm | 最终下降高度 |
| `CYCLE_REST_TIME` | 1秒 | 周期间隔时间（已优化） |

#### 高级参数
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `STEP_DISTANCE` | 20cm | 分步下降距离 |
| `HEIGHT_TOLERANCE` | 10cm | 高度调整容差 |
| `STABILIZATION_TIME` | 1秒 | 移动后稳定时间（已优化） |

#### 安全参数
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `MIN_BATTERY_LEVEL` | 25% | 最低电池要求 |
| `MAX_EXPERIMENT_TIME` | 30分钟 | 最大实验时间 |

### 实验输出

#### 数据记录
- **飞行数据**: 自动保存到 `../data/flight_records/` 目录
- **格式**: CSV文件，包含时间戳、高度、姿态、速度、加速度等数据
- **频率**: 50Hz高频采样
- **数据列说明**:
  - `timestamp`: 绝对时间戳 (ISO格式)
  - `relative_time`: 相对实验开始的时间(秒)，便于数据分析
  - `height_cm`: 无人机高度(厘米) - 核心实验数据
  - `battery_percent`: 电池电量百分比
  - `temperature_deg`: 温度(摄氏度)
  - `pitch_deg`, `roll_deg`, `yaw_deg`: 姿态角度(度)
  - `tof_distance_cm`: TOF距离传感器读数(厘米)
  - `barometer_cm`: 气压计高度(厘米)
  - `vgx_cm_s`, `vgy_cm_s`, `vgz_cm_s`: 三轴速度分量(厘米/秒)
  - `agx_0001g`, `agy_0001g`, `agz_0001g`: 三轴加速度分量(0.001g单位)

#### 日志文件  
- **程序日志**: 保存到 `../logs/` 目录
- **实验报告**: 自动生成实验总结报告

#### 控制台输出
- 实时显示实验进度
- 每个阶段的执行状态
- 错误和警告信息

### 实验流程

```
开始 → 参数验证 → 系统初始化 → 连接无人机 → 检查电池
  ↓
起飞 → 数据记录开始 → 执行实验周期 → 最终降落
  ↓
停止数据记录 → 生成实验报告 → 清理资源 → 结束
```

### 安全注意事项

1. **环境要求**
   - 确保在空旷、无障碍的室内或室外环境
   - 避免强风或恶劣天气条件

2. **设备检查**
   - 确认无人机电池充足（≥25%）
   - 检查螺旋桨是否牢固
   - 确认WiFi连接稳定

3. **实验监控**
   - 实验过程中请保持监控
   - 如有异常可按Ctrl+C安全中止
   - 准备手动遥控器作为备份

4. **数据备份**
   - 重要实验数据建议及时备份
   - 可设置多个较短的实验周期而非单次长时间实验

### 故障排除

#### 常见问题
1. **连接失败**
   - 检查WiFi连接到无人机热点
   - 确认无人机已开机并处于WiFi模式

2. **高度控制不准确**
   - 检查环境中是否有气流干扰
   - 调整`HEIGHT_TOLERANCE`参数
   - 增加`STABILIZATION_TIME`

3. **实验中断**
   - 检查电池电量
   - 查看日志文件了解详细错误信息
   - 确认无人机在安全飞行范围内

#### 参数调优建议
- **首次使用**: 建议使用默认参数
- **精确控制**: 可减小`STEP_DISTANCE`和`HEIGHT_TOLERANCE`
- **快速测试**: 可减少`EXPERIMENT_CYCLES`和`HOVER_DURATION`
- **电池续航优化**: 
  - 减少`EXPERIMENT_CYCLES`（如改为2个周期）
  - 缩短`HOVER_DURATION`（如改为2秒）
  - 已优化`CYCLE_REST_TIME`为1秒
  - 已优化`STABILIZATION_TIME`为1秒

### 数据分析

实验完成后，可使用以下方式分析数据：

```python
import pandas as pd
import matplotlib.pyplot as plt

# 读取实验数据
data = pd.read_csv('path_to_flight_record.csv')

# 分析高度变化
plt.figure(figsize=(12, 8))

# 子图1: 高度变化
plt.subplot(2, 1, 1)
plt.plot(data['relative_time'], data['height_cm'], 'b-', linewidth=2)
plt.xlabel('时间 (秒)')
plt.ylabel('高度 (cm)')
plt.title('RoboMaster TT高度控制实验 - 高度变化')
plt.grid(True)

# 子图2: 速度分析
plt.subplot(2, 1, 2)
plt.plot(data['relative_time'], data['vgz_cm_s'], 'r-', linewidth=2)
plt.xlabel('时间 (秒)')
plt.ylabel('垂直速度 (cm/s)')
plt.title('垂直速度变化')
plt.grid(True)

plt.tight_layout()
plt.show()

# 分析电池消耗
plt.figure(figsize=(10, 6))
plt.plot(data['relative_time'], data['battery_percent'], 'g-', linewidth=2)
plt.xlabel('时间 (秒)')
plt.ylabel('电池电量 (%)')
plt.title('实验过程电池消耗')
plt.grid(True)
plt.show()
```