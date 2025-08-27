# RMTT - RoboMaster TT创造力套装控制系统

专为RoboMaster TT创造力套装设计的Python控制系统，充分利用ESP32-D2WD主控和双重定高传感器的优势，支持高精度飞行控制、实时数据采集、视频流处理等功能。

## RoboMaster TT硬件特性

### ESP32-D2WD主控优势
- **双核处理器**: 240MHz Xtensa LX6，支持高频数据处理
- **集成WiFi**: 2.4GHz 802.11b/g/n，实时连接质量监控
- **实时计算**: 边缘计算三轴速度和加速度数据

### 双重定高传感器系统
- **红外TOF距离传感器**: 0-8m范围，近距离厘米级精度
- **气压计高度传感器**: 绝对高度参考，不受地面材质影响
- **传感器融合**: 智能切换和数据对比，确保定高精度

## 软件功能特性

- ✅ **精确飞行控制**: 充分利用双重定高系统
- ✅ **高频数据采集**: 50Hz采样，18列完整传感器数据
- ✅ **传感器融合分析**: TOF vs 气压计对比，地面检测
- ✅ **实时视频流**: 显示、录制、拍照功能
- ✅ **专业数据分析**: 传感器性能分析和图表生成
- ✅ **自动化实验**: 高度控制等科研实验脚本
- ✅ **智能安全机制**: 电池监控、连接监控、自动降落
- ✅ **ESP32性能监控**: WiFi信号强度、数据完整性检查
- ✅ **模块化设计**: 便于扩展和二次开发

## 系统要求

- **Python**: 3.11+
- **操作系统**: Windows 10/11, Linux, macOS
- **硬件**: RoboMaster TT创造力套装（ESP32-D2WD主控）
- **网络**: WiFi连接到无人机热点 RMTT-A93874
- **可选**: matplotlib, pandas（用于数据分析）

## 安装说明

### 1. 创建Conda环境

```bash
# 创建并激活环境
conda env create -f environment.yml
conda activate rmtt
```

### 2. 或使用pip安装

```bash
pip install -r requirements.txt
```

### 3. 连接到无人机

1. 打开无人机电源
2. 连接到WiFi热点 `RMTT-A93874`
3. 等待连接稳定

## 使用方法

### 启动程序

```bash
python main.py
```

### 自动连接模式

```bash
python main.py --auto-connect
```

### 基本命令

连接后，可使用以下命令：

#### 飞行控制
- `takeoff` - 起飞
- `land` - 降落
- `emergency` - 紧急停止
- `hover [时长]` - 悬停（默认3秒）

#### 移动控制
- `up [距离]` - 上升（默认50cm）
- `down [距离]` - 下降（默认50cm）
- `left [距离]` - 左移（默认50cm）
- `right [距离]` - 右移（默认50cm）
- `forward [距离]` - 前进（默认50cm）
- `back [距离]` - 后退（默认50cm）
- `cw [角度]` - 顺时针旋转（默认90度）
- `ccw [角度]` - 逆时针旋转（默认90度）
- `flip [方向]` - 翻滚（f/b/l/r，默认前翻）

#### 视频和拍照
- `stream start` - 启动视频流
- `stream stop` - 停止视频流
- `record start` - 开始录制视频
- `record stop` - 停止录制视频
- `photo` - 拍照（需要先启动视频流）

#### 飞行数据记录
- `record_data start` - 开始记录飞行数据（50Hz频率）
- `record_data stop` - 停止记录飞行数据
- `record_data status` - 查看记录状态
- 注：起飞时自动开始记录，降落时自动停止

#### 信息查看
- `status` - 显示无人机状态
- `media` - 显示媒体文件统计
- `help` - 显示帮助信息
- `quit` - 退出程序

## 示例飞行序列

```
Tello> takeoff          # 起飞 (自动开始数据记录)
Tello> stream start     # 启动视频流  
Tello> up 100          # 上升100cm
Tello> cw 360          # 360度旋转
Tello> photo           # 拍照
Tello> record start    # 开始录制视频
Tello> forward 100     # 前进100cm
Tello> back 100        # 后退100cm
Tello> record stop     # 停止录制视频
Tello> land            # 降落 (自动停止数据记录)
```

## 安全机制

系统内置多重安全保护：

- **电池监控**: 电池电量低于20%时警告，低于10%时自动降落
- **连接监控**: WiFi连接断开时自动触发紧急降落
- **飞行限制**: 限制最大移动距离和飞行高度
- **命令验证**: 执行飞行命令前进行安全检查

## 文件目录结构

```
rmtt/
├── main.py                 # 主程序入口
├── requirements.txt        # pip依赖
├── environment.yml         # conda环境配置
├── config/
│   └── settings.py        # 配置参数
├── core/
│   ├── connection.py      # 连接管理
│   └── tello_controller.py # 飞行控制
├── media/
│   ├── video_stream.py    # 视频流处理
│   └── media_saver.py     # 媒体文件管理
├── data/
│   ├── flight_data_recorder.py # 飞行数据记录器 (ESP32优化)
│   ├── sensor_analysis.py      # RoboMaster TT传感器分析工具
│   ├── images/           # 图片保存
│   ├── videos/           # 视频保存
│   └── flight_records/   # 飞行数据CSV文件 (18列传感器数据)
├── utils/
│   ├── logger.py          # 日志系统
│   ├── safety.py          # 安全机制
│   └── exceptions.py      # 异常定义
├── examples/
│   ├── basic_flight.py   # 基础飞行演示
│   ├── video_recording.py # 视频录制演示
│   ├── flight_data_recording.py # 数据记录演示
│   └── data_format_test.py # 数据格式测试
├── experiments/
│   ├── altitude_experiment.py # 高度控制实验 (双重定高优化)
│   ├── experiment_config.py # 实验参数配置
│   └── README.md         # 实验使用说明 (RoboMaster TT特性)
├── docs/
│   └── ROBOMASTER_TT_SPECS.md # RoboMaster TT技术规格文档
├── logs/                  # 日志文件
└── test_data_recording.py # 数据记录功能测试脚本
```

## 配置说明

主要配置参数在 `config/settings.py` 中：

```python
TELLO_IP = "192.168.10.1"                    # 无人机IP
WIFI_SSID = "RMTT-A93874"                    # 无人机热点名称
BATTERY_WARNING_THRESHOLD = 20               # 电池警告阈值
BATTERY_CRITICAL_THRESHOLD = 10              # 电池危急阈值
FLIGHT_ALTITUDE_LIMIT = 500                  # 飞行高度限制(cm)
SPEED_LIMIT = 100                            # 移动距离限制(cm)

# 飞行数据记录配置
ENABLE_FLIGHT_DATA_RECORDING = True          # 启用数据记录
FLIGHT_DATA_RECORDING_INTERVAL = 0.02        # 记录间隔(50Hz)
FLIGHT_DATA_CSV_ENCODING = 'utf-8-sig'       # CSV编码
```

## 飞行数据记录

系统支持高频飞行数据记录，以CSV格式存储无人机的关键飞行参数：

### RoboMaster TT数据字段（18列完整传感器数据）

| 字段名 | 描述 | 单位 | 传感器来源 |
|--------|------|------|----------|
| `timestamp` | 绝对时间戳 | ISO格式 | 系统时钟 |
| `relative_time` | 相对实验开始时间 | 秒 | 计算值 |
| `height_cm` | 主要高度数据 | cm | 融合传感器 |
| `battery_percent` | 电池电量 | % | ESP32监控 |
| `temperature_deg` | 温度 | °C | 内置传感器 |
| `pitch_deg`, `roll_deg`, `yaw_deg` | 姿态角度 | 度 | IMU传感器 |
| `tof_distance_cm` | 红外TOF距离 | cm | 红外定高传感器 |
| `barometer_cm` | 气压计高度 | cm | 气压计定高传感器 |
| `height_diff_cm` | 两传感器高度差 | cm | 计算值 |
| `vgx_cm_s`, `vgy_cm_s`, `vgz_cm_s` | 三轴速度分量 | cm/s | ESP32计算 |
| `agx_0001g`, `agy_0001g`, `agz_0001g` | 三轴加速度分量 | 0.001g | ESP32计算 |
| `wifi_snr` | WiFi信号强度 | dBm | ESP32监控 |

### 使用方式

**自动记录（推荐）：**
- 执行 `takeoff` 时自动开始记录
- 执行 `land` 时自动停止记录
- 文件自动保存到 `data/flight_records/` 目录

**手动控制：**
```
record_data start [session_name]  # 开始记录（可选命名）
record_data status                # 查看记录状态
record_data stop                  # 停止记录
```

**文件格式：**
```
20250827_143025_drone_data.csv           # 默认格式
20250827_143025_session_name_drone_data.csv  # 带会话名
```

### RoboMaster TT数据分析工具

**专业传感器分析工具：**
```bash
# 基本传感器性能分析
python data/sensor_analysis.py your_data.csv

# 生成传感器对比图表
python data/sensor_analysis.py your_data.csv --plot

# 保存分析结果到指定目录
python data/sensor_analysis.py your_data.csv --plot --output ./analysis/
```

**分析功能：**
- ✅ 双重定高传感器对比（TOF vs 气压计）
- ✅ ESP32性能监控（数据采集频率、WiFi质量）
- ✅ 传感器数据完整性检查
- ✅ 自动生成传感器性能报告
- ✅ 可视化图表（高度对比、速度分析、系统状态）

**通用分析工具：**
- **Python**: pandas, matplotlib, numpy
- **MATLAB**: readtable, plot
- **Excel**: 直接导入分析
- **R**: read.csv, ggplot2

## 自动化实验系统

系统提供完整的自动化实验脚本，用于科研和测试：

### 高度控制实验

**实验设计:**
- 无人机起飞至1.5m高度
- 悬停3秒钟
- 以0.1m/s速度缓慢降落到0.1m高度
- 支持多周期重复实验

**快速开始:**
```bash
cd experiments
python altitude_experiment.py
```

**自定义参数:**
```python
# 编辑 experiment_config.py
EXPERIMENT_CYCLES = 5      # 修改实验周期数
TARGET_HEIGHT = 200        # 修改目标高度(cm)
DESCENT_SPEED = 0.05       # 修改下降速度(m/s)
```

**RoboMaster TT实验特性:**
- ✅ **双重定高精度控制**: 充分利用TOF+气压计优势
- ✅ **高频数据采集**: 50Hz采样，18列完整传感器数据
- ✅ **ESP32边缘计算**: 实时速度和加速度计算
- ✅ **传感器融合分析**: TOF与气压计对比验证
- ✅ **精确速度控制**: ±0.01m/s控制精度
- ✅ **智能安全机制**: 电池、连接、高度多重保护
- ✅ **自动实验报告**: 传感器性能和实验数据分析
- ✅ **WiFi质量监控**: ESP32连接稳定性实时监测

**输出文件:**
- **数据文件**: `data/flight_records/YYYYMMDD_HHMMSS_altitude_control_experiment.csv`
- **实验报告**: `logs/altitude_experiment_report_YYYYMMDD_HHMMSS.txt`

更多实验详情请查看 `experiments/README.md`

## 故障排除

### 连接问题
1. 确认已连接到无人机WiFi热点
2. 检查无人机是否开机
3. 尝试重新连接WiFi

### 视频流问题
1. 确保无人机固件是最新版本
2. 重启无人机和程序
3. 检查网络连接质量

### 命令不响应
1. 检查无人机电池电量
2. 确认连接状态正常
3. 查看日志文件获取详细错误信息

## 日志文件

所有操作都会记录在 `logs/` 目录下，日志文件按时间命名：
- 飞行操作日志
- 错误信息记录
- 安全事件记录

## 注意事项

1. **安全第一**: 在空旷区域飞行，远离人群和障碍物
2. **电池管理**: 及时关注电池电量，避免强制降落
3. **网络稳定**: 保持笔记本与无人机的WiFi连接稳定
4. **紧急停止**: 遇到危险情况立即使用 `emergency` 命令
5. **定期维护**: 定期清理媒体文件释放存储空间

## 许可证

MIT License

## 贡献

欢迎提交Issue和Pull Request来改进这个项目。