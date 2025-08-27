# HX实验1: 降落控制器实验

## 概述

本实验是基于原始C++/ROS降落实验的完整Python移植版本，严格保持了所有原始控制器的结构、参数和算法细节。实验包含三种控制器的对比研究：

1. **PID控制器** - 经典PID位置控制器
2. **UDE控制器** - 未知扰动估计器控制器  
3. **ADRC控制器** - 基于AMESO论文的自抗扰控制器

## 文件结构

```
hx_exp_1_landing/
├── data_structures.py      # 数据结构定义（完全对应C++结构）
├── pid_controller.py       # PID控制器（严格移植）
├── ude_controller.py       # UDE控制器（严格移植）
├── adrc_controller.py      # ADRC控制器（严格移植）
├── experiment_runner.py    # 实验运行器（RMTT系统适配）
├── __init__.py            # 模块初始化
└── README.md              # 本文档
```

## 原始参数对照

### 基本参数（所有控制器共用）
- `quad_mass`: 2.5kg - 无人机质量
- `hov_percent`: 0.5 - 悬停油门百分比

### PID控制器参数
```python
# 位置控制增益
"pid_gain/Kp_xy": 2.0,    # X,Y轴比例增益
"pid_gain/Kp_z": 2.0,     # Z轴比例增益

# 速度控制增益  
"pid_gain/Kv_xy": 2.0,    # X,Y轴速度增益
"pid_gain/Kv_z": 2.0,     # Z轴速度增益

# 积分增益
"pid_gain/Kvi_xy": 0.3,   # X,Y轴积分增益
"pid_gain/Kvi_z": 0.3,    # Z轴积分增益
```

### UDE控制器参数  
```python
# 控制增益
"ude_gain/Kp_xy": 0.5,    # X,Y轴比例增益
"ude_gain/Kp_z": 0.5,     # Z轴比例增益
"ude_gain/Kd_xy": 2.0,    # X,Y轴微分增益
"ude_gain/Kd_z": 2.0,     # Z轴微分增益

# UDE特有参数
"ude_gain/T_ude": 1.0,    # 滤波器时间常数
```

### ADRC控制器参数（基于AMESO论文）
```python
# 滑模控制参数
"ameso_gain/k": 0.8,         # 滑模增益
"ameso_gain/k1": -0.15,      # 反馈增益k1
"ameso_gain/k2": -3.0,       # 反馈增益k2  
"ameso_gain/c1": 1.5,        # 滑模面参数c1
"ameso_gain/c2": 0.6,        # 滑模面参数c2
"ameso_gain/lambda_D": 1.0,  # 积分参数

# 自适应模型参数
"ameso_gain/lambda": 0.8,    # 自适应学习率
"ameso_gain/sigma": 0.9,     # 收缩因子
"ameso_gain/omega_star": 0.02, # 阈值参数

# 跟踪微分器参数
"ameso_gain/t1": 0.02,       # TD时间常数1
"ameso_gain/t2": 0.04,       # TD时间常数2

# AMESO观测器参数
"ameso_gain/l": 5.0,         # ESO增益参数
```

## 实验流程

### 降落实验设计
严格按照原始launch文件设计的实验流程：

1. **起飞阶段**: 起飞到1.0m高度
2. **分段降落**: 从1.0m缓慢降落到0.5m  
3. **最终着陆**: 从0.5m降落到地面

### 速度参数
- 起飞速度: 0.2m/s
- 第一段下降速度: 0.3m/s (1.6m→1.0m)
- 第二段下降速度: 0.2m/s (1.0m→0.5m)  
- 最终降落速度: 0.1m/s (0.5m→地面)

## 使用方法

### 单独运行某种控制器
```python
from hx_exp_1_landing import ExperimentRunner, ControllerType

# 创建实验运行器
runner = ExperimentRunner()

# 运行ADRC控制器实验
if runner.initialize_system(ControllerType.ADRC):
    runner.run_landing_experiment()
```

### 运行对比实验
```python
# 运行三种控制器对比实验
runner = ExperimentRunner()
results = runner.run_controller_comparison()
```

### 直接运行
```bash
cd /home/daniel/NGW/github_projects/rmtt
python hx_exp_1_landing/experiment_runner.py
```

## 移植质量保证

### 严格对应原始代码
1. **成员变量**: 严格按照.h头文件中定义的所有成员变量进行移植
2. **算法逻辑**: 逐行对应C++代码的控制算法实现
3. **参数设置**: 完全保持launch文件中的所有参数值
4. **数学公式**: 保持所有PID、UDE、ADRC算法的数学表达式

### 关键移植细节
- Eigen::Vector3d → numpy.ndarray
- Eigen::cwiseProduct → numpy逐元素乘法
- C++的sat/sign函数完全复制
- ROS参数服务器 → Python字典参数
- 所有安全检查和边界条件保持不变

### RMTT系统适配
- 传感器数据接口适配
- RC控制指令转换
- 高度数据融合（TOF优先）
- 实时控制频率保持50Hz

## 注意事项

1. **安全第一**: 在空旷安全区域进行实验
2. **参数调试**: 建议先在仿真环境中验证
3. **数据记录**: 系统自动记录所有飞行数据用于分析
4. **异常处理**: 具备完善的异常检测和安全降落机制

## 实验结果分析

实验系统会自动记录：
- 控制器输出指令
- 无人机状态数据  
- 控制误差统计
- 飞行轨迹数据

可用于后续的控制性能分析和算法优化。