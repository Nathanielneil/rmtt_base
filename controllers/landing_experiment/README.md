# RMTT降落实验控制器

这是从C++/ROS代码完整移植到Python的三个降落实验控制器，保持了所有原始参数和控制逻辑。

## 控制器类型

### 1. PID控制器
传统的PID位置控制器，实现了：
- 位置误差的比例、积分、微分控制
- 积分项防饱和处理
- 安全限制和故障保护

### 2. UDE控制器  
不确定性和扰动估计器控制器，实现了：
- 标称控制律 + 扰动补偿
- UDE扰动估计算法
- 滤波器参数可调

### 3. ADRC控制器
基于AMESO论文的自抗扰控制器，实现了：
- 跟踪微分器(TD)
- 自适应模型扩展状态观测器(AMESO)
- 可变指数滑模控制
- 9个基函数的自适应模型
- Z轴使用ADRC，XY轴使用PID

## 文件结构

```
landing_experiment/
├── __init__.py                 # 模块初始化
├── landing_state.py            # 状态数据结构定义
├── pid_controller.py           # PID控制器
├── ude_controller.py           # UDE控制器
├── adrc_controller.py          # ADRC控制器
├── rmtt_adapter.py             # RMTT系统适配器
├── landing_experiment.py       # 主实验程序
└── README.md                   # 本文档
```

## 参数配置

所有参数完全对应原始launch文件配置：

### 基本参数
- `quad_mass`: 0.087 (RMTT实际质量87g)
- `hov_percent`: 0.5 (悬停油门百分比)

### ADRC参数 (来自论文表1)
- `k`: 0.8 (滑模增益)
- `k1`: -0.15, `k2`: -3.0 (反馈增益)
- `c1`: 1.5, `c2`: 0.6 (滑模面参数)
- `lambda_D`: 1.0 (积分参数)
- `beta_max`: 1.0, `gamma`: 0.2 (指数参数)
- `lambda`: 0.8, `sigma`: 0.9 (自适应参数)
- `omega_star`: 0.02 (阈值参数)
- `t1`: 0.02, `t2`: 0.04 (跟踪微分器)
- `l`: 5.0 (ESO增益)

## 实验流程

完全对应原始实验设置：

1. **起飞阶段**: 上升到1.0m高度
2. **第一段下降**: 1.0m → 0.7m (速度0.3m/s)
3. **第二段下降**: 0.7m → 0.5m (速度0.2m/s)  
4. **最终降落**: 0.5m → 地面 (速度0.1m/s)

每个阶段都有悬停稳定时间。

## 使用方法

### 基本使用

```bash
cd /home/daniel/NGW/github_projects/rmtt
python -m controllers.landing_experiment.landing_experiment
```

### 程序化使用

```python
from controllers.landing_experiment import LandingExperiment, ControllerType

# 创建实验实例
experiment = LandingExperiment(ControllerType.ADRC)

# 执行实验
success = experiment.execute_landing_experiment()
```

### 单独使用控制器

```python
from controllers.landing_experiment import RMTTAdapter, ControllerType
import numpy as np

# 创建适配器
adapter = RMTTAdapter(ControllerType.ADRC)

# 初始化控制器
params = {
    "quad_mass": 0.087,  # RMTT实际质量
    "hov_percent": 0.5,
    # ... 其他参数
}
adapter.init_controller(params)

# 使用控制器
tello_state = get_tello_state()  # 获取传感器数据
current_state = adapter.rmtt_to_current_state(tello_state)

desired_state = adapter.create_desired_state(
    target_pos=np.array([0, 0, 1.0]),  # 目标位置
    target_vel=np.zeros(3),            # 目标速度
    target_yaw=0.0                     # 目标偏航
)

control_output = adapter.update_control(current_state, desired_state, dt=0.02)
roll_rc, pitch_rc, throttle_rc, yaw_rc = adapter.control_output_to_tello_rc(control_output)
```

## 技术特点

### 1. 完全保持原始算法
- 逐行对照C++源码实现
- 保持所有原始参数和数值
- 保持原始的控制逻辑和安全检查

### 2. RMTT系统集成
- 双重定高传感器融合 (TOF + 气压计)
- 50Hz控制频率
- 与RMTT数据记录系统集成
- 完整的安全保护机制

### 3. 模块化设计
- 控制器可独立使用
- 清晰的接口设计
- 易于扩展和修改

## 安全注意事项

1. **实验环境**: 确保在空旷安全的环境中进行实验
2. **电池电量**: 确保电池电量充足
3. **紧急停止**: 程序支持Ctrl+C紧急停止
4. **参数谨慎**: 不要随意修改控制参数
5. **备用方案**: 准备手动遥控器作为备用

## 故障排除

### 常见问题

1. **控制器无响应**
   - 检查传感器数据是否正常
   - 确认控制器已正确初始化

2. **飞行不稳定**
   - 检查质量参数是否正确
   - 验证传感器标定

3. **高度控制精度不足**
   - 检查TOF传感器是否工作正常
   - 调整控制器增益参数

### 调试功能

```python
# 打印控制器状态
adapter.print_debug_info()

# 获取控制器状态
status = adapter.get_controller_status()
print(status)
```

## 实验数据

实验过程中的所有数据会自动记录到RMTT数据记录系统中，包括：
- 控制器输出
- 传感器数据
- 飞行轨迹
- 控制性能指标

## 开发说明

如需修改或扩展控制器：

1. 修改控制参数：在`control_params`字典中调整
2. 扩展控制器：继承`BaseController`类
3. 修改实验流程：修改`LandingExperiment`类中的方法
4. 添加新传感器：扩展`rmtt_to_current_state`方法

## 技术支持

- 控制器算法基于AMESO论文实现
- C++源码完全移植，保持算法一致性
- 如有问题请检查日志输出和原始论文