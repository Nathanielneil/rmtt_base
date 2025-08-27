"""
HX实验1: 降落控制器实验

基于原始C++/ROS控制器实现的Python版本，包含三种控制器：
1. PID控制器 - 经典PID位置控制
2. UDE控制器 - 未知扰动估计器控制
3. ADRC控制器 - 基于AMESO的自抗扰控制器

完全保持原始控制器结构和参数，适配RMTT系统进行实物降落实验
"""

from .data_structures import DesiredState, CurrentState, ControlOutput, sign, sat
from .pid_controller import PID_Controller
from .ude_controller import UDE_Controller  
from .adrc_controller import ADRC_Controller
from .experiment_runner import ExperimentRunner, ControllerType

__all__ = [
    'DesiredState', 'CurrentState', 'ControlOutput', 'sign', 'sat',
    'PID_Controller', 'UDE_Controller', 'ADRC_Controller',
    'ExperimentRunner', 'ControllerType'
]