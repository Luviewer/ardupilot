"""
配置模块 - 定义三旋翼的电机位置、倾转角度等参数
"""

import numpy as np
from typing import Dict, List, Tuple


class TricopterConfig:
    """三旋翼配置类"""
    
    def __init__(self, 
                 front_x: float = 0.3,
                 front_y: float = 0.2,
                 rear_x: float = -0.3,
                 rear_y: float = 0.0):
        """
        初始化三旋翼配置
        
        参数:
            front_x: 前电机X坐标（米），默认0.3
            front_y: 前电机Y坐标绝对值（米），默认0.2
            rear_x: 后电机X坐标（米），默认-0.3
            rear_y: 后电机Y坐标（米），默认0.0
        """
        # 电机位置（机体坐标系，单位：米）
        # 前左共轴对（电机0-1）
        self.motor_positions = {
            'FL': np.array([front_x, -front_y, 0.0]),  # 前左
            'FR': np.array([front_x, front_y, 0.0]),   # 前右
            'R': np.array([rear_x, rear_y, 0.0])      # 后置
        }
        
        # 共轴对配置（每个位置有上下两个电机）
        self.coaxial_pairs = {
            'FL': [0, 1],  # 前左共轴对
            'FR': [2, 3],  # 前右共轴对
            'R': [4, 5]    # 后共轴对
        }
        
        # 倾转角度范围（度）
        self.tilt_angle_range = {
            'min': -90.0,
            'max': 90.0,
            'default': 0.0
        }
        
        # 默认倾转角度（度）
        self.default_tilt_angles = {
            'FL': 0.0,
            'FR': 0.0,
            'R': 0.0
        }
        
        # 分析参数
        self.analysis_params = {
            'tilt_step': 5.0,  # 倾转角度分析步长（度）
            'tilt_range': (-90, 90),  # 倾转角度分析范围
            'singularity_threshold': 0.1,  # 奇异性检测阈值（cos值）
            'condition_warn': 10.0,  # 条件数警告阈值
            'condition_critical': 100.0  # 条件数临界阈值
        }
    
    def get_motor_position(self, motor_id: str) -> np.ndarray:
        """获取电机位置向量"""
        return self.motor_positions[motor_id].copy()
    
    def get_all_motor_positions(self) -> Dict[str, np.ndarray]:
        """获取所有电机位置"""
        return self.motor_positions.copy()
    
    def get_coaxial_motors(self, pair_id: str) -> List[int]:
        """获取共轴对的电机索引"""
        return self.coaxial_pairs[pair_id].copy()
    
    def to_dict(self) -> Dict:
        """转换为字典格式（用于JSON输出）"""
        return {
            'motor_positions': {
                k: v.tolist() for k, v in self.motor_positions.items()
            },
            'coaxial_pairs': self.coaxial_pairs,
            'tilt_angle_range': self.tilt_angle_range,
            'default_tilt_angles': self.default_tilt_angles,
            'analysis_params': self.analysis_params
        }
    
    @classmethod
    def from_dict(cls, data: Dict) -> 'TricopterConfig':
        """从字典创建配置对象"""
        config = cls()
        if 'motor_positions' in data:
            for k, v in data['motor_positions'].items():
                config.motor_positions[k] = np.array(v)
        if 'coaxial_pairs' in data:
            config.coaxial_pairs = data['coaxial_pairs']
        if 'tilt_angle_range' in data:
            config.tilt_angle_range = data['tilt_angle_range']
        if 'default_tilt_angles' in data:
            config.default_tilt_angles = data['default_tilt_angles']
        if 'analysis_params' in data:
            config.analysis_params.update(data['analysis_params'])
        return config


# 默认配置实例
default_config = TricopterConfig()

