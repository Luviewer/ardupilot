"""
几何计算模块 - 计算电机倾转后的推力向量分解
"""

import numpy as np
from typing import Dict, Tuple
from config import TricopterConfig


def tilt_thrust_vector(tilt_angle_rad: float) -> np.ndarray:
    """
    计算倾转后的推力向量（在机体坐标系中）
    
    参数:
        tilt_angle_rad: 倾转角度（弧度），绕Y轴旋转，向前倾转为正
    
    返回:
        推力向量 [Fx, Fy, Fz]，其中：
        - Fx: X方向分量（Forward）
        - Fy: Y方向分量（Right，倾转绕Y轴，所以为0）
        - Fz: Z方向分量（Down，升力）
    """
    sin_theta = np.sin(tilt_angle_rad)
    cos_theta = np.cos(tilt_angle_rad)
    
    # 推力向量：F = T · [sin(θ), 0, -cos(θ)]
    return np.array([sin_theta, 0.0, -cos_theta])


def decompose_thrust(thrust: float, tilt_angle_rad: float) -> Dict[str, float]:
    """
    分解推力为X和Z方向分量
    
    参数:
        thrust: 推力大小
        tilt_angle_rad: 倾转角度（弧度）
    
    返回:
        包含Fx和Fz的字典
    """
    thrust_vec = tilt_thrust_vector(tilt_angle_rad)
    return {
        'Fx': thrust * thrust_vec[0],  # X方向分量
        'Fy': thrust * thrust_vec[1],  # Y方向分量（应该为0）
        'Fz': thrust * thrust_vec[2]   # Z方向分量（升力）
    }


def calculate_all_motor_thrusts(config: TricopterConfig, 
                                tilt_angles: Dict[str, float],
                                thrusts: Dict[str, float] = None) -> Dict[str, Dict]:
    """
    计算所有电机的推力分解
    
    参数:
        config: 三旋翼配置
        tilt_angles: 各电机倾转角度（度）字典，如 {'FL': 45.0, 'FR': 45.0, 'R': 0.0}
        thrusts: 各电机推力（归一化，0-1），如果为None则使用1.0
    
    返回:
        每个电机的推力分解信息
    """
    if thrusts is None:
        thrusts = {motor_id: 1.0 for motor_id in ['FL', 'FR', 'R']}
    
    results = {}
    
    for motor_id in ['FL', 'FR', 'R']:
        tilt_deg = tilt_angles.get(motor_id, 0.0)
        tilt_rad = np.deg2rad(tilt_deg)
        thrust = thrusts.get(motor_id, 1.0)
        
        # 计算推力分解
        thrust_vec = tilt_thrust_vector(tilt_rad)
        decomposed = decompose_thrust(thrust, tilt_rad)
        
        # 获取电机位置
        position = config.get_motor_position(motor_id)
        
        results[motor_id] = {
            'position': position.tolist(),
            'tilt_angle_deg': tilt_deg,
            'tilt_angle_rad': tilt_rad,
            'thrust': thrust,
            'thrust_vector': thrust_vec.tolist(),
            'Fx': decomposed['Fx'],
            'Fy': decomposed['Fy'],
            'Fz': decomposed['Fz'],
            'lift_ratio': -decomposed['Fz'] / thrust if thrust > 0 else 0.0  # 升力比例
        }
    
    return results


def calculate_total_forces(motor_thrusts: Dict[str, Dict]) -> Dict[str, float]:
    """
    计算总力和力矩
    
    参数:
        motor_thrusts: 所有电机的推力分解信息
    
    返回:
        总力和力矩信息
    """
    total_Fx = 0.0
    total_Fy = 0.0
    total_Fz = 0.0
    
    for motor_id, data in motor_thrusts.items():
        # 每个共轴对有两个电机，所以乘以2
        total_Fx += 2 * data['Fx']
        total_Fy += 2 * data['Fy']
        total_Fz += 2 * data['Fz']
    
    return {
        'total_Fx': total_Fx,
        'total_Fy': total_Fy,
        'total_Fz': total_Fz,
        'total_lift': -total_Fz,  # 升力（向上为正）
        'total_thrust_magnitude': np.sqrt(total_Fx**2 + total_Fy**2 + total_Fz**2)
    }

