"""
力矩计算模块 - 计算各轴力矩（Roll, Pitch, Yaw）
"""

import numpy as np
from typing import Dict, Tuple
from config import TricopterConfig
from geometry import tilt_thrust_vector


def calculate_torque(position: np.ndarray, 
                     thrust_vector: np.ndarray, 
                     thrust: float) -> np.ndarray:
    """
    计算单个电机产生的力矩
    
    参数:
        position: 电机位置向量 [x, y, z]
        thrust_vector: 推力方向向量（归一化）
        thrust: 推力大小
    
    返回:
        力矩向量 [τ_roll, τ_pitch, τ_yaw]
    """
    # 力矩 = r × F
    # r: 位置向量
    # F: 推力向量 = thrust * thrust_vector
    
    force = thrust * thrust_vector
    torque = np.cross(position, force)
    
    return torque


def calculate_motor_torques(config: TricopterConfig,
                            tilt_angles: Dict[str, float],
                            thrusts: Dict[str, float] = None) -> Dict[str, Dict]:
    """
    计算所有电机的力矩
    
    参数:
        config: 三旋翼配置
        tilt_angles: 各电机倾转角度（度）
        thrusts: 各电机推力（归一化），如果为None则使用1.0
    
    返回:
        每个电机的力矩信息
    """
    if thrusts is None:
        thrusts = {motor_id: 1.0 for motor_id in ['FL', 'FR', 'R']}
    
    results = {}
    
    for motor_id in ['FL', 'FR', 'R']:
        tilt_deg = tilt_angles.get(motor_id, 0.0)
        tilt_rad = np.deg2rad(tilt_deg)
        thrust = thrusts.get(motor_id, 1.0)
        
        # 获取电机位置
        position = config.get_motor_position(motor_id)
        
        # 计算推力向量
        thrust_vec = tilt_thrust_vector(tilt_rad)
        
        # 计算力矩
        torque = calculate_torque(position, thrust_vec, thrust)
        
        results[motor_id] = {
            'position': position.tolist(),
            'tilt_angle_deg': tilt_deg,
            'thrust': thrust,
            'torque_roll': torque[0],   # Roll力矩（绕X轴）
            'torque_pitch': torque[1],   # Pitch力矩（绕Y轴）
            'torque_yaw': torque[2]      # Yaw力矩（绕Z轴）
        }
    
    return results


def calculate_total_torques(motor_torques: Dict[str, Dict]) -> Dict[str, float]:
    """
    计算总力矩（所有电机之和）
    
    参数:
        motor_torques: 所有电机的力矩信息
    
    返回:
        总力矩信息
    """
    total_roll = 0.0
    total_pitch = 0.0
    total_yaw = 0.0
    
    for motor_id, data in motor_torques.items():
        # 每个共轴对有两个电机，所以乘以2
        total_roll += 2 * data['torque_roll']
        total_pitch += 2 * data['torque_pitch']
        total_yaw += 2 * data['torque_yaw']
    
    return {
        'total_roll': total_roll,
        'total_pitch': total_pitch,
        'total_yaw': total_yaw,
        'total_torque_magnitude': np.sqrt(total_roll**2 + total_pitch**2 + total_yaw**2)
    }


def calculate_torque_from_position_and_force(position: np.ndarray,
                                           force: np.ndarray) -> np.ndarray:
    """
    从位置和力直接计算力矩（用于控制分配矩阵）
    
    参数:
        position: 电机位置向量 [x, y, z]
        force: 力向量 [Fx, Fy, Fz]
    
    返回:
        力矩向量 [τ_roll, τ_pitch, τ_yaw]
    """
    return np.cross(position, force)

