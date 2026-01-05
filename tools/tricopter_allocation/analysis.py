"""
分析模块 - 不同倾转角度下的矩阵分析、升力损失分析、控制能力分析
"""

import numpy as np
from typing import Dict, List, Tuple
from config import TricopterConfig
from geometry import calculate_all_motor_thrusts, calculate_total_forces
from allocation_matrix import build_allocation_matrix, analyze_matrix, calculate_condition_number


def analyze_tilt_angles(config: TricopterConfig,
                       tilt_angles: Dict[str, float],
                       thrusts: Dict[str, float] = None) -> Dict:
    """
    分析特定倾转角度配置
    
    参数:
        config: 三旋翼配置
        tilt_angles: 各电机倾转角度（度）
        thrusts: 各电机推力（归一化）
    
    返回:
        分析结果字典
    """
    # 计算推力分解
    motor_thrusts = calculate_all_motor_thrusts(config, tilt_angles, thrusts)
    
    # 计算总力
    total_forces = calculate_total_forces(motor_thrusts)
    
    # 构建控制分配矩阵
    matrix = build_allocation_matrix(config, tilt_angles)
    
    # 分析矩阵
    matrix_analysis = analyze_matrix(matrix)
    
    # 计算升力损失
    if thrusts is None:
        thrusts = {m: 1.0 for m in ['FL', 'FR', 'R']}
    total_thrust = sum([thrusts.get(m, 1.0) for m in ['FL', 'FR', 'R']]) * 2  # 每个共轴对2个电机
    lift_loss = 1.0 - (total_forces['total_lift'] / total_thrust) if total_thrust > 0 else 0.0
    
    return {
        'tilt_angles': tilt_angles,
        'thrusts': thrusts if thrusts else {m: 1.0 for m in ['FL', 'FR', 'R']},
        'motor_thrusts': motor_thrusts,
        'total_forces': total_forces,
        'matrix_analysis': matrix_analysis,
        'lift_loss_ratio': lift_loss,
        'lift_efficiency': 1.0 - lift_loss
    }


def sweep_tilt_angles(config: TricopterConfig,
                     tilt_range: Tuple[float, float] = (-90, 90),
                     step: float = 5.0,
                     symmetric: bool = True) -> List[Dict]:
    """
    扫描倾转角度范围，分析不同配置
    
    参数:
        config: 三旋翼配置
        tilt_range: 倾转角度范围（度）
        step: 步长（度）
        symmetric: 是否对称倾转（前左和前右角度相同）
    
    返回:
        分析结果列表
    """
    results = []
    angles = np.arange(tilt_range[0], tilt_range[1] + step, step)
    
    if symmetric:
        # 对称倾转：前左和前右角度相同
        for tilt_front in angles:
            for tilt_rear in angles:
                tilt_angles = {
                    'FL': float(tilt_front),
                    'FR': float(tilt_front),
                    'R': float(tilt_rear)
                }
                result = analyze_tilt_angles(config, tilt_angles)
                results.append(result)
    else:
        # 非对称倾转：前左和前右可以不同
        for tilt_fl in angles:
            for tilt_fr in angles:
                for tilt_rear in angles:
                    tilt_angles = {
                        'FL': float(tilt_fl),
                        'FR': float(tilt_fr),
                        'R': float(tilt_rear)
                    }
                    result = analyze_tilt_angles(config, tilt_angles)
                    results.append(result)
    
    return results


def analyze_condition_number_vs_tilt(config: TricopterConfig,
                                    tilt_range: Tuple[float, float] = (-90, 90),
                                    step: float = 5.0) -> List[Dict]:
    """
    分析条件数随倾转角度的变化
    
    参数:
        config: 三旋翼配置
        tilt_range: 倾转角度范围（度）
        step: 步长（度）
    
    返回:
        条件数分析结果列表
    """
    results = []
    angles = np.arange(tilt_range[0], tilt_range[1] + step, step)
    
    for tilt_front in angles:
        for tilt_rear in angles:
            tilt_angles = {
                'FL': float(tilt_front),
                'FR': float(tilt_front),
                'R': float(tilt_rear)
            }
            
            matrix = build_allocation_matrix(config, tilt_angles)
            cond = calculate_condition_number(matrix)
            
            # 计算升力因子
            lift_factors = [
                2 * np.cos(np.deg2rad(tilt_front)),
                2 * np.cos(np.deg2rad(tilt_front)),
                2 * np.cos(np.deg2rad(tilt_rear))
            ]
            min_lift_factor = min([abs(f) for f in lift_factors])
            
            results.append({
                'tilt_FL': float(tilt_front),
                'tilt_FR': float(tilt_front),
                'tilt_R': float(tilt_rear),
                'condition_number': float(cond) if not np.isinf(cond) else None,
                'min_lift_factor': float(min_lift_factor),
                'is_singular': bool(np.isinf(cond) or min_lift_factor < 0.1)
            })
    
    return results


def find_singularity_points(config: TricopterConfig,
                           tilt_range: Tuple[float, float] = (-90, 90),
                           step: float = 1.0) -> List[Dict]:
    """
    查找奇异性点
    
    参数:
        config: 三旋翼配置
        tilt_range: 倾转角度范围（度）
        step: 步长（度，使用较小步长以提高精度）
    
    返回:
        奇异性点列表
    """
    singularity_points = []
    angles = np.arange(tilt_range[0], tilt_range[1] + step, step)
    
    for tilt_front in angles:
        for tilt_rear in angles:
            tilt_angles = {
                'FL': float(tilt_front),
                'FR': float(tilt_front),
                'R': float(tilt_rear)
            }
            
            matrix = build_allocation_matrix(config, tilt_angles)
            cond = calculate_condition_number(matrix)
            
            # 检查是否奇异
            if np.isinf(cond) or cond > 1000:
                lift_factors = [
                    2 * np.cos(np.deg2rad(tilt_front)),
                    2 * np.cos(np.deg2rad(tilt_front)),
                    2 * np.cos(np.deg2rad(tilt_rear))
                ]
                min_lift_factor = min([abs(f) for f in lift_factors])
                
                singularity_points.append({
                    'tilt_FL': float(tilt_front),
                    'tilt_FR': float(tilt_front),
                    'tilt_R': float(tilt_rear),
                    'condition_number': float(cond) if not np.isinf(cond) else None,
                    'min_lift_factor': float(min_lift_factor)
                })
    
    return singularity_points


def analyze_lift_loss(config: TricopterConfig,
                      tilt_range: Tuple[float, float] = (-90, 90),
                      step: float = 5.0) -> List[Dict]:
    """
    分析升力损失
    
    参数:
        config: 三旋翼配置
        tilt_range: 倾转角度范围（度）
        step: 步长（度）
    
    返回:
        升力损失分析结果列表
    """
    results = []
    angles = np.arange(tilt_range[0], tilt_range[1] + step, step)
    
    for tilt_front in angles:
        for tilt_rear in angles:
            tilt_angles = {
                'FL': float(tilt_front),
                'FR': float(tilt_front),
                'R': float(tilt_rear)
            }
            
            # 计算升力
            motor_thrusts = calculate_all_motor_thrusts(config, tilt_angles)
            total_forces = calculate_total_forces(motor_thrusts)
            
            # 总推力（假设所有电机推力为1）
            total_thrust = 6.0  # 6个电机
            
            lift_ratio = total_forces['total_lift'] / total_thrust if total_thrust > 0 else 0.0
            lift_loss = 1.0 - lift_ratio
            
            results.append({
                'tilt_FL': float(tilt_front),
                'tilt_FR': float(tilt_front),
                'tilt_R': float(tilt_rear),
                'lift_ratio': float(lift_ratio),
                'lift_loss_ratio': float(lift_loss),
                'total_lift': float(total_forces['total_lift']),
                'total_thrust': float(total_thrust)
            })
    
    return results

