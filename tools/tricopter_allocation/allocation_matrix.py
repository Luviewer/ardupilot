"""
控制分配矩阵模块 - 构建和分析控制分配矩阵
"""

import numpy as np
from typing import Dict, Tuple, Optional
from config import TricopterConfig
from geometry import tilt_thrust_vector


def build_allocation_matrix(config: TricopterConfig,
                           tilt_angles: Dict[str, float]) -> np.ndarray:
    """
    构建控制分配矩阵
    
    控制分配方程：
    [τ_roll  ]   [M_roll_FL  M_roll_FR  M_roll_R ]   [T_FL]
    [τ_pitch ] = [M_pitch_FL M_pitch_FR M_pitch_R] · [T_FR]
    [L_body  ]   [cos(θ_FL)  cos(θ_FR)  cos(θ_R) ] [T_R ]
    
    参数:
        config: 三旋翼配置
        tilt_angles: 各电机倾转角度（度）
    
    返回:
        控制分配矩阵 (3x3)
    """
    # 获取电机位置
    pos_FL = config.get_motor_position('FL')
    pos_FR = config.get_motor_position('FR')
    pos_R = config.get_motor_position('R')
    
    # 转换倾转角度为弧度
    tilt_FL_rad = np.deg2rad(tilt_angles.get('FL', 0.0))
    tilt_FR_rad = np.deg2rad(tilt_angles.get('FR', 0.0))
    tilt_R_rad = np.deg2rad(tilt_angles.get('R', 0.0))
    
    # 计算推力向量
    thrust_vec_FL = tilt_thrust_vector(tilt_FL_rad)
    thrust_vec_FR = tilt_thrust_vector(tilt_FR_rad)
    thrust_vec_R = tilt_thrust_vector(tilt_R_rad)
    
    # 计算力矩系数（每个共轴对有两个电机，所以乘以2）
    # 
    # 根据叉积计算：τ = r × F
    # r = [x, y, 0], F = [sin(θ), 0, -cos(θ)]
    # τ = [-y·cos(θ), x·cos(θ), -y·sin(θ)]
    #
    # Roll力矩：τ_roll = -y · cos(θ)  (绕X轴，y为负时产生正roll)
    # Pitch力矩：τ_pitch = x · cos(θ)  (绕Y轴，x为正时产生正pitch)
    # Yaw力矩：τ_yaw = -y · sin(θ)  (绕Z轴，通常很小)
    
    # Roll力矩系数
    M_roll_FL = -2 * pos_FL[1] * np.cos(tilt_FL_rad)  # y_FL < 0，所以负号
    M_roll_FR = -2 * pos_FR[1] * np.cos(tilt_FR_rad)  # y_FR > 0
    M_roll_R = -2 * pos_R[1] * np.cos(tilt_R_rad)     # y_R = 0
    
    # Pitch力矩系数
    M_pitch_FL = 2 * pos_FL[0] * np.cos(tilt_FL_rad)   # x_FL > 0
    M_pitch_FR = 2 * pos_FR[0] * np.cos(tilt_FR_rad)   # x_FR > 0
    M_pitch_R = 2 * pos_R[0] * np.cos(tilt_R_rad)      # x_R < 0
    
    # 升力行：L_body = Σ(T_i · cos(θ_i))
    M_lift_FL = 2 * np.cos(tilt_FL_rad)
    M_lift_FR = 2 * np.cos(tilt_FR_rad)
    M_lift_R = 2 * np.cos(tilt_R_rad)
    
    # 构建矩阵
    matrix = np.array([
        [M_roll_FL, M_roll_FR, M_roll_R],
        [M_pitch_FL, M_pitch_FR, M_pitch_R],
        [M_lift_FL, M_lift_FR, M_lift_R]
    ])
    
    return matrix


def calculate_condition_number(matrix: np.ndarray) -> float:
    """
    计算矩阵条件数
    
    参数:
        matrix: 控制分配矩阵
    
    返回:
        条件数（cond(A) = ||A|| · ||A^(-1)||）
    """
    try:
        # 使用2-范数
        cond = np.linalg.cond(matrix)
        return cond
    except np.linalg.LinAlgError:
        # 矩阵奇异，条件数为无穷大
        return np.inf


def check_singularity(matrix: np.ndarray, threshold: float = 0.1) -> Tuple[bool, str]:
    """
    检测矩阵是否接近奇异
    
    参数:
        matrix: 控制分配矩阵
        threshold: 奇异性阈值（最小cos值）
    
    返回:
        (is_singular, message) 元组
    """
    # 检查第三行（升力行）是否接近零
    lift_row = matrix[2, :]
    min_cos = np.min(np.abs(lift_row))
    
    if min_cos < threshold:
        return True, f"矩阵接近奇异：最小升力因子 = {min_cos:.4f} < {threshold}"
    
    # 检查矩阵是否可逆
    try:
        det = np.linalg.det(matrix)
        if abs(det) < 1e-10:
            return True, f"矩阵奇异：行列式 = {det:.4e}"
    except:
        pass
    
    return False, "矩阵正常"


def analyze_matrix(matrix: np.ndarray) -> Dict:
    """
    分析控制分配矩阵
    
    参数:
        matrix: 控制分配矩阵
    
    返回:
        分析结果字典
    """
    # 计算条件数
    cond = calculate_condition_number(matrix)
    
    # 检测奇异性
    is_singular, singularity_msg = check_singularity(matrix)
    
    # 计算行列式
    try:
        det = np.linalg.det(matrix)
    except:
        det = 0.0
    
    # 计算特征值
    try:
        eigenvals = np.linalg.eigvals(matrix)
        eigenvals_real = np.real(eigenvals)
        min_eigenval = np.min(np.abs(eigenvals_real))
        max_eigenval = np.max(np.abs(eigenvals_real))
    except:
        eigenvals_real = []
        min_eigenval = 0.0
        max_eigenval = 0.0
    
    # 提取各行信息
    roll_row = matrix[0, :].tolist()
    pitch_row = matrix[1, :].tolist()
    lift_row = matrix[2, :].tolist()
    
    return {
        'matrix': matrix.tolist(),
        'condition_number': float(cond) if not np.isinf(cond) else None,
        'determinant': float(det),
        'is_singular': bool(is_singular),
        'singularity_message': singularity_msg,
        'eigenvalues': [float(v) for v in eigenvals_real],
        'min_eigenvalue': float(min_eigenval),
        'max_eigenvalue': float(max_eigenval),
        'roll_row': roll_row,
        'pitch_row': pitch_row,
        'lift_row': lift_row,
        'min_lift_factor': float(np.min(np.abs(lift_row)))
    }

