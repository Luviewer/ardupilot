"""
输出模块 - JSON格式输出，分块保存结果
"""

import json
import os
import numpy as np
from typing import Dict, List, Any
from pathlib import Path


class NumpyJSONEncoder(json.JSONEncoder):
    """自定义JSON编码器，处理numpy类型"""
    def default(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, (np.bool_, bool)):
            return bool(obj)
        return super().default(obj)


def convert_numpy_types(obj: Any) -> Any:
    """
    递归转换numpy类型为Python原生类型
    
    参数:
        obj: 要转换的对象
    
    返回:
        转换后的对象
    """
    if isinstance(obj, np.integer):
        return int(obj)
    elif isinstance(obj, np.floating):
        return float(obj)
    elif isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, (np.bool_, bool)):
        return bool(obj)
    elif isinstance(obj, dict):
        return {key: convert_numpy_types(value) for key, value in obj.items()}
    elif isinstance(obj, list):
        return [convert_numpy_types(item) for item in obj]
    elif isinstance(obj, tuple):
        return tuple(convert_numpy_types(item) for item in obj)
    return obj


def save_json(data: Dict, filepath: str, indent: int = 2) -> None:
    """
    保存数据为JSON文件
    
    参数:
        data: 要保存的数据
        filepath: 文件路径
        indent: JSON缩进
    """
    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    # 转换numpy类型
    data = convert_numpy_types(data)
    with open(filepath, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=indent, ensure_ascii=False, cls=NumpyJSONEncoder)


def save_config(config: Dict, output_dir: str) -> str:
    """
    保存配置参数
    
    参数:
        config: 配置字典
        output_dir: 输出目录
    
    返回:
        保存的文件路径
    """
    filepath = os.path.join(output_dir, 'config.json')
    save_json(config, filepath)
    return filepath


def save_motor_positions(motor_positions: Dict, output_dir: str) -> str:
    """
    保存电机位置信息
    
    参数:
        motor_positions: 电机位置字典
        output_dir: 输出目录
    
    返回:
        保存的文件路径
    """
    filepath = os.path.join(output_dir, 'motor_positions.json')
    save_json(motor_positions, filepath)
    return filepath


def save_tilt_analysis(results: List[Dict], output_dir: str, chunk_size: int = 100) -> List[str]:
    """
    保存倾转角度分析结果（分块）
    
    参数:
        results: 分析结果列表
        output_dir: 输出目录
        chunk_size: 每块的大小
    
    返回:
        保存的文件路径列表
    """
    filepaths = []
    num_chunks = (len(results) + chunk_size - 1) // chunk_size
    
    for i in range(num_chunks):
        start_idx = i * chunk_size
        end_idx = min((i + 1) * chunk_size, len(results))
        chunk = results[start_idx:end_idx]
        
        filename = f'tilt_analysis_chunk_{i+1:03d}_of_{num_chunks:03d}.json'
        filepath = os.path.join(output_dir, filename)
        
        chunk_data = {
            'chunk_index': i + 1,
            'total_chunks': num_chunks,
            'start_index': start_idx,
            'end_index': end_idx,
            'total_results': len(results),
            'results': chunk
        }
        
        save_json(chunk_data, filepath)
        filepaths.append(filepath)
    
    return filepaths


def save_matrix_condition(condition_data: List[Dict], output_dir: str, chunk_size: int = 500) -> List[str]:
    """
    保存矩阵条件数分析结果（分块）
    
    参数:
        condition_data: 条件数分析数据列表
        output_dir: 输出目录
        chunk_size: 每块的大小
    
    返回:
        保存的文件路径列表
    """
    filepaths = []
    num_chunks = (len(condition_data) + chunk_size - 1) // chunk_size
    
    for i in range(num_chunks):
        start_idx = i * chunk_size
        end_idx = min((i + 1) * chunk_size, len(condition_data))
        chunk = condition_data[start_idx:end_idx]
        
        filename = f'matrix_condition_chunk_{i+1:03d}_of_{num_chunks:03d}.json'
        filepath = os.path.join(output_dir, filename)
        
        chunk_data = {
            'chunk_index': i + 1,
            'total_chunks': num_chunks,
            'start_index': start_idx,
            'end_index': end_idx,
            'total_results': len(condition_data),
            'results': chunk
        }
        
        save_json(chunk_data, filepath)
        filepaths.append(filepath)
    
    return filepaths


def save_singularity_analysis(singularity_data: List[Dict], output_dir: str) -> str:
    """
    保存奇异性分析结果
    
    参数:
        singularity_data: 奇异性分析数据列表
        output_dir: 输出目录
    
    返回:
        保存的文件路径
    """
    filepath = os.path.join(output_dir, 'singularity_analysis.json')
    
    data = {
        'total_singularity_points': len(singularity_data),
        'singularity_points': singularity_data
    }
    
    save_json(data, filepath)
    return filepath


def save_summary(summary: Dict, output_dir: str) -> str:
    """
    保存结果汇总
    
    参数:
        summary: 汇总数据字典
        output_dir: 输出目录
    
    返回:
        保存的文件路径
    """
    filepath = os.path.join(output_dir, 'summary.json')
    save_json(summary, filepath)
    return filepath


def create_output_directory(base_dir: str = 'output') -> str:
    """
    创建输出目录
    
    参数:
        base_dir: 基础目录名
    
    返回:
        创建的目录路径
    """
    from datetime import datetime
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    output_dir = os.path.join(base_dir, f'tricopter_analysis_{timestamp}')
    os.makedirs(output_dir, exist_ok=True)
    return output_dir

