"""
主程序 - 整合所有模块，执行计算流程，生成报告
"""

import os
import sys
from typing import Dict, List
from config import TricopterConfig, default_config
from geometry import calculate_all_motor_thrusts, calculate_total_forces
from torque import calculate_motor_torques, calculate_total_torques
from allocation_matrix import build_allocation_matrix, analyze_matrix
from analysis import (
    analyze_tilt_angles,
    sweep_tilt_angles,
    analyze_condition_number_vs_tilt,
    find_singularity_points,
    analyze_lift_loss
)
from output import (
    create_output_directory,
    save_config,
    save_motor_positions,
    save_tilt_analysis,
    save_matrix_condition,
    save_singularity_analysis,
    save_summary
)


def main():
    """主函数"""
    print("=" * 60)
    print("三旋翼控制分配矩阵计算工具")
    print("=" * 60)
    
    # 创建输出目录
    output_dir = create_output_directory('output')
    print(f"\n输出目录: {output_dir}")
    
    # 使用默认配置或自定义配置
    config = default_config
    
    # 保存配置
    print("\n[1/7] 保存配置参数...")
    config_dict = config.to_dict()
    save_config(config_dict, output_dir)
    print("  ✓ 配置已保存到 config.json")
    
    # 保存电机位置
    print("\n[2/7] 保存电机位置信息...")
    motor_positions = {
        'positions': {k: v.tolist() for k, v in config.get_all_motor_positions().items()},
        'coaxial_pairs': config.coaxial_pairs
    }
    save_motor_positions(motor_positions, output_dir)
    print("  ✓ 电机位置已保存到 motor_positions.json")
    
    # 分析特定倾转角度配置
    print("\n[3/7] 分析特定倾转角度配置...")
    test_tilt_angles = [
        {'FL': 0.0, 'FR': 0.0, 'R': 0.0},      # 无倾转
        {'FL': 30.0, 'FR': 30.0, 'R': 0.0},    # 前电机30度
        {'FL': 45.0, 'FR': 45.0, 'R': 0.0},   # 前电机45度
        {'FL': 60.0, 'FR': 60.0, 'R': 0.0},   # 前电机60度
        {'FL': 90.0, 'FR': 90.0, 'R': 0.0},   # 前电机90度（奇异）
    ]
    
    specific_analyses = []
    for tilt_angles in test_tilt_angles:
        result = analyze_tilt_angles(config, tilt_angles)
        specific_analyses.append(result)
    
    # 保存特定分析结果
    specific_filepath = os.path.join(output_dir, 'specific_tilt_analyses.json')
    import json
    with open(specific_filepath, 'w', encoding='utf-8') as f:
        json.dump(specific_analyses, f, indent=2, ensure_ascii=False)
    print(f"  ✓ 特定倾转角度分析已保存到 specific_tilt_analyses.json")
    
    # 扫描倾转角度范围
    print("\n[4/7] 扫描倾转角度范围（这可能需要一些时间）...")
    print("  扫描范围: -90° 到 90°，步长 10°")
    sweep_results = sweep_tilt_angles(
        config,
        tilt_range=(-90, 90),
        step=10.0,
        symmetric=True
    )
    print(f"  共生成 {len(sweep_results)} 个配置")
    
    # 分块保存倾转分析结果
    tilt_files = save_tilt_analysis(sweep_results, output_dir, chunk_size=100)
    print(f"  ✓ 倾转分析结果已分块保存到 {len(tilt_files)} 个文件")
    
    # 分析条件数随倾转角度的变化
    print("\n[5/7] 分析矩阵条件数随倾转角度的变化...")
    print("  分析范围: -90° 到 90°，步长 5°")
    condition_results = analyze_condition_number_vs_tilt(
        config,
        tilt_range=(-90, 90),
        step=5.0
    )
    print(f"  共分析 {len(condition_results)} 个配置")
    
    # 分块保存条件数分析
    condition_files = save_matrix_condition(condition_results, output_dir, chunk_size=500)
    print(f"  ✓ 条件数分析结果已分块保存到 {len(condition_files)} 个文件")
    
    # 查找奇异性点
    print("\n[6/7] 查找奇异性点...")
    print("  搜索范围: -90° 到 90°，步长 1°")
    singularity_points = find_singularity_points(
        config,
        tilt_range=(-90, 90),
        step=1.0
    )
    print(f"  找到 {len(singularity_points)} 个奇异性点")
    
    # 保存奇异性分析
    save_singularity_analysis(singularity_points, output_dir)
    print("  ✓ 奇异性分析已保存到 singularity_analysis.json")
    
    # 分析升力损失
    print("\n[7/7] 分析升力损失...")
    lift_loss_results = analyze_lift_loss(
        config,
        tilt_range=(-90, 90),
        step=10.0
    )
    print(f"  共分析 {len(lift_loss_results)} 个配置")
    
    # 保存升力损失分析
    lift_filepath = os.path.join(output_dir, 'lift_loss_analysis.json')
    with open(lift_filepath, 'w', encoding='utf-8') as f:
        json.dump(lift_loss_results, f, indent=2, ensure_ascii=False)
    print("  ✓ 升力损失分析已保存到 lift_loss_analysis.json")
    
    # 生成汇总报告
    print("\n生成汇总报告...")
    summary = {
        'config': config_dict,
        'analysis_summary': {
            'specific_analyses_count': len(specific_analyses),
            'sweep_results_count': len(sweep_results),
            'condition_analysis_count': len(condition_results),
            'singularity_points_count': len(singularity_points),
            'lift_loss_analysis_count': len(lift_loss_results)
        },
        'output_files': {
            'config': 'config.json',
            'motor_positions': 'motor_positions.json',
            'specific_analyses': 'specific_tilt_analyses.json',
            'tilt_analysis_chunks': len(tilt_files),
            'condition_analysis_chunks': len(condition_files),
            'singularity_analysis': 'singularity_analysis.json',
            'lift_loss_analysis': 'lift_loss_analysis.json'
        },
        'key_findings': {
            'singularity_warning': f"发现 {len(singularity_points)} 个奇异性配置",
            'max_condition_number': max([
                r['condition_number'] for r in condition_results 
                if r['condition_number'] is not None
            ]) if condition_results else None,
            'min_lift_ratio': min([
                r['lift_ratio'] for r in lift_loss_results
            ]) if lift_loss_results else None
        }
    }
    
    save_summary(summary, output_dir)
    print("  ✓ 汇总报告已保存到 summary.json")
    
    # 打印关键发现
    print("\n" + "=" * 60)
    print("关键发现:")
    print("=" * 60)
    print(f"  奇异性点数量: {len(singularity_points)}")
    if condition_results:
        max_cond = max([r['condition_number'] for r in condition_results if r['condition_number'] is not None])
        print(f"  最大条件数: {max_cond:.2f}")
    if lift_loss_results:
        min_lift = min([r['lift_ratio'] for r in lift_loss_results])
        print(f"  最小升力比例: {min_lift:.2%}")
    print("\n" + "=" * 60)
    print(f"所有结果已保存到: {output_dir}")
    print("=" * 60)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n用户中断")
        sys.exit(1)
    except Exception as e:
        print(f"\n\n错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

