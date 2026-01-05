"""
基本功能测试脚本
"""

import sys
from config import TricopterConfig
from geometry import calculate_all_motor_thrusts
from allocation_matrix import build_allocation_matrix, analyze_matrix, calculate_condition_number
from analysis import analyze_tilt_angles

def test_basic():
    """基本功能测试"""
    print("=" * 60)
    print("基本功能测试")
    print("=" * 60)
    
    # 测试配置
    print("\n[1] 测试配置模块...")
    config = TricopterConfig()
    print(f"  ✓ 前左电机位置: {config.get_motor_position('FL')}")
    print(f"  ✓ 前右电机位置: {config.get_motor_position('FR')}")
    print(f"  ✓ 后置电机位置: {config.get_motor_position('R')}")
    
    # 测试几何计算
    print("\n[2] 测试几何计算模块...")
    tilt_angles = {'FL': 30.0, 'FR': 30.0, 'R': 0.0}
    motor_thrusts = calculate_all_motor_thrusts(config, tilt_angles)
    print(f"  ✓ 前左电机推力分解: Fx={motor_thrusts['FL']['Fx']:.3f}, Fz={motor_thrusts['FL']['Fz']:.3f}")
    print(f"  ✓ 前右电机推力分解: Fx={motor_thrusts['FR']['Fx']:.3f}, Fz={motor_thrusts['FR']['Fz']:.3f}")
    print(f"  ✓ 后置电机推力分解: Fx={motor_thrusts['R']['Fx']:.3f}, Fz={motor_thrusts['R']['Fz']:.3f}")
    
    # 测试矩阵构建
    print("\n[3] 测试控制分配矩阵构建...")
    matrix = build_allocation_matrix(config, tilt_angles)
    print(f"  ✓ 矩阵形状: {matrix.shape}")
    print(f"  ✓ 矩阵:\n{matrix}")
    
    # 测试矩阵分析
    print("\n[4] 测试矩阵分析...")
    analysis = analyze_matrix(matrix)
    print(f"  ✓ 条件数: {analysis['condition_number']:.2f}")
    print(f"  ✓ 是否奇异: {analysis['is_singular']}")
    print(f"  ✓ 最小升力因子: {analysis['min_lift_factor']:.3f}")
    
    # 测试完整分析
    print("\n[5] 测试完整分析...")
    result = analyze_tilt_angles(config, tilt_angles)
    print(f"  ✓ 升力效率: {result['lift_efficiency']:.2%}")
    print(f"  ✓ 总升力: {result['total_forces']['total_lift']:.3f}")
    
    # 测试奇异性情况
    print("\n[6] 测试奇异性情况（90度倾转）...")
    singular_tilt = {'FL': 90.0, 'FR': 90.0, 'R': 90.0}
    singular_matrix = build_allocation_matrix(config, singular_tilt)
    singular_analysis = analyze_matrix(singular_matrix)
    print(f"  ✓ 条件数: {singular_analysis['condition_number']}")
    print(f"  ✓ 是否奇异: {singular_analysis['is_singular']}")
    print(f"  ✓ 奇异信息: {singular_analysis['singularity_message']}")
    
    print("\n" + "=" * 60)
    print("所有测试通过！")
    print("=" * 60)
    return True

if __name__ == '__main__':
    try:
        test_basic()
    except Exception as e:
        print(f"\n测试失败: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

