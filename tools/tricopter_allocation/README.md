# 三旋翼控制分配矩阵计算工具

## 概述

这是一个模块化的Python工具，用于计算和分析共轴三旋翼的控制分配矩阵。工具可以分析不同倾转角度配置下的控制能力、奇异性、升力损失等。

## 功能特性

- **模块化设计**：代码分为配置、几何、力矩、矩阵、分析和输出等模块
- **分块输出**：结果以JSON格式分块保存，避免文件过大
- **全面分析**：包括矩阵条件数、奇异性检测、升力损失分析等
- **可配置参数**：支持自定义电机位置和倾转角度范围

## 模块说明

### 1. config.py - 配置模块
定义三旋翼的电机位置、倾转角度范围等参数。

### 2. geometry.py - 几何计算模块
计算电机倾转后的推力向量分解。

### 3. torque.py - 力矩计算模块
计算各轴力矩（Roll, Pitch, Yaw）。

### 4. allocation_matrix.py - 控制分配矩阵模块
构建控制分配矩阵，计算条件数，检测奇异性。

### 5. analysis.py - 分析模块
进行多角度分析，包括：
- 特定倾转角度配置分析
- 倾转角度范围扫描
- 条件数随倾转角度的变化
- 奇异性点查找
- 升力损失分析

### 6. output.py - 输出模块
将结果以JSON格式分块保存。

### 7. main.py - 主程序
整合所有模块，执行完整分析流程。

## 使用方法

### 基本使用

```bash
cd tools/tricopter_allocation
python main.py
```

### 自定义配置

修改 `config.py` 中的 `TricopterConfig` 类，或创建自定义配置：

```python
from config import TricopterConfig

# 自定义电机位置
config = TricopterConfig(
    front_x=0.3,    # 前电机X坐标（米）
    front_y=0.2,    # 前电机Y坐标绝对值（米）
    rear_x=-0.3,    # 后电机X坐标（米）
    rear_y=0.0      # 后电机Y坐标（米）
)
```

### 单独使用模块

```python
from config import TricopterConfig
from allocation_matrix import build_allocation_matrix, analyze_matrix

config = TricopterConfig()
tilt_angles = {'FL': 45.0, 'FR': 45.0, 'R': 0.0}
matrix = build_allocation_matrix(config, tilt_angles)
analysis = analyze_matrix(matrix)
print(analysis)
```

## 输出文件

运行 `main.py` 后，会在 `output/tricopter_analysis_YYYYMMDD_HHMMSS/` 目录下生成以下文件：

- `config.json` - 配置参数
- `motor_positions.json` - 电机位置信息
- `specific_tilt_analyses.json` - 特定倾转角度分析
- `tilt_analysis_chunk_XXX_of_XXX.json` - 倾转角度扫描结果（分块）
- `matrix_condition_chunk_XXX_of_XXX.json` - 条件数分析结果（分块）
- `singularity_analysis.json` - 奇异性点
- `lift_loss_analysis.json` - 升力损失分析
- `summary.json` - 结果汇总

## 依赖

- Python 3.7+
- numpy

安装依赖：
```bash
pip install numpy
```

## 理论依据

控制分配矩阵基于以下理论：

1. **推力分解**：倾转电机的推力向量为 `F = T · [sin(θ), 0, -cos(θ)]`
2. **力矩计算**：`τ = r × F`，其中 `r` 为电机位置向量
3. **控制分配**：`[τ_roll, τ_pitch, L_body]^T = M · [T_FL, T_FR, T_R]^T`

详细理论请参考 `MotorMatrix_coaxial_tricopter_tilt_theory.md`。

## 注意事项

- 默认使用对称倾转（前左和前右角度相同），可在 `analysis.py` 中修改
- 分析大量配置时可能需要较长时间，建议调整步长
- 奇异性检测使用较小的步长（1度）以提高精度

## 许可证

与ArduPilot项目保持一致。

