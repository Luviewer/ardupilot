# AP_QuadRuped 架构重构文档

## 概述

本次重构将AP_QuadRuped库从传统的继承结构改为符合Ardupilot标准的**前端控制器+多后端**架构模式。

## 架构设计

### 核心架构图

```
┌─────────────────────────────────────────────────────────────┐
│                    AP_QuadRuped (前端控制器)                   │
├─────────────────────────────────────────────────────────────┤
│  • 步态管理                • 参数管理                        │
│  • 控制输入                • 硬件接口                        │
│  • 后端切换                • 健康监控                        │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                AP_QuadRuped_Backend (后端接口)                │
├─────────────────────────────────────────────────────────────┤
│  • 纯虚函数接口            • 通用工具函数                    │
│  • 硬件抽象                • 运动学计算                      │
└─────────────────────────────────────────────────────────────┘
                            │
          ┌─────────────────┼─────────────────┐
          ▼                 ▼                 ▼
┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐
│AP_QuadRuped_Diag│ │AP_QuadRuped_WAVE│ │AP_QuadRuped_shuxiang │
│   (对角步态)    │ │   (波浪步态)    │ │   (工字步态)    │
└─────────────────┘ └─────────────────┘ └─────────────────┘
```

## 文件结构

### 新增文件

1. **AP_QuadRuped_Backend.h/.cpp** - 后端接口定义和基础实现
2. **AP_QuadRuped_New.h/.cpp** - 重构后的前端控制器
3. **AP_QuadRuped_Diag_New.h/.cpp** - 对角步态后端实现
4. **AP_QuadRuped_WAVE_New.h/.cpp** - 波浪步态后端实现
5. **AP_QuadRuped_shuxiang.h/.cpp** - 工字步态后端实现
5. **AP_QuadRuped_hengxiang.h/.cpp** - 工字步态后端实现

### 原有文件

- **AP_QuadRuped.h/.cpp** - 保持兼容性，包含原有接口
- **AP_QuadRuped_Base.h/.cpp** - 基础功能类
- **AP_QuadRuped_Params.h/.cpp** - 参数定义

## 核心特性

### 1. 前后端分离

**前端控制器 (AP_QuadRuped_New)**：
- 管理所有后端实例
- 处理步态切换逻辑
- 提供统一的控制接口
- 管理参数和硬件接口

**后端接口 (AP_QuadRuped_Backend)**：
- 定义纯虚函数接口
- 提供通用工具函数
- 实现硬件抽象
- 统一的逆运动学计算

### 2. 动态步态切换

```cpp
// 支持运行时动态切换步态
quadru.set_gait_type(AP_QuadRuped::GAIT_DIAGONAL);
quadru.set_gait_type(AP_QuadRuped::GAIT_WAVE);
quadru.set_gait_type(AP_QuadRuped::GAIT_shuxiang);
quadru.set_gait_type(AP_QuadRuped::GAIT_hengxiang);
```

### 3. 参数系统

每个后端都有自己的参数组：
```
QRUD_GTYPE     # 步态类型选择
QRUD_ENBL      # 使能状态

QRUD_DIAG_STEP_H    # 对角步态抬腿高度
QRUD_DIAG_STEP_L    # 对角步态步长
QRUD_DIAG_STEP_F    # 对角步态步频

QRUD_WAVE_STEP_H    # 波浪步态抬腿高度
QRUD_WAVE_STEP_L    # 波浪步态步长
QRUD_WAVE_STEP_F    # 波浪步态步频

QRUD_shuxiang_STEP_H    # 竖向工字步态抬腿高度
QRUD_shuxiang_STEP_L    # 竖向工字步态步长
QRUD_shuxiang_STEP_F    # 竖向工字步态步频

QRUD_hengxiang_STEP_H    # 横向工字步态抬腿高度
QRUD_hengxiang_STEP_L    # 横向工字步态步长
QRUD_hengxiang_STEP_F    # 横向工字步态步频
```

### 4. 健康监控

每个后端都有独立的健康检查：
- 参数范围验证
- 硬件接口状态
- 初始化状态检查

## 使用方法

### 基本使用

```cpp
// 1. 创建实例
AP_QuadRuped quadru(ahrs, motors, rangefinder);

// 2. 初始化
quadru.init();

// 3. 设置步态
quadru.set_gait_type(AP_QuadRuped::GAIT_DIAGONAL);

// 4. 设置控制输入
quadru.set_throttle(0.5f, 0.3f);
quadru.set_yaw_rate(0.1f);
quadru.set_body_height(0.8f);

// 5. 更新循环
quadru.update();
```

### 步态切换

```cpp
// 根据需求切换不同步态
if (need_speed) {
    quadru.set_gait_type(AP_QuadRuped::GAIT_DIAGONAL);
} else if (need_stability) {
    quadru.set_gait_type(AP_QuadRuped::GAIT_WAVE);
} else if (need_shuxiang_maneuverability) {
    quadru.set_gait_type(AP_QuadRuped::GAIT_shuxiang);
} else if (need_hengxiang_maneuverability) {
    quadru.set_gait_type(AP_QuadRuped::GAIT_hengxiang);
}
```

## 步态特性

### 对角步态 (Diagonal Gait)
- **特点**：对角线腿部同步运动
- **优势**：速度快，稳定性好
- **适用**：平地快速行走

### 波浪步态 (Wave Gait)
- **特点**：腿部依次抬腿，形成波浪状
- **优势**：稳定性最高，适应性强
- **适用**：复杂地形，慢速行走

### 工字步态 (shuxiang Gait)
- **特点**：同侧腿部同步运动
- **优势**：竖向移动灵活
- **适用**：狭窄空间，竖向移动

### 工字步态 (hengxiang Gait)
- **特点**：同侧腿部同步运动
- **优势**：横向移动灵活
- **适用**：狭窄空间，横向移动

## 扩展指南

### 添加新步态

1. **创建后端类**：
```cpp
class AP_QuadRuped_Custom : public AP_QuadRuped_Backend {
    // 实现所有纯虚函数
};
```

2. **注册到前端**：
```cpp
// 在AP_QuadRuped_New.cpp中添加
_gait_backends[GAIT_CUSTOM] = new AP_QuadRuped_Custom(*this, _ahrs, _motors);
```

3. **添加参数组**：
```cpp
// 在参数表中添加新的参数组
AP_SUBGROUPINFO(_custom_params, "CUSTOM_", X, AP_QuadRuped, AP_QuadRuped_Custom_Params);
```

### 添加新功能

1. **在后端接口中添加虚函数**：
```cpp
virtual void new_function() = 0;
```

2. **在各个后端中实现**：
```cpp
void AP_QuadRuped_Diag::new_function() override {
    // 具体实现
}
```

## 兼容性

### 向后兼容

保持原有的AP_QuadRuped接口不变，确保现有代码可以继续工作：

```cpp
// 原有接口仍然可用
AP_QuadRuped quadru(ahrs, motors, rangefinder);
quadru.update_all();  // 调用原有接口
```

### 平滑迁移

可以逐步迁移到新的架构：

```cpp
// 使用新的重构接口
AP_QuadRuped_New quadru(ahrs, motors, rangefinder);
quadru.init();
quadru.update();
```

## 测试

提供了完整的测试套件：

```bash
# 编译测试
g++ test_architecture.cpp -o test_architecture

# 运行测试
./test_architecture
```

测试覆盖：
- 后端接口测试
- 步态切换测试
- 参数访问测试
- 健康监控测试

## 总结

这次重构成功地将AP_QuadRuped转换为符合Ardupilot标准的架构，提供了：

1. **更好的模块化**：每个步态独立实现，互不影响
2. **更强的扩展性**：轻松添加新步态和新功能
3. **更好的维护性**：代码结构清晰，职责明确
4. **更高的兼容性**：保持向后兼容，平滑迁移
5. **更好的测试性**：每个模块可独立测试

这个架构为四足机器人控制提供了一个稳定、灵活、可扩展的基础平台。