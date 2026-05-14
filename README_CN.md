# PMSM PID 转速控制仿真

**[English](README.md) | [中文说明](README_CN.md)**

---

## 简介

本项目使用 **PID控制器** 对 **永磁同步电机(PMSM)** 进行转速控制仿真。

### 关键结果

| 指标 | 目标 | 结果 |
|------|------|------|
| 3秒转速 | 1500 RPM | ✅ 1500.00 RPM |
| 超调量 | < 2% | ✅ 0.24% |
| 稳定性(5-10秒) | < 2 RPM波动 | ✅ 0.00 RPM |

### 快速开始

```bash
python pmsm_pid_control.py
```

### 性能图表

![转速响应](pmsm_pid_speed_control.png)

### 系统模型

**电气方程：**
```
dIq/dt = (Vq - Ra·Iq) / Lq
```

**机械方程：**
```
J·dω/dt = Kt·Iq - B·ω
```

### PID参数

| 参数 | 数值 |
|------|------|
| Kp | 20.0 |
| Ki | 80.0 |
| Kd | 2.0 |

---

## PID调参Skill

通用PID调参工具，用于电机转速控制。

### 功能特点

- **增量式PID控制器** - 稳定的控制算法
- **符号自动映射** - 支持各种参数命名方式
- **用户确认机制** - 确保参数正确理解
- **自动调参** - 自动优化PID参数

### 使用方法

```python
from skill.pid_tuning_core import auto_tune, IncrementalPID, MotorModel

# 自动调参
result = auto_tune(J=0.1, B=0.05, Kt=0.5, Ra=1.0, target_rpm=1500)
print(f"PID参数: Kp={result['Kp']:.2f}, Ki={result['Ki']:.2f}, Kd={result['Kd']:.2f}")

# 手动使用
pid = IncrementalPID(Kp=20, Ki=80, Kd=2)
motor = MotorModel(Ra=1.0, J=0.1, B=0.05, Kt=0.5)

for i in range(2000):
    u = pid.compute(target=157, measurement=motor.omega, dt=0.005)
    motor.step(u, dt=0.005)
```

### Skill结构

```
skill/
├── skill.md              # Skill配置（触发词 + 工作流程）
├── pid_tuning_core.py    # 核心算法
├── __init__.py           # Python包标识
└── README.md             # 文档
```

### 参数符号映射

Skill自动识别各种参数名称：

| 标准符号 | 别名 |
|----------|------|
| Ra (电阻) | r, res, R, resistance |
| J (转动惯量) | jm, inertia, moment |
| B (阻尼) | b, damp, friction, D |
| Kt (转矩常数) | kt, torque_k, tm |

### 工作流程

1. 用户以任意格式提供参数：`Ra=1, Lq=0.01, J=0.001`
2. Skill识别并映射符号
3. 用户确认映射关系
4. 自动调参生成最优PID参数

### 默认目标

| 参数 | 默认值 |
|------|--------|
| 上升时间 | < 3秒 |
| 超调量 | < 2% |
| 稳态误差 | < 1% |

---

## 项目结构

```
pid-pmsm/
├── pmsm_pid_control.py          # 主仿真程序
├── pmsm_pid_speed_control.png    # 结果图表
├── skill/                        # PID调参工具
│   ├── skill.md                  # Skill配置
│   ├── pid_tuning_core.py        # 核心算法
│   └── README.md                # Skill文档
├── README.md                     # 英文版
└── README_CN.md                  # 中文版
```

---

## 开源许可

MIT License