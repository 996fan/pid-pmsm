# PMSM PID Speed Control Simulation

**[English](README.md) | [中文说明](README_CN.md)**

---

## Overview

This project simulates a **Permanent Magnet Synchronous Motor (PMSM)** speed control system using a **PID controller** implemented in Python.

### Key Results

| Metric | Target | Result |
|--------|--------|--------|
| Speed at 3s | 1500 RPM | ✅ 1500.00 RPM |
| Overshoot | < 2% | ✅ 0.24% |
| Stability (5-10s) | < 2 RPM variation | ✅ 0.00 RPM |

### Quick Start

```bash
python pmsm_pid_control.py
```

### Performance

![Speed Control Response](pmsm_pid_speed_control.png)

### System Model

**Electrical:**
```
dIq/dt = (Vq - Ra·Iq) / Lq
```

**Mechanical:**
```
J·dω/dt = Kt·Iq - B·ω
```

### PID Parameters

| Parameter | Value |
|-----------|-------|
| Kp | 20.0 |
| Ki | 80.0 |
| Kd | 2.0 |

---

## PID Tuning Skill

A reusable PID tuning tool for motor speed control.

### Features

- **Incremental PID Controller** - Stable control algorithm
- **Symbol Mapping** - Supports various parameter naming conventions
- **User Confirmation** - Ensures correct parameter understanding
- **Auto Tuning** - Automatic PID parameter optimization

### Usage

```python
from skill.pid_tuning_core import auto_tune, IncrementalPID, MotorModel

# Auto tune
result = auto_tune(J=0.1, B=0.05, Kt=0.5, Ra=1.0, target_rpm=1500)
print(f"PID: Kp={result['Kp']:.2f}, Ki={result['Ki']:.2f}, Kd={result['Kd']:.2f}")

# Manual control
pid = IncrementalPID(Kp=20, Ki=80, Kd=2)
motor = MotorModel(Ra=1.0, J=0.1, B=0.05, Kt=0.5)

for i in range(2000):
    u = pid.compute(target=157, measurement=motor.omega, dt=0.005)
    motor.step(u, dt=0.005)
```

### Skill Structure

```
skill/
├── skill.md              # Skill configuration (triggers + workflow)
├── pid_tuning_core.py    # Core algorithm
├── __init__.py          # Package init
└── README.md            # Documentation
```

### Parameter Mapping

The skill automatically recognizes various parameter names:

| Standard | Aliases |
|----------|---------|
| Ra (resistance) | r, res, R, resistance |
| J (inertia) | jm, inertia, moment |
| B (damping) | b, damp, friction, D |
| Kt (torque const) | kt, torque_k, tm |

### Workflow

1. User provides parameters in any format: `Ra=1, Lq=0.01, J=0.001`
2. Skill identifies and maps symbols
3. User confirms the mapping
4. Auto tuning generates optimal PID parameters

### Default Targets

| Parameter | Default |
|-----------|---------|
| Rise time | < 3s |
| Overshoot | < 2% |
| Steady state error | < 1% |

---

## Project Structure

```
pid-pmsm/
├── pmsm_pid_control.py          # Main simulation
├── pmsm_pid_speed_control.png    # Results plot
├── skill/                        # PID tuning skill
│   ├── skill.md                  # Skill configuration
│   ├── pid_tuning_core.py        # Core algorithm
│   └── README.md                # Skill documentation
├── README.md                     # English (this file)
└── README_CN.md                  # Chinese version
```

---

## License

MIT License