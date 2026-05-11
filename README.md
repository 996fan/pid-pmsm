# PMSM PID Speed Control Simulation

<div align="center">

![Python](https://img.shields.io/badge/Python-3.7+-blue.svg)
![NumPy](https://img.shields.io/badge/NumPy-1.20+-green.svg)
![Matplotlib](https://img.shields.io/badge/Matplotlib-3.4+-orange.svg)
![License](https://img.shields.io/badge/License-MIT-yellow.svg)

**A PID-based speed control simulation for Permanent Magnet Synchronous Motor (PMSM)**

</div>

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [System Model](#system-model)
- [PID Controller](#pid-controller)
- [Installation](#installation)
- [Usage](#usage)
- [Results](#results)
- [Project Structure](#project-structure)
- [References](#references)

---

## Overview

This project simulates a **Permanent Magnet Synchronous Motor (PMSM)** speed control system using a **PID controller** implemented in Python. The simulation demonstrates how PID control achieves smooth, stable speed regulation from 0 to 1500 RPM within 3 seconds.

### Key Objectives

| Objective | Target | Result |
|-----------|--------|--------|
| Reach 1500 RPM | Within 3 seconds | ✅ 1500.0 RPM @ 3s |
| Overshoot | < 2% | ✅ 0.09% |
| Speed Variation (5-10s) | < 2 RPM | ✅ 0.00 RPM |

---

## Features

- **Complete PMSM Mathematical Model** - D-Q axis electrical dynamics + mechanical dynamics
- **PID Controller with Anti-Windup** - Prevents integral windup for stable control
- **Real-time Simulation** - Time-domain analysis of motor response
- **Performance Metrics** - Automatic calculation of overshoot, settling time, and stability
- **Visualization** - Speed response and voltage output plots

---

## System Model

### PMSM Electrical Model

The d-q axis current dynamics are described by:

```
dIq/dt = (Vq - Ra·Iq) / Lq
```

### PMSM Mechanical Model

The motor's mechanical dynamics follow:

```
J·dω/dt + B·ω = Kt·Iq
```

Where:

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Stator Resistance | Ra | 1.0 | Ω |
| Torque Constant | Kt | 0.5 | N·m/A |
| Moment of Inertia | J | 0.1 | kg·m² |
| Damping Coefficient | B | 0.05 | N·m·s/rad |

---

## PID Controller

### Control Law

```
Vq = Kp·e(t) + Ki·∫e(t)dt + Kd·de(t)/dt
```

### Tuned Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| Kp | 20.0 | Proportional gain |
| Ki | 80.0 | Integral gain |
| Kd | 2.0 | Derivative gain |

### Anti-Windup

Integral term is clamped to `[-20, 20]` to prevent windup phenomena.

---

## Installation

### Prerequisites

- Python 3.7 or higher
- NumPy
- Matplotlib

### Install Dependencies

```bash
pip install numpy matplotlib
```

### Clone the Repository

```bash
git clone https://github.com/yourusername/pid-pmsm.git
cd pid-pmsm
```

---

## Usage

### Run the Simulation

```bash
python pmsm_pid_control.py
```

### Output

The program will:
1. Simulate the PMSM speed control system
2. Generate `pmsm_pid_speed_control.png` with response plots
3. Display performance metrics in the console

### Sample Output

```
Image saved: pmsm_pid_speed_control.png

============================================================
PMSM PID Speed Control Simulation Results
============================================================
Target Speed:           1500 RPM
Speed at 3s:           1500.00 RPM
Max Speed (0-10s):     1501.36 RPM
Overshoot:             0.09%
Final Speed (10s):     1500.00 RPM
Speed Variation (5-10s): 0.00 RPM

[OK] Speed at 3s (1500.0 RPM) is within 2% of target
[OK] System is stable from 5-10s (variation: 0.00 RPM)
============================================================
```

---

## Results

### Speed Response

![Speed Control Response](pmsm_pid_speed_control.png)

The plot shows:
- **Red dashed line**: Reference speed (1500 RPM)
- **Blue solid line**: Actual motor speed
- **Orange dashed vertical line**: 3-second mark
- **Green horizontal band**: ±2% tolerance zone

### Performance Analysis

| Metric | Target | Achieved |
|--------|--------|----------|
| Rise Time to 1500 RPM | < 3s | ✅ < 3s |
| Overshoot | < 2% | ✅ 0.09% |
| Steady-State Error | < 2% | ✅ 0% |
| 5-10s Stability | < 2 RPM variation | ✅ 0 RPM |

---

## Project Structure

```
pid-pmsm/
├── pmsm_pid_control.py      # Main simulation code
├── pmsm_pid_speed_control.png  # Simulation results plot
└── README.md                # This file
```

---

## How It Works

### Simulation Loop

```
1. Set reference speed = 1500 RPM
2. For each time step:
   ├── Calculate error = ω_ref - ω_actual
   ├── Compute PID output: Vq = Kp·e + Ki·∫e·dt + Kd·de/dt
   ├── Update q-axis current: Iq += (Vq/Ra - Iq)/τe · dt
   ├── Update motor speed: ω += (Kt·Iq - B·ω)/J · dt
   └── Record ω (RPM) and Vq
3. Plot results and print metrics
```

### Block Diagram

```
        ┌─────────┐      ┌──────────┐      ┌─────────────┐
Ref ──►│   PID   │─────►│ Electrical│─────►│ Mechanical  │─────► ω
       │Control  │  Vq  │  Model    │  Iq  │   Model     │
       └─────────┘      └───────────┘      └─────────────┘
           ▲                                        │
           │                                        │
           └────────────── Feedback ────────────────┘
```

---

## References

- [PMSM Control Theory](https://en.wikipedia.org/wiki/Synchronous_motor) - Wikipedia
- [PID Controller](https://en.wikipedia.org/wiki/PID_controller) - Wikipedia
- [Python Control Systems Library](https://python-control.readthedocs.io/) - python-control

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## Author

Created with Claude Code

<div align="center">

*"Simulation is the key to understanding complex control systems"*

</div>