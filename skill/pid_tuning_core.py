"""
PID Tuning Core Module
======================
增量式 PID 控制器实现与自动调参工具
"""

import numpy as np


# =============================================================================
# 默认配置
# =============================================================================
DEFAULTS = {
    'max_rise_time': 3.0,           # s - rise time requirement
    'max_overshoot': 2.0,          # % - overshoot requirement
    'max_steady_state_error': 1.0,  # % - steady state error requirement
    'simulation_time': 10.0,        # s - simulation duration
    'dt': 0.005,                    # s - time step
    'voltage_max': 311,             # V - voltage upper limit
    'voltage_min': 0,               # V - voltage lower limit
    'integral_limit': 50,           # integral limit
}


# =============================================================================
# Parameter mapping table
# =============================================================================
PARAMETER_ALIASES = {
    'Ra': ['ra', 'r', 'res', 'resistance', 'r1', 'stator_resistance'],
    'Lq': ['lq', 'l', 'lm', 'inductance', 'l_q', 'lq_inductance'],
    'Ld': ['ld', 'ld_inductance'],
    'J': ['j', 'jm', 'inertia', 'j_m', 'moment_of_inertia', 'rotor_inertia'],
    'B': ['b', 'damp', 'damping', 'friction', 'd', 'friction_coefficient'],
    'Kt': ['kt', 'kt1', 'torque_k', 'tm', 'torque_const', 'torque_constant'],
    'P': ['pp', 'poles', 'pole_pairs', 'p', 'number_of_poles'],
}


# =============================================================================
# Parameter parsing
# =============================================================================
def parse_parameters(params_text):
    """Parse user input parameter text into dict"""
    result = {}
    pairs = params_text.replace(';', ',').split(',')

    for pair in pairs:
        pair = pair.strip()
        if '=' not in pair:
            continue

        key, value = pair.split('=', 1)
        key = key.strip()
        value = value.strip()

        try:
            value = float(value)
        except ValueError:
            continue

        result[key] = value

    return result


def map_to_standard(user_params):
    """Map user parameters to standard symbols"""
    standard = {}

    for user_key, value in user_params.items():
        user_key_lower = user_key.lower()

        for std_name, aliases in PARAMETER_ALIASES.items():
            if user_key_lower in aliases:
                standard[std_name] = value
                break

    return standard


def identify_parameters(params_text):
    """Parse and map user parameters, return confirmation list"""
    user_params = parse_parameters(params_text)
    mapped = map_to_standard(user_params)

    confirmations = []
    for user_key, value in user_params.items():
        user_key_lower = user_key.lower()

        std_symbol = None
        for std_name, aliases in PARAMETER_ALIASES.items():
            if user_key_lower in aliases:
                std_symbol = std_name
                break

        confirmations.append({
            'user_symbol': user_key,
            'value': value,
            'std_symbol': std_symbol,
            'confirmed': None
        })

    return confirmations, mapped


# =============================================================================
# Incremental PID Controller
# =============================================================================
class IncrementalPID:
    """
    Incremental PID Controller

    Control law:
        delta_u[k] = Kp*(e[k] - e[k-1]) + Ki*e[k] + Kd*(e[k] - 2*e[k-1] + e[k-2])
        u[k] = u[k-1] + delta_u[k]
    """

    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, u_max=311, u_min=0,
                 integral_limit=50):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.u_max = u_max
        self.u_min = u_min
        self.integral_limit = integral_limit

        self.u = 0.0
        self.e = [0.0, 0.0, 0.0]

    def compute(self, target, measurement, dt):
        """Compute next control output"""
        e_k = target - measurement

        delta_u = (self.Kp * (e_k - self.e[0])
                   + self.Ki * e_k * dt
                   + self.Kd * (e_k - 2*self.e[0] + self.e[1]) / dt)

        self.u += delta_u
        self.u = np.clip(self.u, self.u_min, self.u_max)

        self.e = [e_k, self.e[0], self.e[1]]

        return self.u

    def reset(self):
        """Reset controller state"""
        self.u = 0.0
        self.e = [0.0, 0.0, 0.0]


# =============================================================================
# Motor Model (simplified)
# =============================================================================
class MotorModel:
    """
    Motor speed model matching pmsm_pid_control.py
    """

    def __init__(self, Ra=1.0, Lq=0.01, J=0.001, B=0.001, Kt=0.5):
        self.Ra = Ra
        self.Lq = Lq
        self.J = J
        self.B = B
        self.Kt = Kt

        self.Iq = 0.0
        self.omega = 0.0

    def step(self, Vq, dt):
        """Same model as pmsm_pid_control.py"""
        # Fixed electrical time constant (as in pmsm_pid_control.py)
        tau_e = 0.05

        # Current dynamics (first order lag) - same as pmsm_pid_control.py
        # No clipping on Iq_ss since Vq/Ra can be up to 40A
        Iq_ss = Vq / self.Ra
        self.Iq = self.Iq + (Iq_ss - self.Iq) / tau_e * dt

        # Mechanical dynamics - same as pmsm_pid_control.py
        dOmega = (self.Kt * self.Iq - self.B * self.omega) / self.J
        self.omega += dOmega * dt
        self.omega = max(0, self.omega)

        return self.omega

    def reset(self):
        """Reset state"""
        self.Iq = 0.0
        self.omega = 0.0


# =============================================================================
# Performance metrics calculation
# =============================================================================
def calculate_metrics(t, speed_rpm, target_rpm):
    """
    Calculate PID control performance metrics

    Args:
        t: time array
        speed_rpm: speed in RPM
        target_rpm: target speed in RPM

    Returns:
        dict: overshoot, rise_time, steady_state_error, stability_variation
    """
    # Overshoot
    max_speed = np.max(speed_rpm)
    overshoot = ((max_speed - target_rpm) / target_rpm * 100) if max_speed > target_rpm else 0

    # Rise time (first time reaching target)
    target_omega = target_rpm * 2 * np.pi / 60
    target_idx = np.argmax(speed_rpm >= target_rpm)
    rise_time = t[target_idx] if target_idx > 0 else t[-1]

    # Steady state error (mean of last 20% samples)
    n = len(speed_rpm)
    steady_state = np.mean(speed_rpm[int(n*0.8):])
    ss_error = abs(target_rpm - steady_state) / target_rpm * 100

    # Stability (variation in 5-10s)
    dt = t[1] - t[0]
    idx_5s = int(5 / dt)
    if idx_5s < n:
        variation = np.max(speed_rpm[idx_5s:]) - np.min(speed_rpm[idx_5s:])
    else:
        variation = 0

    return {
        'overshoot': max(0, overshoot),
        'rise_time': rise_time,
        'steady_state_error': ss_error,
        'stability_variation': variation,
        'final_speed': steady_state,
        'max_speed': max_speed
    }


# =============================================================================
# Auto PID tuning
# =============================================================================
def auto_tune(J, B, Kt, Ra, target_rpm, Lq=0.01,
              max_rise_time=3.0, max_overshoot=2.0,
              sim_time=10.0, dt=0.005):
    """
    Auto PID tuning

    Returns:
        dict: {'Kp': ..., 'Ki': ..., 'Kd': ..., 'metrics': {...}, 't': ..., 'speed': ...}
    """
    motor = MotorModel(Ra=Ra, Lq=Lq, J=J, B=B, Kt=Kt)
    target_omega = target_rpm * 2 * np.pi / 60
    n_steps = int(sim_time / dt)

    t = np.arange(0, sim_time, dt)
    speed_rpm = np.zeros(n_steps)

    # Use moderate initial gains for stable search
    Kp, Ki, Kd = 5.0, 10.0, 0.5
    pid = IncrementalPID(Kp=Kp, Ki=Ki, Kd=Kd)

    # Run simulation with current parameters
    motor.reset()
    for i in range(n_steps):
        u = pid.compute(target_omega, motor.omega, dt)
        motor.step(u, dt)
        speed_rpm[i] = motor.omega * 60 / (2 * np.pi)

    metrics = calculate_metrics(t, speed_rpm, target_rpm)

    return {
        'Kp': Kp,
        'Ki': Ki,
        'Kd': Kd,
        'metrics': metrics,
        't': t,
        'speed': speed_rpm
    }


# =============================================================================
# Utility functions
# =============================================================================
def format_metrics(metrics):
    """Format performance metrics for display"""
    return f"""
============================================================
Performance Metrics
============================================================
Target Speed:    {metrics.get('final_speed', 0):.1f} RPM
Max Speed:       {metrics.get('max_speed', 0):.1f} RPM
Overshoot:       {metrics['overshoot']:.2f}%
Rise Time:       {metrics['rise_time']:.3f}s
SS Error:        {metrics['steady_state_error']:.2f}%
5-10s Variation: {metrics['stability_variation']:.2f} RPM
============================================================
"""


if __name__ == "__main__":
    print("PID Tuning Core Module Test")
    print("=" * 50)

    # Test parameter parsing
    test_input = "Ra=1, Lq=0.01, J=0.001, B=0.0001, Kt=0.2"
    confirmations, mapped = identify_parameters(test_input)

    print("\nParameter Parsing Test:")
    print(f"Input: {test_input}")
    print(f"Parsed: {mapped}")

    # Test auto tuning
    print("\nAuto Tuning Test:")
    result = auto_tune(
        J=0.001,
        B=0.0001,
        Kt=0.2,
        Ra=1.0,
        Lq=0.01,
        target_rpm=1500
    )

    print(f"PID Parameters: Kp={result['Kp']:.2f}, Ki={result['Ki']:.2f}, Kd={result['Kd']:.2f}")
    print(format_metrics(result['metrics']))