"""
PMSM PID Speed Control Simulation
=================================
This script simulates a Permanent Magnet Synchronous Motor (PMSM) speed control
system using a PID controller. The goal is to achieve 1500 RPM from 0 and
maintain stability for 10 seconds, with speed stable at 1500 RPM by 3 seconds.

Requirements:
- Y-axis range: 0-2500 RPM
- Reference is constant 1500 RPM (straight line)
- Actual speed should track smoothly without sharp corners
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt

# PMSM Parameters
# =============================================================================
Ra = 1.0         # Stator resistance (Ohm)
Kt = 0.5         # Torque constant (N.m/A)
Jm = 0.1         # Moment of inertia (kg.m^2)
B = 0.05         # Damping coefficient (N.m.s/rad)


def simulate_pmsm_pid(target_rpm=1500, sim_time=10.0, dt=0.005,
                      Kp=20.0, Ki=80.0, Kd=2.0):
    """
    Simulate PMSM with PID speed control.
    """
    target_omega = target_rpm * 2 * np.pi / 60  # rad/s
    t = np.arange(0, sim_time, dt)
    n_steps = len(t)

    omega_rpm = np.zeros(n_steps)
    ref_rpm = np.zeros(n_steps)
    Vq_array = np.zeros(n_steps)

    # State variables
    Iq = 0.0
    omega = 0.0

    prev_error = 0.0
    integral = 0.0

    for i in range(n_steps):
        # Constant reference (straight line at 1500 RPM)
        ref_rpm[i] = target_rpm
        ref_omega = target_omega

        error = ref_omega - omega

        # PI controller with anti-windup
        P_term = Kp * error
        integral += Ki * error * dt
        integral = np.clip(integral, -20, 20)
        D_term = Kd * (error - prev_error) / dt if dt > 0 else 0

        Vq = P_term + integral + D_term
        Vq = np.clip(Vq, 0, 40)

        Vq_array[i] = Vq

        # Electrical dynamics
        tau_e = 0.05
        Iq += ((Vq / Ra) - Iq) / tau_e * dt

        # Mechanical dynamics
        dOmega = (Kt * Iq - B * omega) / Jm
        omega += dOmega * dt

        omega = max(0, omega)

        omega_rpm[i] = omega * 60 / (2 * np.pi)
        prev_error = error

    return t, omega_rpm, ref_rpm, Vq_array


def plot_results(t, omega_rpm, ref_rpm, Vq_array):
    """
    Plot the simulation results.
    """
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    target = 1500

    ax1.plot(t, ref_rpm, 'r--', linewidth=2, label='Reference (1500 RPM)')
    ax1.plot(t, omega_rpm, 'b-', linewidth=2, label='Actual Speed')
    ax1.set_xlabel('Time (s)', fontsize=12)
    ax1.set_ylabel('Speed (RPM)', fontsize=12)
    ax1.set_title('PMSM PID Speed Control Response', fontsize=14, fontweight='bold')
    ax1.legend(loc='lower right', fontsize=11)
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim([0, 10])
    ax1.set_ylim([0, 2500])

    # Check speed at 3 seconds
    idx_3s = np.argmin(np.abs(t - 3.0))
    speed_at_3s = omega_rpm[idx_3s]
    ax1.axvline(x=3.0, color='orange', linestyle='--', linewidth=2, alpha=0.8)
    ax1.axhline(y=1500, color='gray', linestyle=':', linewidth=1, alpha=0.5)
    ax1.annotate(f'Speed at 3s: {speed_at_3s:.1f} RPM',
                 xy=(3.0, speed_at_3s),
                 xytext=(3.2, speed_at_3s + 100),
                 fontsize=10,
                 color='orange')

    # Check stability
    idx_5s = np.argmin(np.abs(t - 5.0))
    final_rpm = omega_rpm[-1]
    max_5_10s = omega_rpm[idx_5s:].max()
    min_5_10s = omega_rpm[idx_5s:].min()
    stability_5_10s = max_5_10s - min_5_10s

    ax2.plot(t, Vq_array, 'g-', linewidth=1.5)
    ax2.set_xlabel('Time (s)', fontsize=12)
    ax2.set_ylabel('Voltage Vq (V)', fontsize=12)
    ax2.set_title('PID Controller Output Voltage', fontsize=14, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim([0, 10])

    plt.tight_layout()
    plt.savefig('pmsm_pid_speed_control.png', dpi=150, bbox_inches='tight')
    plt.close()
    print("Image saved: pmsm_pid_speed_control.png")

    print("\n" + "="*60)
    print("PMSM PID Speed Control Simulation Results")
    print("="*60)

    overshoot = ((omega_rpm.max() - target) / target) * 100 if omega_rpm.max() > target else 0

    print(f"Target Speed:           {target} RPM")
    print(f"Speed at 3s:           {speed_at_3s:.2f} RPM")
    print(f"Max Speed (0-10s):     {omega_rpm.max():.2f} RPM")
    print(f"Overshoot:             {overshoot:.2f}%")
    print(f"Final Speed (10s):     {final_rpm:.2f} RPM")
    print(f"Speed Variation (5-10s): {stability_5_10s:.2f} RPM")

    if abs(speed_at_3s - target) <= 30:
        print(f"\n[OK] Speed at 3s ({speed_at_3s:.1f} RPM) is within 2% of target")
    else:
        print(f"\n[FAIL] Speed at 3s ({speed_at_3s:.1f} RPM) exceeds 2% tolerance")

    if stability_5_10s <= 2:
        print(f"[OK] System is stable from 5-10s (variation: {stability_5_10s:.2f} RPM)")
    else:
        print(f"[FAIL] System is unstable from 5-10s (variation: {stability_5_10s:.2f} RPM)")

    print("="*60)


if __name__ == "__main__":
    t, omega_rpm, ref_rpm, Vq_array = simulate_pmsm_pid(
        target_rpm=1500,
        sim_time=10.0,
        dt=0.005,
        Kp=20.0,
        Ki=80.0,
        Kd=2.0
    )

    plot_results(t, omega_rpm, ref_rpm, Vq_array)
