# Operation Shaheen — Theory Module 4: Python for Drone Engineering

> **Python is your laboratory bench. C is your flight controller. Know when to use which.**

---

## When to Use Python vs. C

| Task | Language | Why |
|---|---|---|
| Control loop on STM32 | C | Hard real-time, microsecond determinism |
| Simulate dynamics before hardware | Python | Fast iteration, rich libraries |
| Plot and analyze flight logs | Python | matplotlib, pandas, numpy |
| Train computer vision models | Python | PyTorch, OpenCV |
| Ground station / telemetry viewer | Python | Serial, networking, GUI |
| Path planning algorithms | Python (prototype) → C (deploy) | Prototype fast, optimize later |
| Communicate with drone via MAVLink | Python (DroneKit/pymavlink) | High-level scripting |
| Hardware-in-the-loop testing | Python + SITL | Test autopilot without flying |

---

## Part 1: Drone Simulation in Python

### 1.1 — 2D Quadcopter Simulator (Start Here)

Before you simulate in 3D, build intuition in 2D (pitch axis only).

```python
"""
2D Quadcopter Pitch Simulator
State: [theta, theta_dot]  (pitch angle and rate)
Input: torque tau (from differential motor thrust)
"""
import numpy as np
import matplotlib.pyplot as plt

class QuadSim2D:
    def __init__(self):
        self.I_yy = 0.01      # pitch moment of inertia (kg·m²)
        self.dt = 0.001        # 1kHz simulation
        self.theta = 0.0       # pitch angle (rad)
        self.theta_dot = 0.0   # pitch rate (rad/s)
        self.g = 9.81
        self.mass = 0.5        # kg
    
    def step(self, torque):
        """Advance simulation by one timestep."""
        # Euler's equation (simplified, no gyroscopic coupling)
        theta_ddot = torque / self.I_yy
        
        # Integrate (RK4 is better, but Euler is fine for 1kHz)
        self.theta_dot += theta_ddot * self.dt
        self.theta += self.theta_dot * self.dt
        
        return self.theta, self.theta_dot
    
    def get_sensor_readings(self, noise=True):
        """Simulate IMU readings with realistic noise."""
        gyro = self.theta_dot
        accel_angle = self.theta  # simplified: atan2(ax, az) ≈ theta for small angles
        
        if noise:
            gyro += np.random.normal(0, 0.05 * np.pi/180)  # 0.05 °/s noise
            accel_angle += np.random.normal(0, 0.5 * np.pi/180)  # 0.5° noise
        
        return gyro, accel_angle


class PIDController:
    def __init__(self, kp, ki, kd, dt, output_limit=1.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0
        self.output_limit = output_limit
    
    def update(self, setpoint, measurement):
        error = setpoint - measurement
        
        # P
        p_term = self.kp * error
        
        # I with anti-windup
        self.integral += error * self.dt
        self.integral = np.clip(self.integral, 
                                -self.output_limit / max(self.ki, 1e-6),
                                 self.output_limit / max(self.ki, 1e-6))
        i_term = self.ki * self.integral
        
        # D on measurement (not error) to avoid derivative kick
        d_term = -self.kd * (measurement - self.prev_error) / self.dt  
        # Note: for proper "D on measurement", track prev_measurement separately
        
        self.prev_error = measurement
        
        output = p_term + i_term + d_term
        return np.clip(output, -self.output_limit, self.output_limit)


# --- Simulation ---
sim = QuadSim2D()
pid = PIDController(kp=5.0, ki=0.5, kd=0.3, dt=0.001, output_limit=0.5)

time_steps = 5000  # 5 seconds
t = np.zeros(time_steps)
theta_log = np.zeros(time_steps)
command_log = np.zeros(time_steps)
setpoint = 15.0 * np.pi / 180  # 15 degree step command

for i in range(time_steps):
    t[i] = i * sim.dt
    gyro, accel = sim.get_sensor_readings(noise=True)
    torque = pid.update(setpoint, sim.theta)
    sim.step(torque)
    theta_log[i] = sim.theta * 180 / np.pi
    command_log[i] = torque

# Plot
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))
ax1.plot(t, theta_log, label='Actual')
ax1.axhline(y=15, color='r', linestyle='--', label='Setpoint')
ax1.set_ylabel('Pitch (°)')
ax1.legend()
ax1.set_title('PID Step Response')

ax2.plot(t, command_log)
ax2.set_ylabel('Torque Command')
ax2.set_xlabel('Time (s)')
plt.tight_layout()
plt.show()
```

**Exercises:**
1. Run this code. Tune Kp, Ki, Kd until you get <5% overshoot and <1s settling time.
2. Add a disturbance torque at t=2.5s (simulating a wind gust). How does the controller respond?
3. Add sensor delay (use measurements from 5 timesteps ago). See how it affects stability.
4. Implement the complementary filter in the loop (instead of using true angle).

---

### 1.2 — Full 6-DOF Quadcopter Simulator

```python
"""
6-DOF Quadcopter Simulator
States: [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r] (12 states)
Inputs: [f1, f2, f3, f4] (4 motor forces)
"""
import numpy as np
from scipy.spatial.transform import Rotation

class QuadSim6DOF:
    def __init__(self):
        # Physical parameters
        self.mass = 0.5          # kg
        self.g = 9.81            # m/s²
        self.arm_length = 0.175  # m (distance from center to motor)
        self.k_thrust = 1e-5     # thrust coefficient (N/(rad/s)²)
        self.k_torque = 1e-7     # torque coefficient (Nm/(rad/s)²)
        
        # Inertia tensor (diagonal for symmetric quad)
        self.Ixx = 0.0082
        self.Iyy = 0.0082
        self.Izz = 0.0149
        self.I = np.diag([self.Ixx, self.Iyy, self.Izz])
        self.I_inv = np.linalg.inv(self.I)
        
        # State: [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
        self.state = np.zeros(12)
        self.dt = 0.001  # 1kHz
    
    def motor_mixing(self, motor_forces):
        """Convert 4 motor forces to total thrust + 3 torques."""
        f1, f2, f3, f4 = motor_forces  # FR, RR, RL, FL (X-config)
        L = self.arm_length * np.sin(np.pi/4)  # effective arm for X-config
        
        thrust = f1 + f2 + f3 + f4
        tau_x = L * (-f1 - f2 + f3 + f4)   # roll torque
        tau_y = L * (-f1 + f2 + f3 - f4)    # pitch torque
        tau_z = self.k_torque/self.k_thrust * (-f1 + f2 - f3 + f4)  # yaw torque
        
        return thrust, np.array([tau_x, tau_y, tau_z])
    
    def derivatives(self, state, thrust, torques):
        """Compute state derivatives (equations of motion)."""
        x, y, z, vx, vy, vz, phi, theta, psi, p, q, r = state
        
        # Rotation matrix (body to world)
        R = Rotation.from_euler('ZYX', [psi, theta, phi]).as_matrix()
        
        # Translational dynamics (Newton's law in world frame)
        thrust_world = R @ np.array([0, 0, thrust])
        gravity = np.array([0, 0, -self.mass * self.g])
        accel = (thrust_world + gravity) / self.mass
        
        # Rotational dynamics (Euler's equations in body frame)
        omega = np.array([p, q, r])
        omega_dot = self.I_inv @ (torques - np.cross(omega, self.I @ omega))
        
        # Euler angle rates from body rates
        # (This has singularity at theta=±90°, use quaternions for real implementation)
        cphi, sphi = np.cos(phi), np.sin(phi)
        ctheta, ttheta = np.cos(theta), np.tan(theta)
        
        phi_dot = p + (q * sphi + r * cphi) * ttheta
        theta_dot = q * cphi - r * sphi
        psi_dot = (q * sphi + r * cphi) / ctheta
        
        return np.array([
            vx, vy, vz,           # position derivatives = velocity
            accel[0], accel[1], accel[2],  # velocity derivatives = acceleration
            phi_dot, theta_dot, psi_dot,    # angle derivatives
            omega_dot[0], omega_dot[1], omega_dot[2]  # rate derivatives
        ])
    
    def step(self, motor_forces):
        """RK4 integration step."""
        thrust, torques = self.motor_mixing(motor_forces)
        
        k1 = self.derivatives(self.state, thrust, torques)
        k2 = self.derivatives(self.state + 0.5*self.dt*k1, thrust, torques)
        k3 = self.derivatives(self.state + 0.5*self.dt*k2, thrust, torques)
        k4 = self.derivatives(self.state + self.dt*k3, thrust, torques)
        
        self.state += (self.dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
        return self.state.copy()
    
    def get_imu(self, noise=True):
        """Simulate IMU readings in body frame."""
        phi, theta, psi = self.state[6:9]
        p, q, r = self.state[9:12]
        
        # Accelerometer: measures specific force (thrust/mass in body frame)
        # In hover: [0, 0, g] (gravity appears as upward acceleration)
        R = Rotation.from_euler('ZYX', [psi, theta, phi]).as_matrix()
        accel_body = R.T @ np.array([0, 0, self.g])  # gravity in body frame
        
        gyro = np.array([p, q, r])
        
        if noise:
            accel_body += np.random.normal(0, 0.05, 3)  # m/s² noise
            gyro += np.random.normal(0, 0.001, 3)        # rad/s noise
        
        return accel_body, gyro


# --- Usage Example ---
sim = QuadSim6DOF()
hover_thrust = sim.mass * sim.g / 4  # each motor provides 1/4 of weight

# Hover test
for i in range(5000):
    motors = [hover_thrust] * 4  # equal thrust = hover
    state = sim.step(motors)

print(f"After 5s hover: z={state[2]:.3f}m (should be ~0)")
print(f"Drift: x={state[0]:.4f}, y={state[1]:.4f}")
```

**Exercises:**
1. Run hover test. Verify altitude stays at 0 (it should, no perturbation).
2. Add a small perturbation to motor 1 (+5% thrust). Watch the quad drift — this is why you need a controller.
3. Implement your cascaded PID from Project 4 in this sim. Get it to hover stably.
4. Command a step roll of 10°. Watch the lateral translation. This is how a quadcopter moves.
5. Add wind as a constant force perturbation. Tune your controller to reject it.

---

### 1.3 — Fixed-Wing Simulator (for Project 5)

```python
"""
Simplified Fixed-Wing 3-DOF Longitudinal Simulator
States: [V, gamma, h, x]  (airspeed, flight path angle, altitude, downrange)
Inputs: [thrust, alpha]    (engine thrust, angle of attack → relates to elevator)
"""
import numpy as np

class FixedWingSim:
    def __init__(self):
        self.mass = 2.0       # kg
        self.g = 9.81
        self.S = 0.3          # wing area (m²)
        self.rho = 1.225      # air density
        self.dt = 0.01        # 100Hz sim rate (slower dynamics than quad)
        
        # Aerodynamic coefficients (typical small UAV)
        self.CL0 = 0.25       # lift at zero AoA
        self.CLa = 5.0        # lift curve slope (per radian)
        self.CD0 = 0.03       # parasitic drag
        self.K = 0.04         # induced drag factor (CD = CD0 + K*CL²)
        
        # State: [V, gamma, h, x]
        self.V = 15.0         # initial airspeed (m/s)
        self.gamma = 0.0      # flight path angle (rad)
        self.h = 100.0        # altitude (m)
        self.x = 0.0          # downrange distance (m)
    
    def aero_forces(self, V, alpha):
        """Compute lift and drag."""
        q = 0.5 * self.rho * V**2  # dynamic pressure
        CL = self.CL0 + self.CLa * alpha
        CD = self.CD0 + self.K * CL**2
        L = q * self.S * CL
        D = q * self.S * CD
        return L, D
    
    def step(self, thrust, alpha):
        """Advance by one timestep. Returns state."""
        L, D = self.aero_forces(self.V, alpha)
        
        # Equations of motion (point-mass, longitudinal plane)
        V_dot = (thrust - D) / self.mass - self.g * np.sin(self.gamma)
        gamma_dot = (L - self.mass * self.g * np.cos(self.gamma)) / (self.mass * self.V)
        h_dot = self.V * np.sin(self.gamma)
        x_dot = self.V * np.cos(self.gamma)
        
        self.V += V_dot * self.dt
        self.gamma += gamma_dot * self.dt
        self.h += h_dot * self.dt
        self.x += x_dot * self.dt
        
        return np.array([self.V, self.gamma, self.h, self.x])
    
    def trim(self, V_target):
        """Find trim conditions for level flight at given airspeed."""
        # At trim: V_dot = 0, gamma_dot = 0, gamma = 0
        # L = mg → CL_trim = mg / (0.5 * rho * V² * S)
        q = 0.5 * self.rho * V_target**2
        CL_trim = self.mass * self.g / (q * self.S)
        alpha_trim = (CL_trim - self.CL0) / self.CLa
        CD_trim = self.CD0 + self.K * CL_trim**2
        thrust_trim = q * self.S * CD_trim  # thrust = drag at trim
        
        return thrust_trim, alpha_trim


# --- TECS Implementation ---
class TECS:
    """Simplified Total Energy Control System."""
    
    def __init__(self, mass, g=9.81):
        self.mass = mass
        self.g = g
        
        # Gains
        self.kT_throttle = 0.1   # throttle response to total energy error
        self.kD_throttle = 0.05  # throttle damping
        self.kT_pitch = 0.08     # pitch response to energy distribution error
        self.kD_pitch = 0.04     # pitch damping
        
        self.prev_energy_error = 0
        self.prev_dist_error = 0
    
    def update(self, V, h, V_cmd, h_cmd, dt):
        """Compute throttle and pitch commands."""
        # Energy errors
        KE_error = 0.5 * self.mass * (V_cmd**2 - V**2)
        PE_error = self.mass * self.g * (h_cmd - h)
        
        total_energy_error = KE_error + PE_error
        energy_dist_error = PE_error - KE_error  # positive = need more altitude, less speed
        
        # PD control on energy
        d_total = (total_energy_error - self.prev_energy_error) / dt
        d_dist = (energy_dist_error - self.prev_dist_error) / dt
        
        throttle_cmd = self.kT_throttle * total_energy_error + self.kD_throttle * d_total
        pitch_cmd = self.kT_pitch * energy_dist_error + self.kD_pitch * d_dist
        
        self.prev_energy_error = total_energy_error
        self.prev_dist_error = energy_dist_error
        
        return np.clip(throttle_cmd, 0, 10), np.clip(pitch_cmd, -0.2, 0.2)


# --- Test: Altitude change with TECS ---
sim = FixedWingSim()
thrust_trim, alpha_trim = sim.trim(15.0)
tecs = TECS(sim.mass)

# Command: climb from 100m to 150m while maintaining 15 m/s
time_steps = 3000  # 30 seconds
logs = {'t': [], 'h': [], 'V': [], 'thrust': [], 'alpha': []}

for i in range(time_steps):
    t = i * sim.dt
    h_cmd = 150.0 if t > 2.0 else 100.0  # step command at t=2s
    V_cmd = 15.0
    
    throttle_adj, pitch_adj = tecs.update(sim.V, sim.h, V_cmd, h_cmd, sim.dt)
    thrust = thrust_trim + throttle_adj
    alpha = alpha_trim + pitch_adj
    
    state = sim.step(thrust, alpha)
    
    logs['t'].append(t)
    logs['h'].append(state[2])
    logs['V'].append(state[0])
    logs['thrust'].append(thrust)
    logs['alpha'].append(np.degrees(alpha))

# Plot results
# (Add your matplotlib plotting code here)
```

**Exercises:**
1. Run the TECS altitude step. Plot altitude and airspeed. Does speed dip during the climb? (It should, slightly — TECS manages this tradeoff.)
2. Tune TECS gains. What happens with aggressive gains? (Airspeed oscillates during climbs.)
3. Add L1 lateral guidance in 2D. Simulate a waypoint circuit. Plot the ground track.
4. Add wind (constant 3 m/s from the north). See how L1 guidance handles crosswind.

---

## Part 2: Flight Log Analysis

### 2.1 — Serial Data Logger and Plotter

```python
"""
Real-time serial plotter for STM32 sensor data.
Expects CSV format: timestamp_ms,ax,ay,az,gx,gy,gz
"""
import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

class RealtimePlotter:
    def __init__(self, port='/dev/ttyUSB0', baud=115200, window=500):
        self.ser = serial.Serial(port, baud, timeout=0.01)
        self.window = window
        self.data = {
            'ax': deque(maxlen=window), 'ay': deque(maxlen=window),
            'az': deque(maxlen=window), 'gx': deque(maxlen=window),
            'gy': deque(maxlen=window), 'gz': deque(maxlen=window),
        }
        self.fig, self.axes = plt.subplots(2, 3, figsize=(14, 6))
        self.lines = {}
        
        names = ['ax', 'ay', 'az', 'gx', 'gy', 'gz']
        titles = ['Accel X', 'Accel Y', 'Accel Z', 'Gyro X', 'Gyro Y', 'Gyro Z']
        units = ['g', 'g', 'g', '°/s', '°/s', '°/s']
        
        for idx, (name, title, unit) in enumerate(zip(names, titles, units)):
            ax = self.axes[idx // 3][idx % 3]
            self.lines[name], = ax.plot([], [])
            ax.set_title(title)
            ax.set_ylabel(unit)
            ax.set_xlim(0, window)
    
    def update(self, frame):
        while self.ser.in_waiting:
            try:
                line = self.ser.readline().decode().strip()
                parts = line.split(',')
                if len(parts) == 7:
                    _, ax, ay, az, gx, gy, gz = [float(x) for x in parts]
                    for name, val in zip(self.data.keys(), [ax, ay, az, gx, gy, gz]):
                        self.data[name].append(val)
            except (ValueError, UnicodeDecodeError):
                continue
        
        for name, line in self.lines.items():
            if len(self.data[name]) > 0:
                y = list(self.data[name])
                line.set_data(range(len(y)), y)
                ax = line.axes
                ax.set_xlim(0, max(len(y), 10))
                ax.set_ylim(min(y) - 0.1, max(y) + 0.1)
        
        return self.lines.values()
    
    def run(self):
        ani = animation.FuncAnimation(self.fig, self.update, interval=20, blit=False)
        plt.tight_layout()
        plt.show()

# Usage:
# plotter = RealtimePlotter(port='COM3', baud=115200)  # Windows
# plotter = RealtimePlotter(port='/dev/ttyUSB0', baud=115200)  # Linux
# plotter.run()
```

---

### 2.2 — Post-Flight Analysis Tools

```python
"""
Flight log analysis toolkit.
Reads CSV logs from your flight controller and generates analysis plots.
"""
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import signal

class FlightLogAnalyzer:
    def __init__(self, filepath):
        self.df = pd.read_csv(filepath)
        # Expected columns: time_ms, ax, ay, az, gx, gy, gz, 
        #   roll, pitch, yaw, motor1, motor2, motor3, motor4,
        #   roll_sp, pitch_sp, yaw_sp, throttle
    
    def plot_attitude(self):
        """Plot actual vs. commanded attitude."""
        fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        t = self.df['time_ms'] / 1000
        
        for i, (angle, sp) in enumerate([
            ('roll', 'roll_sp'), ('pitch', 'pitch_sp'), ('yaw', 'yaw_sp')
        ]):
            axes[i].plot(t, self.df[angle], label='Actual', alpha=0.8)
            axes[i].plot(t, self.df[sp], label='Setpoint', linestyle='--')
            axes[i].set_ylabel(f'{angle.title()} (°)')
            axes[i].legend()
        
        axes[-1].set_xlabel('Time (s)')
        plt.suptitle('Attitude Tracking Performance')
        plt.tight_layout()
        return fig
    
    def analyze_step_response(self, axis='pitch'):
        """Compute step response metrics for a given axis."""
        t = self.df['time_ms'].values / 1000
        actual = self.df[axis].values
        setpoint = self.df[f'{axis}_sp'].values
        
        # Find step (largest change in setpoint)
        sp_diff = np.abs(np.diff(setpoint))
        step_idx = np.argmax(sp_diff)
        step_size = setpoint[step_idx + 1] - setpoint[step_idx]
        
        if abs(step_size) < 1.0:
            print("No significant step found in data.")
            return
        
        # Analyze from step onwards
        t_step = t[step_idx:]
        y_step = actual[step_idx:]
        sp_final = setpoint[step_idx + 1]
        
        # Metrics
        overshoot_val = np.max(y_step) if step_size > 0 else np.min(y_step)
        overshoot_pct = abs((overshoot_val - sp_final) / step_size) * 100
        
        # Settling time (within 2% of final value)
        settled = np.abs(y_step - sp_final) < 0.02 * abs(step_size)
        if np.any(settled):
            settle_idx = np.where(settled)[0]
            # Find first index where it stays settled
            for idx in settle_idx:
                if np.all(settled[idx:min(idx+50, len(settled))]):
                    settling_time = t_step[idx] - t_step[0]
                    break
            else:
                settling_time = float('inf')
        else:
            settling_time = float('inf')
        
        # Rise time (10% to 90%)
        y_norm = (y_step - y_step[0]) / step_size
        t10 = t_step[np.argmax(y_norm >= 0.1)] - t_step[0] if np.any(y_norm >= 0.1) else None
        t90 = t_step[np.argmax(y_norm >= 0.9)] - t_step[0] if np.any(y_norm >= 0.9) else None
        rise_time = (t90 - t10) if (t10 is not None and t90 is not None) else None
        
        print(f"--- {axis.title()} Step Response Analysis ---")
        print(f"Step size: {step_size:.1f}°")
        print(f"Overshoot: {overshoot_pct:.1f}%")
        print(f"Settling time (2%): {settling_time:.3f}s")
        print(f"Rise time (10-90%): {rise_time:.3f}s" if rise_time else "Rise time: N/A")
    
    def vibration_analysis(self):
        """FFT of accelerometer data to identify vibration frequencies."""
        fig, axes = plt.subplots(3, 1, figsize=(12, 8))
        
        dt = np.mean(np.diff(self.df['time_ms'])) / 1000
        fs = 1.0 / dt  # sample rate
        
        for i, axis in enumerate(['ax', 'ay', 'az']):
            data = self.df[axis].values
            data = data - np.mean(data)  # remove DC offset
            
            freqs, psd = signal.welch(data, fs=fs, nperseg=min(1024, len(data)//2))
            
            axes[i].semilogy(freqs, psd)
            axes[i].set_ylabel(f'{axis} PSD')
            axes[i].set_xlim(0, fs/2)
            axes[i].axvline(x=fs/2 * 0.1, color='r', linestyle='--', alpha=0.5, 
                           label='Typical motor freq range')
        
        axes[-1].set_xlabel('Frequency (Hz)')
        plt.suptitle('Vibration Spectrum Analysis')
        plt.tight_layout()
        return fig
    
    def motor_analysis(self):
        """Analyze motor outputs for saturation and balance."""
        t = self.df['time_ms'] / 1000
        motors = [self.df[f'motor{i}'] for i in range(1, 5)]
        
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6))
        
        for i, m in enumerate(motors):
            ax1.plot(t, m, label=f'Motor {i+1}', alpha=0.7)
        ax1.set_ylabel('Motor Output (%)')
        ax1.legend()
        ax1.set_title('Motor Outputs')
        
        # Motor imbalance: if one motor is consistently higher, CG is off
        motor_means = [m.mean() for m in motors]
        ax2.bar(range(1, 5), motor_means)
        ax2.set_xlabel('Motor Number')
        ax2.set_ylabel('Average Output (%)')
        ax2.set_title('Motor Balance (should be roughly equal in hover)')
        
        # Check for saturation
        for i, m in enumerate(motors):
            sat_pct = (m >= 95).sum() / len(m) * 100
            if sat_pct > 1:
                print(f"WARNING: Motor {i+1} saturated {sat_pct:.1f}% of the time!")
        
        plt.tight_layout()
        return fig
```

**Exercises:**
1. After each Project 4 test flight, run all four analyses. Save plots to your flight-logs folder.
2. If vibration analysis shows a spike at a specific frequency, that's your motor RPM. You need notch filter there.
3. If motor analysis shows imbalance, your center of gravity is off. Reposition the battery.
4. Track your step response metrics across flights — they should improve as you tune.

---

## Part 3: MAVLink and DroneKit (Ground Station Programming)

### 3.1 — MAVLink Protocol Basics

MAVLink is THE standard protocol for drone communication. Used by ArduPilot, PX4, and most ground stations.

```python
"""
MAVLink basics using pymavlink.
Install: pip install pymavlink
"""
from pymavlink import mavutil

# Connect to SITL (Software-In-The-Loop simulator)
# First, start ArduPilot SITL: sim_vehicle.py -v ArduCopter
connection = mavutil.mavlink_connection('tcp:127.0.0.1:5760')

# Wait for heartbeat
connection.wait_heartbeat()
print(f"Connected to system {connection.target_system}")

# Request data streams
connection.mav.request_data_stream_send(
    connection.target_system, connection.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1  # 10 Hz, start
)

# Read messages
while True:
    msg = connection.recv_match(blocking=True, timeout=1)
    if msg is None:
        continue
    
    msg_type = msg.get_type()
    
    if msg_type == 'ATTITUDE':
        print(f"Roll: {msg.roll:.2f}, Pitch: {msg.pitch:.2f}, Yaw: {msg.yaw:.2f}")
    
    elif msg_type == 'GLOBAL_POSITION_INT':
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.relative_alt / 1000
        print(f"Position: {lat:.6f}, {lon:.6f}, Alt: {alt:.1f}m")
    
    elif msg_type == 'VFR_HUD':
        print(f"Airspeed: {msg.airspeed:.1f}, Groundspeed: {msg.groundspeed:.1f}, "
              f"Heading: {msg.heading}°, Throttle: {msg.throttle}%")
```

---

### 3.2 — DroneKit: High-Level Drone Control

```python
"""
DroneKit autonomous mission example.
Install: pip install dronekit dronekit-sitl
"""
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import time

# Connect to SITL
vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)

def arm_and_takeoff(target_altitude):
    """Arms vehicle and flies to target altitude."""
    print("Pre-arm checks...")
    while not vehicle.is_armable:
        time.sleep(1)
    
    print("Arming...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)
    
    print(f"Taking off to {target_altitude}m...")
    vehicle.simple_takeoff(target_altitude)
    
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"  Altitude: {alt:.1f}m")
        if alt >= target_altitude * 0.95:
            print("Reached target altitude!")
            break
        time.sleep(1)


def fly_mission(waypoints):
    """Fly a list of waypoints [(lat, lon, alt), ...]."""
    cmds = vehicle.commands
    cmds.clear()
    
    for i, (lat, lon, alt) in enumerate(waypoints):
        cmd = Command(
            0, 0, 0,                    # target system/component/seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 0,                        # current, autocontinue
            0, 0, 0, 0,                 # params (hold, radius, pass-by, yaw)
            lat, lon, alt
        )
        cmds.add(cmd)
    
    cmds.upload()
    print(f"Uploaded {len(waypoints)} waypoints")
    
    vehicle.mode = VehicleMode("AUTO")
    
    while vehicle.commands.next < len(waypoints):
        print(f"  Waypoint {vehicle.commands.next}/{len(waypoints)}, "
              f"Alt: {vehicle.location.global_relative_frame.alt:.1f}m")
        time.sleep(2)
    
    print("Mission complete!")


# --- Execute ---
arm_and_takeoff(20)

waypoints = [
    (vehicle.location.global_frame.lat + 0.001, 
     vehicle.location.global_frame.lon, 30),
    (vehicle.location.global_frame.lat + 0.001, 
     vehicle.location.global_frame.lon + 0.001, 30),
    (vehicle.location.global_frame.lat, 
     vehicle.location.global_frame.lon + 0.001, 30),
]
fly_mission(waypoints)

# Return to launch
print("Returning to launch...")
vehicle.mode = VehicleMode("RTL")

vehicle.close()
```

**Exercises:**
1. Run this with SITL. Watch the simulated drone fly the waypoints.
2. Add a geofence (check if drone goes too far, trigger RTL).
3. Add real-time plotting of the drone's position on a map (use `folium` library).
4. Implement a search pattern (lawnmower pattern) — calculate the waypoints programmatically.
5. Add a "mission abort" trigger based on battery voltage.

---

## Part 4: ArduPilot SITL (Software In The Loop)

### Setting Up SITL

SITL lets you run the full ArduPilot autopilot on your PC with simulated physics. This is how you test before flying.

```bash
# Install ArduPilot SITL (Linux/WSL)
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile

# Run SITL for quadcopter
cd ArduCopter
sim_vehicle.py -w  # first run: loads default parameters
sim_vehicle.py --console --map  # subsequent runs: with console and map

# Run SITL for fixed-wing (for Project 5)
cd ../ArduPlane
sim_vehicle.py --console --map
```

### SITL Testing Workflow

```
1. Write/modify your Python script (DroneKit or pymavlink)
2. Start SITL: sim_vehicle.py --console --map
3. Run your script against SITL
4. Observe behavior in the map window
5. Download and analyze logs: logs/00000001.BIN
6. Fix issues, repeat
7. Only when SITL behavior is correct → move to real hardware
```

### Analyzing SITL Logs

```python
"""
Parse ArduPilot .BIN logs using pymavlink.
"""
from pymavlink import mavutil

log = mavutil.mavlink_connection('00000001.BIN')

# Extract attitude data
attitudes = []
while True:
    msg = log.recv_match(type='ATT', blocking=False)
    if msg is None:
        break
    attitudes.append({
        'time': msg._timestamp,
        'roll': msg.Roll,
        'pitch': msg.Pitch,
        'yaw': msg.Yaw,
        'desired_roll': msg.DesRoll,
        'desired_pitch': msg.DesPitch,
    })

import pandas as pd
df = pd.DataFrame(attitudes)
# Now use FlightLogAnalyzer from Part 2 for analysis
```

---

## Part 5: Useful Python Libraries Reference

### Essential (install all of these)
```bash
pip install numpy scipy matplotlib pandas pyserial
pip install control                   # control system analysis (transfer functions, Bode, root locus)
pip install filterpy                  # Kalman filter library (study the source!)
pip install opencv-python             # computer vision
pip install pymavlink dronekit        # MAVLink communication
```

### For Simulation
```bash
pip install vpython                   # 3D visualization (great for drone attitude display)
pip install pygame                    # 2D visualization and joystick input
pip install pybullet                  # physics simulation (can simulate quadcopter dynamics)
```

### For Data Analysis
```bash
pip install seaborn                   # prettier plots
pip install scikit-learn              # for ML-based anomaly detection in flight logs
pip install folium                    # GPS track visualization on maps
```

### For Computer Vision (Phase 3 prep)
```bash
pip install torch torchvision         # PyTorch for deep learning
pip install ultralytics               # YOLOv8 object detection
pip install roboflow                  # dataset management
```

---

## Part 6: Python Project Ideas (Phase 1 Companions)

### Project P1: "Allan Variance Calculator"
Characterize your IMU noise. Compute and plot Allan deviation for gyroscope data. Identify: angle random walk, bias instability, rate random walk. These parameters go directly into your Kalman filter Q matrix.

### Project P2: "Magnetic Calibration Tool"
Read magnetometer data while rotating the sensor through all orientations. Fit an ellipsoid to the data (hard-iron and soft-iron calibration). Output calibration matrix and offset. This is critical for accurate heading estimation.

### Project P3: "PID Auto-Tuner"
Implement relay-based auto-tuning (Åström method):
1. Apply relay output (bang-bang control)
2. System oscillates at ultimate frequency
3. Measure amplitude and frequency
4. Compute Ku and Tu
5. Apply Ziegler-Nichols formula
Run this on your reaction wheel (Project 2) automatically.

### Project P4: "KML Mission Plotter"
Read a list of GPS waypoints, generate a KML file for Google Earth visualization. Add: waypoint markers, flight path lines, altitude profile, estimated flight time. Use for pre-flight mission review (Project 5).

### Project P5: "Battery Discharge Analyzer"
Log battery voltage during flight. Plot voltage vs. time, voltage vs. throttle, estimate remaining capacity using Coulomb counting. Correlate with flight events (takeoff, hover, landing). Build the foundation for your BMS in Phase 2.

---

> *"Python lets you think at the speed of ideas. C lets you execute at the speed of hardware. Use both."*
