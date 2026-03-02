# Operation Shaheen — Phase 1: The Foundation (Months 1–6)

> **"Master the fundamentals that separate hobbyists from engineers."**
>
> This document is the detailed execution plan for Phase 1. Every section maps to a project folder. Every task has acceptance criteria. No hand-waving.

---

## How to Use This File

This is designed as a **Claude Code project file**. Place it in your repo root as `CLAUDE.md`. Each project below maps to a directory:

```
operation-shaheen/
├── CLAUDE.md                  ← this file
├── 01-sensor-whisperer/
├── 02-control-theorist/
├── 03-embedded-kalman/
├── 04-flight-controller-zero/
├── 05-mahogany-bomber/
├── math-notes/
├── tools/                     ← shared plotting scripts, serial utils
├── hardware/                  ← BOM files, wiring diagrams, datasheets
└── docs/                      ← research papers, personal notes
```

When asking Claude Code for help, reference the specific project and sub-task (e.g., *"Help me with Project 1, Task 1.3 — complementary filter implementation"*).

---

## Global Setup & Toolchain

Before any project, set up the development environment once.

### Hardware You Need (Phase 1 Total BOM)

| Item | Est. Cost (USD) | When Needed |
|---|---|---|
| STM32F411 Black Pill + ST-Link | $12 | Project 1 |
| MPU-6050 breakout (or BMI088) | $5–$25 | Project 1 |
| USB-UART adapter (CP2102/FTDI) | $5 | Project 1 |
| Breadboard, jumper wires, headers | $10 | Project 1 |
| Servo motor (MG996R or similar) | $8 | Project 2 |
| Reaction wheel / flywheel disk | $5 (DIY) | Project 2 |
| BMP280 / MS5611 barometer breakout | $5–$15 | Project 3 |
| Quadcopter frame kit 250–450mm | $25–$50 | Project 4 |
| 4× Brushless motors (2205–2306) | $30–$50 | Project 4 |
| 4× ESCs (BLHeli_S 30A) | $30–$40 | Project 4 |
| LiPo battery 3S 1500–2200mAh | $15–$25 | Project 4 |
| RC Transmitter + Receiver (FlySky i6X or RadioMaster) | $50–$80 | Project 4 |
| Propellers (5" or 6", pack of 10) | $8 | Project 4 |
| Power distribution board (PDB) | $8 | Project 4 |
| Fixed-wing airframe (foam, 1.5–2m) | $60–$120 | Project 5 |
| GPS module (u-blox NEO-M8N/M9N) | $15–$30 | Project 5 |
| Pitot tube + airspeed sensor | $10–$15 | Project 5 |
| Servos × 4 (for control surfaces) | $20 | Project 5 |
| **Estimated Phase 1 Total** | **$350–$550** | |

### Software Toolchain

```bash
# STM32 Development
- STM32CubeIDE (free, includes HAL library + debugger)
- STM32CubeMX (pin/clock configuration GUI)
- OpenOCD (open-source debugger, alternative to ST-Link utility)

# Python Environment (for plotting, simulation, path planning)
- Python 3.10+
- pip install numpy matplotlib scipy pyserial filterpy control

# Simulation
- MATLAB/Octave (for control system analysis, Bode plots)
- Gazebo + ArduPilot SITL (for flight controller testing before real hardware)

# Version Control
- Git + GitHub (every project gets its own branch, main stays clean)

# Documentation
- Markdown for all notes
- Jupyter notebooks for math derivations + plots
```

---

## Month 1–2: Mathematical Foundations

> **Do not skip this.** Every project below will punish you if you don't internalize the math first. You don't need to finish every textbook — focus on the specific chapters and concepts listed.

### Linear Algebra (3–4 weeks)

**Primary Text:** *Introduction to Linear Algebra* — Gilbert Strang (Chapters 1–7)

Study plan (map each concept to its drone application):

| Week | Chapters | Key Concepts | Why It Matters |
|---|---|---|---|
| 1 | Ch 1–2 | Vectors, dot product, matrix multiplication | Sensor data is vectors. Transformations are matrices. |
| 2 | Ch 3–4 | Vector spaces, null space, rank, linear independence | Understanding when your system is observable/controllable |
| 3 | Ch 5–6 | Determinants, eigenvalues, eigenvectors | Stability analysis — if eigenvalues have positive real parts, your drone diverges |
| 4 | Ch 7 | SVD, positive definite matrices | Covariance matrices in Kalman filters are positive semi-definite |

**Exercises (do these, not optional):**
- [ ] Implement matrix multiplication in C (no libraries) — you'll need this on the STM32
- [ ] Write a Python script that computes eigenvalues of a 4×4 state transition matrix
- [ ] Prove to yourself: multiply a rotation matrix by its transpose → identity
- [ ] Implement least squares fitting in Python (fit a line to noisy accelerometer data)

### Multivariable Calculus & Dynamics (3–4 weeks)

**Primary Text:** *Calculus: Early Transcendentals* — Stewart (Chapters 12–16)
**Secondary Text:** *Classical Mechanics* — John R. Taylor (Chapters 1–4, 9–10)

| Week | Focus | Key Concepts |
|---|---|---|
| 5 | Stewart Ch 12–13 | Vectors in 3D, partial derivatives, gradient |
| 6 | Stewart Ch 14–15 | Chain rule (for Jacobians), optimization |
| 7 | Taylor Ch 1–4 | Newton's laws in 3D, angular momentum, torque |
| 8 | Taylor Ch 9–10 | Rigid body rotation, moment of inertia tensor, Euler's equations |

**Exercises:**
- [ ] Derive the equations of motion for a spinning top (this is basically a quadcopter)
- [ ] Compute the moment of inertia tensor for an X-frame quadcopter (approximate as point masses at motor positions)
- [ ] Implement Euler angle → rotation matrix conversion in Python
- [ ] Implement quaternion multiplication and quaternion → rotation matrix in Python
- [ ] Understand gimbal lock: create a visualization that shows it happening

### Kalman Filter Theory (parallel with Projects 1–3)

**Primary Text:** *Kalman Filter from the Ground Up* — Alex Becker (entire book)
**Secondary Text:** *Optimal State Estimation* — Dan Simon (Chapters 1–5)

This is studied **in parallel** with the hands-on projects below. Read 2–3 chapters per week while building.

- [ ] Implement a 1D Kalman filter in Python (track a falling ball with noisy position measurements)
- [ ] Extend to 2D (position + velocity state)
- [ ] Visualize: plot predicted vs. measured vs. filtered estimates
- [ ] Understand every variable: x (state), P (covariance), Q (process noise), R (measurement noise), K (Kalman gain)

---

## Project 1: "Sensor Whisperer" (Weeks 1–3)

> **Goal:** Read raw IMU data from an MPU-6050/BMI088, implement a complementary filter, and output real-time orientation estimates.

### Directory Structure

```
01-sensor-whisperer/
├── firmware/
│   ├── Core/Src/main.c
│   ├── Core/Inc/imu.h
│   ├── Core/Src/imu.c
│   ├── Core/Inc/complementary_filter.h
│   └── Core/Src/complementary_filter.c
├── python/
│   ├── serial_plotter.py        ← real-time matplotlib plot
│   └── log_analyzer.py          ← post-flight analysis
├── docs/
│   ├── wiring_diagram.png
│   └── notes.md
├── data/                         ← logged CSV files
└── README.md
```

### Task 1.1 — STM32 Bring-Up (Day 1–3)

**Acceptance Criteria:** LED blinks at exactly 1Hz using a hardware timer (not delay loops).

Steps:
- [ ] Install STM32CubeIDE, create new project for STM32F411CEU6
- [ ] Configure system clock to 100MHz (HSE → PLL)
- [ ] Configure TIM2 to generate 1Hz interrupt
- [ ] Toggle onboard LED (PC13) in timer ISR
- [ ] Verify with oscilloscope or logic analyzer (if available) — otherwise count visually

**What you're learning:** Clock tree, timer peripherals, interrupt handling, HAL library structure.

### Task 1.2 — I2C Communication with IMU (Day 4–7)

**Acceptance Criteria:** Read WHO_AM_I register from MPU-6050, get correct response (0x68).

Steps:
- [ ] Wire MPU-6050: VCC→3.3V, GND→GND, SDA→PB7, SCL→PB6
- [ ] Configure I2C1 in STM32CubeMX (100kHz standard mode first, then try 400kHz fast mode)
- [ ] Write `imu_init()` function: wake device, set accel range (±4g), gyro range (±500°/s)
- [ ] Write `imu_read_raw()` function: burst-read 14 bytes (accel XYZ + temp + gyro XYZ)
- [ ] Convert raw 16-bit values to physical units (g's and °/s) using sensitivity scale factors
- [ ] Print to UART at 100Hz, verify values make sense (gravity = ~1g on Z when flat)

**Reference:** MPU-6050 register map (InvenSense document RM-MPU-6000A), Freescale AN4248

**Debugging checklist if I2C fails:**
1. Check pull-up resistors on SDA/SCL (4.7kΩ to 3.3V)
2. Verify correct I2C address (0x68 or 0x69 depending on AD0 pin)
3. Check voltage levels (MPU-6050 is 3.3V, don't use 5V)
4. Use logic analyzer to verify I2C transactions

### Task 1.3 — UART Streaming + Python Plotter (Day 8–10)

**Acceptance Criteria:** Real-time plot showing accelerometer and gyroscope data updating at 50+ Hz.

Steps:
- [ ] Configure UART2 at 115200 baud (or 921600 for higher throughput)
- [ ] Format output as CSV: `timestamp_ms,ax,ay,az,gx,gy,gz\n`
- [ ] Write `serial_plotter.py`:
  - Use `pyserial` to read from COM port
  - Use `matplotlib.animation` for real-time plotting
  - Plot 6 subplots (3 accel + 3 gyro)
  - Scrolling window of last 500 samples
- [ ] Verify: tilt sensor, see accelerometer change. Rotate, see gyroscope respond.
- [ ] Log 60 seconds of stationary data to CSV → compute noise statistics (mean, std dev)

**Key insight:** The noise floor you measure here determines your filter parameters later. Typical MPU-6050 gyro noise: ~0.05°/s RMS. Accel noise: ~0.01g RMS.

### Task 1.4 — Complementary Filter (Day 11–17)

**Acceptance Criteria:** Real-time roll/pitch estimate accurate to ±2° compared to known reference angles.

Steps:
- [ ] Implement in `complementary_filter.c`:
  ```c
  // Complementary filter — trust gyro short-term, accel long-term
  float alpha = 0.98;  // tunable
  angle_pitch = alpha * (angle_pitch + gyro_x * dt) + (1 - alpha) * accel_pitch;
  angle_roll  = alpha * (angle_roll  + gyro_y * dt) + (1 - alpha) * accel_roll;
  ```
- [ ] Calculate `accel_pitch` and `accel_roll` from accelerometer using `atan2()`
- [ ] Run filter at 1kHz (use TIM interrupt), output filtered angles via UART at 100Hz
- [ ] Test: place sensor at known angles (0°, 30°, 45°, 90°) using protractor or known surface
- [ ] Measure drift: leave stationary for 5 minutes, log angle — should drift <1°
- [ ] Tune `alpha`: try 0.95, 0.98, 0.99 — plot comparison

**Understanding the tradeoff:**
- `alpha` too high (0.999): trusts gyro more → less noise but more drift over time
- `alpha` too low (0.90): trusts accel more → no drift but shaky response
- The complementary filter is a cheap approximation of a Kalman filter (you'll prove this later)

### Task 1.5 — Sampling Rate Optimization (Day 18–21)

**Acceptance Criteria:** Sensor read + filter computation completes in <200µs at 1kHz, verified by GPIO toggle measurement.

Steps:
- [ ] Toggle a GPIO pin before/after sensor read + filter computation
- [ ] Measure pulse width with oscilloscope or logic analyzer
- [ ] If >200µs: optimize I2C (use DMA transfer instead of polling)
- [ ] Implement double buffering: DMA fills buffer A while you process buffer B
- [ ] Profile each function: I2C read time vs. filter computation time vs. UART output time

### Deliverable Checklist — Project 1
- [ ] Real-time orientation estimate accurate to ±2°
- [ ] 1kHz sampling rate verified
- [ ] Python plotter showing live data
- [ ] 60-second stationary noise characterization logged
- [ ] All code pushed to GitHub with README explaining the system
- [ ] Short video demo (phone recording of plotter + sensor movement)

---

## Project 2: "The Control Theorist" (Weeks 4–8)

> **Goal:** Build a 1-DOF reaction wheel stabilizer and implement PID control from scratch. No libraries.

### Required Reading (before starting)

- *Feedback Control of Dynamic Systems* — Franklin, Powell, Emami-Naeini (Chapters 1–5)
- "Quadrotor Control: Modeling, Nonlinear Control Design, and Simulation" — Bouabdallah et al.
- *PID Control: New Identification and Design Methods* — Åström & Hägglund (skim for tuning methods)

### Directory Structure

```
02-control-theorist/
├── firmware/
│   ├── Core/Src/main.c
│   ├── Core/Inc/pid.h
│   ├── Core/Src/pid.c
│   ├── Core/Inc/motor.h
│   └── Core/Src/motor.c
├── simulation/
│   ├── pid_sim.py             ← simulate PID on mathematical model first
│   ├── plant_model.py         ← 1-DOF dynamics model
│   └── tuning_analysis.py     ← Ziegler-Nichols, step response plots
├── hardware/
│   ├── BOM.md
│   └── assembly_photos/
├── data/
│   ├── step_responses/        ← CSV logs of each tuning attempt
│   └── analysis/
└── README.md
```

### Task 2.1 — Control Theory Fundamentals (Week 4)

**Acceptance Criteria:** Can explain PID in your own words, derive discrete PID from continuous, and simulate a controlled system in Python.

Steps:
- [ ] Read Franklin Ch 1–3 (feedback concept, Laplace transforms, transfer functions)
- [ ] Implement continuous PID in Python simulation:
  ```python
  # Plant: simple pendulum (theta_ddot = -g/L * sin(theta) + tau/I)
  # Controller: PID outputs torque tau
  ```
- [ ] Plot step response: identify overshoot, settling time, steady-state error
- [ ] Derive discrete PID (trapezoidal integration for I term, backward difference for D term)
- [ ] Compare forward Euler vs. trapezoidal integration in simulation — see which is more stable

**Key formulas to internalize:**
- Transfer function of PID: `C(s) = Kp + Ki/s + Kd*s`
- Discrete: `output[k] = Kp*e[k] + Ki*sum(e[0..k])*dt + Kd*(e[k]-e[k-1])/dt`
- Derivative kick: use `Kd*(measurement[k-1] - measurement[k])/dt` instead of error derivative

### Task 2.2 — Hardware Build: 1-DOF Reaction Wheel (Week 5)

**Acceptance Criteria:** A physical beam that can be tilted by hand and returns to level using a spinning reaction wheel.

Build options (pick one):
1. **Reaction wheel on a beam:** Aluminum bar pivoting on a bearing, brushless motor with heavy disk on one end, IMU at center
2. **Gimbal with servo:** Simpler — servo directly tilts a platform, IMU on platform

Recommended: Option 1 (reaction wheel) — it's closer to quadcopter physics.

Steps:
- [ ] Mount IMU (from Project 1) on the beam
- [ ] Mount brushless motor + ESC (or DC motor + H-bridge for easier bidirectional control)
- [ ] Configure PWM output on STM32 for motor speed control
- [ ] Verify: sending PWM command → motor spins → beam reacts
- [ ] Calibrate: motor command vs. torque mapping (static test with scale)

### Task 2.3 — PID Implementation in C (Week 5–6)

**Acceptance Criteria:** PID controller running at 500Hz on STM32, with anti-windup and derivative filtering.

```c
// pid.h — implement this struct and functions
typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float output_min, output_max;    // output clamp
    float integral_min, integral_max; // anti-windup clamp
    float dt;
    float d_filter_alpha;            // low-pass on derivative (0.1–0.3)
    float prev_derivative;
} PID_Controller;

float pid_update(PID_Controller *pid, float setpoint, float measurement);
void pid_reset(PID_Controller *pid);
```

Implementation checklist:
- [ ] Basic P control first — verify direction is correct (negative feedback!)
- [ ] Add I term — observe: removes steady-state error but may oscillate
- [ ] Add D term — observe: dampens oscillation
- [ ] Anti-windup: clamp integral term when output is saturated
- [ ] Derivative filter: low-pass filter on D term to reduce noise amplification
- [ ] Derivative on measurement (not error) to avoid derivative kick on setpoint changes
- [ ] Output clamping: never exceed motor's safe command range

### Task 2.4 — Ziegler-Nichols Tuning (Week 6–7)

**Acceptance Criteria:** Document the tuning process with logged data. Know your system's ultimate gain and period.

Process:
- [ ] Set Ki=0, Kd=0
- [ ] Increase Kp until system oscillates continuously (marginally stable) → this is `Ku` (ultimate gain)
- [ ] Measure oscillation period → this is `Tu`
- [ ] Apply Ziegler-Nichols table:
  - P only: `Kp = 0.5*Ku`
  - PI: `Kp = 0.45*Ku, Ki = Kp/(0.83*Tu)`
  - PID: `Kp = 0.6*Ku, Ki = Kp/(0.5*Tu), Kd = Kp*0.125*Tu`
- [ ] Log step responses for each configuration
- [ ] Fine-tune manually from Z-N starting point (Z-N is aggressive, usually needs de-tuning)

### Task 2.5 — Performance Characterization (Week 7–8)

**Acceptance Criteria:** Step response data showing <2% overshoot and <2 second settling time.

Steps:
- [ ] Apply step input (e.g., tilt setpoint from 0° to 15°)
- [ ] Log: timestamp, setpoint, measurement, error, P/I/D contributions, output
- [ ] Plot in Python:
  - Step response (setpoint vs. actual angle over time)
  - Error over time
  - Control effort over time
  - Individual P, I, D contributions
- [ ] Measure and document:
  - Rise time (10% → 90% of setpoint)
  - Overshoot percentage
  - Settling time (within ±1% of setpoint)
  - Steady-state error
- [ ] Disturbance rejection test: flick the beam, measure recovery time
- [ ] Repeat with different setpoints (5°, 15°, 30°, 45°)

### Task 2.6 — Bonus: Frequency Domain Analysis (Week 8)

**Acceptance Criteria:** Bode plot of open-loop system matches expected behavior.

- [ ] In MATLAB/Octave: create transfer function model of your plant + controller
- [ ] Plot Bode diagram — identify gain margin, phase margin
- [ ] Verify: phase margin >45° (robust stability)
- [ ] If phase margin is low, increase D gain or reduce loop bandwidth

### Deliverable Checklist — Project 2
- [ ] System settles to ±1% of setpoint in <2 seconds
- [ ] Documented tuning process with Ziegler-Nichols data
- [ ] Step response plots for at least 3 setpoints
- [ ] Disturbance rejection demonstrated
- [ ] PID code with anti-windup, derivative filter, output clamping
- [ ] All code on GitHub, video demo of stabilization

---

## Project 3: "The Embedded Kalman" (Weeks 9–12)

> **Goal:** Implement a 1D Kalman filter for altitude estimation by fusing barometer and accelerometer data. Then compare against your complementary filter.

### Required Reading

- *Kalman Filter from the Ground Up* — Alex Becker (should be mostly done by now)
- *Optimal State Estimation* — Dan Simon (Ch 3–5)
- Study TinyEKF source code (understand, don't copy)

### Directory Structure

```
03-embedded-kalman/
├── firmware/
│   ├── Core/Inc/kalman.h
│   ├── Core/Src/kalman.c
│   ├── Core/Inc/barometer.h
│   ├── Core/Src/barometer.c
│   └── Core/Src/main.c
├── simulation/
│   ├── kalman_1d_sim.py       ← simulate before implementing on hardware
│   ├── data_generator.py      ← synthetic baro + accel data with noise
│   └── comparison.py          ← KF vs complementary filter analysis
├── data/
│   ├── stationary_tests/
│   └── elevator_tests/        ← ride elevator to test altitude tracking
└── README.md
```

### Task 3.1 — Barometer Bring-Up (Week 9)

**Acceptance Criteria:** BMP280/MS5611 reads pressure, converts to altitude, noise floor characterized.

Steps:
- [ ] Wire barometer to STM32 (I2C or SPI — SPI preferred for speed)
- [ ] Implement driver: read raw temperature + pressure, apply calibration coefficients
- [ ] Convert pressure to altitude: `alt = 44330 * (1 - (P/P0)^0.1903)`
- [ ] Log 60 seconds of stationary data at 50Hz
- [ ] Calculate noise statistics: typical BMP280 altitude noise is ~0.3m RMS
- [ ] Note: barometer is accurate long-term but slow. Accelerometer is fast but drifts. Perfect for fusion.

### Task 3.2 — Kalman Filter in Python First (Week 9–10)

**Acceptance Criteria:** Python KF tracks simulated altitude with noise better than raw measurements.

State vector: `x = [altitude, vertical_velocity]`

```python
# State transition (predict with accelerometer)
F = [[1, dt],
     [0,  1]]
B = [[0.5*dt**2],
     [dt]]
# Control input: u = accelerometer_z - gravity (vertical acceleration)

# Measurement (barometer gives altitude only)
H = [[1, 0]]

# Tune these:
Q = process_noise_covariance    # how much you trust the model
R = measurement_noise_covariance # how much you trust the barometer
```

Steps:
- [ ] Generate synthetic data: true altitude (sine wave), noisy accel, noisy baro
- [ ] Implement predict step: `x = F*x + B*u`, `P = F*P*F' + Q`
- [ ] Implement update step: `K = P*H'*(H*P*H' + R)^-1`, `x = x + K*(z - H*x)`, `P = (I - K*H)*P`
- [ ] Plot: true altitude, raw baro, raw accel-integrated, Kalman filtered
- [ ] The Kalman estimate should be visibly smoother and more accurate than either raw source

### Task 3.3 — Kalman Filter in C on STM32 (Week 10–11)

**Acceptance Criteria:** KF running at 100Hz on STM32, altitude estimate matches Python results.

Steps:
- [ ] Implement 2×2 matrix operations in C (multiply, add, transpose, inverse)
- [ ] Port your Python KF to C using these matrix helpers
- [ ] Predict step runs at IMU rate (1kHz), update step runs at baro rate (50Hz)
- [ ] Stream filtered altitude via UART, compare with raw barometer in Python plotter
- [ ] Handle edge case: what happens if barometer reading is missing? (predict-only mode)

### Task 3.4 — Comparison: Kalman vs. Complementary (Week 11)

**Acceptance Criteria:** Quantitative comparison showing KF outperforms complementary filter.

Steps:
- [ ] Implement complementary filter for altitude: `alt = alpha*(alt + accel_z*dt²/2) + (1-alpha)*baro_alt`
- [ ] Run both filters on identical data (log raw data, process offline with both)
- [ ] Metrics to compare:
  - RMS error vs. known reference (use elevator ride: floor 1 → floor 5 = known height)
  - Drift over 60 seconds (stationary test)
  - Response speed to sudden altitude change
  - Noise in output (standard deviation of filtered signal when stationary)
- [ ] Generate comparison plot and table for README

### Task 3.5 — Fixed-Point Optimization (Week 12)

**Acceptance Criteria:** KF runs using Q16.16 fixed-point math, results within 1% of floating-point version.

Steps:
- [ ] Implement Q16.16 fixed-point arithmetic library (multiply with 64-bit intermediate)
- [ ] Port KF to fixed-point
- [ ] Compare execution time: float vs. fixed-point (toggle GPIO, measure)
- [ ] Compare accuracy: run both on identical data, compute max difference
- [ ] On STM32F411 with FPU, float may actually be faster — document this finding

**Why learn this:** Not all microcontrollers have FPUs. On a Cortex-M0 (no FPU), fixed-point is 5–10× faster.

### Deliverable Checklist — Project 3
- [ ] Altitude drift <5cm over 60 seconds (stationary)
- [ ] Quantitative KF vs. complementary filter comparison (table + plots)
- [ ] Both Python simulation and embedded C implementation
- [ ] Fixed-point implementation with performance comparison
- [ ] GitHub repo with data, analysis notebooks, firmware
- [ ] Video: elevator ride demonstrating altitude tracking

---

## Project 4: "Flight Controller Zero" (Weeks 13–24)

> **Goal:** Build a quadcopter flight controller from scratch. Angle mode only (stabilization). This is the hardest project in Phase 1.

### Required Reading (non-negotiable)

- *Small Unmanned Aircraft: Theory and Practice* — Beard & McLain (Chapters 1–6)
- *Quadrotor Dynamics and Control* — Randal Beard lecture notes (free online)
- Betaflight source code (study `src/main/flight/` directory)
- "Quadrotor Control: Modeling, Nonlinear Control Design, and Simulation" — Bouabdallah et al.

### Directory Structure

```
04-flight-controller-zero/
├── firmware/
│   ├── Core/Inc/
│   │   ├── imu.h
│   │   ├── pid.h               ← reuse from Project 2, extend
│   │   ├── motor_mixing.h
│   │   ├── rc_input.h
│   │   ├── safety.h
│   │   └── config.h
│   ├── Core/Src/
│   │   ├── main.c
│   │   ├── imu.c
│   │   ├── pid.c
│   │   ├── motor_mixing.c
│   │   ├── rc_input.c
│   │   └── safety.c
│   └── Drivers/
├── simulation/
│   ├── quad_dynamics.py        ← 6-DOF quadcopter simulator
│   ├── cascaded_pid_sim.py     ← test controller in sim before hardware
│   └── motor_mixing_test.py
├── ground-station/
│   ├── telemetry_viewer.py     ← real-time plots during test flights
│   └── log_parser.py
├── hardware/
│   ├── BOM.md
│   ├── wiring_diagram.png
│   └── frame_dimensions.md
├── test-logs/                   ← every test flight logged here
├── safety/
│   ├── pre_flight_checklist.md
│   └── test_plan.md
└── README.md
```

### Sub-Project 4A — Quadcopter Dynamics Simulation (Week 13–14)

> Build a sim first. Never test untested control code on real hardware.

**Acceptance Criteria:** 6-DOF quadcopter simulator where you can command roll/pitch/yaw/thrust and see the quadcopter respond.

Steps:
- [ ] Implement rigid body dynamics:
  ```
  Forces:  F = m*a (gravity + thrust)
  Torques: tau = I*alpha + omega × (I*omega)  (Euler's equation)
  ```
- [ ] Motor model: thrust = k * omega² (k from motor datasheet or estimated)
- [ ] Motor mixing for X-configuration:
  ```
  Motor 1 (front-right): throttle - pitch + roll - yaw
  Motor 2 (rear-right):  throttle + pitch + roll + yaw
  Motor 3 (rear-left):   throttle + pitch - roll - yaw
  Motor 4 (front-left):  throttle - pitch - roll + yaw
  ```
- [ ] Integrate with Runge-Kutta 4 (RK4) — do NOT use Euler integration for 6-DOF
- [ ] Visualize with matplotlib 3D or a simple PyGame/PyOpenGL viewer
- [ ] Test: apply step roll command → quad tilts → translates → stabilize

### Sub-Project 4B — Hardware Assembly (Week 15–16)

**Acceptance Criteria:** All motors spin, ESCs calibrated, IMU reads correctly on the assembled frame.

Steps:
- [ ] Assemble frame (X-config, 250–450mm)
- [ ] Mount motors, solder ESC signal + power wires
- [ ] Wire PDB: battery → PDB → ESCs, PDB 5V → STM32
- [ ] Mount STM32 + IMU at center of gravity, vibration-dampened (foam tape)
- [ ] ESC calibration: all-high, all-low throttle sequence
- [ ] Motor direction test: verify each motor spins correct direction (CW/CCW alternating)
- [ ] Propeller direction: verify props match motor rotation
- [ ] **SAFETY: Remove propellers for all bench testing until Sub-Project 4E**

### Sub-Project 4C — RC Input Decoding (Week 16–17)

**Acceptance Criteria:** Read 4 channels (throttle, roll, pitch, yaw) from RC receiver, map to control commands.

Steps:
- [ ] Configure STM32 timer input capture for PWM reading (or UART for SBUS)
- [ ] Read 4 channels: throttle (1000–2000µs), roll, pitch, yaw
- [ ] Map to physical commands:
  - Throttle: 0–100% motor output
  - Roll/Pitch: ±30° angle command
  - Yaw: ±180°/s rate command
- [ ] Implement failsafe: if no RC signal for 500ms → disarm
- [ ] Log RC values, verify smooth response with no glitches

**Protocol options:**
- PWM: simplest, 1 timer channel per RC channel (4 channels = 4 timers)
- PPM: all channels on 1 wire, time-division multiplexed
- SBUS: serial protocol, 16 channels on 1 UART (inverted serial, 100kbps)

### Sub-Project 4D — Cascaded PID Controller (Week 17–20)

**Acceptance Criteria:** Cascaded PID (rate + angle) stabilizes the quadcopter in simulation with realistic noise.

Architecture:
```
Setpoint (from RC) → [Angle PID] → Rate setpoint → [Rate PID] → Motor commands
                         ↑                              ↑
                    Angle estimate                 Gyro rate (raw)
                  (complementary filter)
```

Steps:
- [ ] Implement inner loop (rate PID): runs at 1kHz, reads gyro directly
  - Input: desired angular rate (°/s)
  - Output: torque command
  - This loop is FAST — raw gyro, no filter needed (gyro is already a rate sensor)
- [ ] Implement outer loop (angle PID): runs at 500Hz, reads filtered angle
  - Input: desired angle (°) from RC
  - Output: desired angular rate (°/s) → feeds into inner loop
- [ ] Three separate PID instances: roll rate, pitch rate, yaw rate
- [ ] Three separate PID instances: roll angle, pitch angle (yaw is rate-only in angle mode)
- [ ] Test in simulation first with realistic sensor noise injected
- [ ] Tune inner loop FIRST (with outer loop disabled), then tune outer loop

**Critical tuning order:**
1. Rate PID → fast response, minimal oscillation
2. Angle PID → smooth tracking, no overshoot
3. Never tune both simultaneously

### Sub-Project 4E — Motor Mixing & Output (Week 18–19)

**Acceptance Criteria:** Motor mixing matrix correctly converts roll/pitch/yaw/thrust commands to individual motor PWM values.

```c
// Motor mixing for X-configuration
// Inputs: throttle (0-1), roll (-1 to 1), pitch (-1 to 1), yaw (-1 to 1)
motor[0] = throttle - pitch_cmd + roll_cmd - yaw_cmd;  // Front-Right (CW)
motor[1] = throttle + pitch_cmd + roll_cmd + yaw_cmd;  // Rear-Right  (CCW)
motor[2] = throttle + pitch_cmd - roll_cmd - yaw_cmd;  // Rear-Left   (CW)
motor[3] = throttle - pitch_cmd - roll_cmd + yaw_cmd;  // Front-Left  (CCW)

// Clamp to [motor_min, motor_max]
// Scale to ESC PWM range (1000-2000µs)
```

Steps:
- [ ] Implement mixing matrix
- [ ] Implement motor output via PWM (configure TIM channels for 400Hz or OneShot125)
- [ ] **Bench test WITHOUT props:** command roll → verify correct two motors speed up
- [ ] Test all 4 axes: positive roll, negative roll, positive pitch, negative pitch, positive yaw, negative yaw
- [ ] Verify: no motor ever exceeds max command, no motor goes below idle when armed

### Sub-Project 4F — Safety Systems (Week 19–20)

**Acceptance Criteria:** Arm/disarm logic works, failsafe triggers correctly, geofence limits in place.

**This is the most important sub-project. Rushing safety gets people hurt.**

- [ ] Arm/disarm: throttle low + yaw right for 2 seconds = arm. Throttle low + yaw left = disarm.
- [ ] Pre-arm checks: IMU calibrated, RC connected, battery voltage OK
- [ ] Failsafe conditions (any triggers immediate disarm):
  - RC signal lost for >500ms
  - IMU data invalid (NaN, stuck values)
  - Extreme angle (>60° in any axis)
  - Battery voltage below threshold
- [ ] Motor idle: when armed but throttle = 0, motors spin at low idle (aids gyro stability)
- [ ] Max angle limit: software-limited to ±45° regardless of RC input
- [ ] Disarm on landing: if throttle low + no motion for 5 seconds → auto-disarm

### Sub-Project 4G — Ground Testing (Week 21–22)

**Acceptance Criteria:** Quadcopter held in hand (with props) responds correctly to tilt — counter-torques are correct direction and magnitude.

**SAFETY PROTOCOL:**
1. Test outdoors in open area
2. Wear safety glasses
3. First powered test: quadcopter strapped to test stand (zip-tied to heavy object)
4. Have a kill switch (second person with TX, throttle cut channel)

Steps:
- [ ] Test stand hover: strap quad to stand, increase throttle until light on straps
- [ ] Verify: tilt forward → rear motors speed up (correct)
- [ ] Verify: tilt right → left motors speed up (correct)
- [ ] Listen for oscillation (buzzing sound = gains too high)
- [ ] Hand test (experienced pilot only): hold quad lightly, feel it try to stabilize
- [ ] Log all sensor data during tests for post-analysis

### Sub-Project 4H — First Flight (Week 23–24)

**Acceptance Criteria:** Stable hover for 30 seconds with manual RC input, <5° drift.

Steps:
- [ ] Pre-flight checklist (create this document, use EVERY time):
  - Battery charged + voltage check
  - Props tight, correct rotation direction
  - IMU calibrated (gyro bias at startup)
  - RC range check (walk 50m, verify control)
  - Failsafe test (turn off TX, verify motors cut)
  - Weather: wind <5 m/s
- [ ] Flight 1: 10cm hover, 5 seconds, land immediately
- [ ] Flight 2: 50cm hover, 15 seconds, test gentle roll/pitch commands
- [ ] Flight 3: 1m hover, 30 seconds, full angle authority test
- [ ] Log EVERYTHING: full rate sensor data, PID outputs, motor commands, RC inputs
- [ ] Post-flight analysis: plot all data, identify any oscillation or drift issues
- [ ] Iterate: adjust PID gains based on flight data, re-fly

### Deliverable Checklist — Project 4
- [ ] Stable hover for 30 seconds with <5° drift
- [ ] Cascaded PID with tuned gains (documented tuning process)
- [ ] Safety systems: arm/disarm, failsafe, angle limits all functional
- [ ] 6-DOF simulation matching real-world behavior
- [ ] All flight test data logged and analyzed
- [ ] GitHub repo with firmware, simulation, ground station tools
- [ ] Video: hover demonstration + hand-held stabilization test
- [ ] Post-mortem document: what went wrong, what was fixed, lessons learned

---

## Project 5: Phase 1 Capstone — "The Mahogany Bomber" (Weeks 20–26)

> **Goal:** Build a fixed-wing UAV with autopilot capabilities: altitude hold and 3–5 waypoint navigation.
>
> **Note:** This overlaps with Project 4. Start airframe build in week 20, begin autopilot integration after first quad hover.

### Why Fixed-Wing After Quadcopter?

Fixed-wing teaches you fundamentally different concepts: aerodynamic forces (not just motor thrust), energy management (you can't hover — run out of speed = crash), and trajectory planning (turn radius constraints). Defense UAVs are predominantly fixed-wing for endurance.

### Required Reading

- *Flight Stability and Automatic Control* — Nelson (Chapters 1–5)
- *Small Unmanned Aircraft* — Beard & McLain (Chapters 7–10 for guidance)
- ArduPlane documentation (guidance laws, TECS)

### Directory Structure

```
05-mahogany-bomber/
├── firmware/
│   ├── Core/Inc/
│   │   ├── ahrs.h              ← attitude heading reference system
│   │   ├── navigator.h         ← waypoint management
│   │   ├── guidance.h          ← L1 controller
│   │   ├── tecs.h              ← total energy control
│   │   ├── servo_control.h
│   │   └── gps.h
│   ├── Core/Src/
│   │   ├── main.c
│   │   ├── ahrs.c
│   │   ├── navigator.c
│   │   ├── guidance.c
│   │   ├── tecs.c
│   │   ├── servo_control.c
│   │   └── gps.c
│   └── Drivers/
├── simulation/
│   ├── fixed_wing_sim.py       ← 3-DOF longitudinal sim
│   └── l1_guidance_sim.py
├── mission-planner/
│   ├── waypoint_editor.py      ← define waypoints on map
│   └── kml_export.py           ← visualize in Google Earth
├── flight-logs/
├── safety/
│   ├── pre_flight_checklist.md
│   └── emergency_procedures.md
└── README.md
```

### Sub-Project 5A — Airframe & Assembly (Week 20–21)

Steps:
- [ ] Choose airframe: foam flying wing (simpler) or conventional with tail (more stable)
- [ ] Recommended: conventional trainer-style, 1.5–2m wingspan, high wing for stability
- [ ] Install servos for aileron, elevator, rudder (4-channel minimum)
- [ ] Install motor + ESC + propeller (pusher or tractor configuration)
- [ ] Install STM32 flight computer + IMU + barometer + GPS + pitot tube
- [ ] CG (center of gravity) check: should be at 25–30% of wing chord
- [ ] Control surface throw: ±20° ailerons, ±15° elevator, ±25° rudder (adjust to taste)
- [ ] Range test RC system

### Sub-Project 5B — GPS Integration (Week 21–22)

**Acceptance Criteria:** Parse NMEA or UBX protocol, get position fix with <3m accuracy (HDOP < 2).

Steps:
- [ ] Wire u-blox GPS to UART (9600 or 115200 baud depending on configuration)
- [ ] Parse NMEA sentences: $GPGGA (position), $GPRMC (velocity + heading)
- [ ] Or configure UBX binary protocol (more efficient, u-blox specific)
- [ ] Extract: latitude, longitude, altitude, ground speed, heading, number of satellites, HDOP
- [ ] Log GPS data for 10 minutes stationary → plot scatter of position fixes → understand GPS noise
- [ ] Implement coordinate conversions: GPS (LLA) → local NED (north-east-down) frame

### Sub-Project 5C — AHRS (Attitude Heading Reference System) (Week 22–23)

**Acceptance Criteria:** Full 3-axis attitude (roll, pitch, yaw) from IMU + magnetometer fusion.

Extend your Project 1 complementary filter to full AHRS:
- [ ] Add magnetometer (LIS3MDL or HMC5883L) for yaw/heading reference
- [ ] Hard-iron and soft-iron magnetometer calibration (rotate in 3D, fit ellipsoid)
- [ ] Tilt-compensated compass heading (use accel to correct mag for non-level flight)
- [ ] Full complementary filter or Madgwick/Mahony filter for all 3 axes
- [ ] Validate against known headings (use compass app on phone as rough reference)

### Sub-Project 5D — Lateral Guidance: L1 Controller (Week 23–24)

**Acceptance Criteria:** In simulation, aircraft follows a series of waypoints with smooth turns.

The L1 guidance law is used by ArduPlane and PX4. It's elegant and works well.

Steps:
- [ ] Study L1 guidance: *"A New Nonlinear Guidance Logic for Trajectory Tracking"* — Park, Deyst, How (AIAA 2004)
- [ ] Implement in Python simulation first:
  - Given: current position, current heading, next waypoint
  - Output: commanded bank angle (lateral acceleration)
  - L1 parameter: distance to reference point on path
- [ ] Simulate 5-waypoint circuit, tune L1 distance parameter
- [ ] Verify: smooth S-turns between waypoints, no overshoot at waypoints
- [ ] Port to C on STM32

### Sub-Project 5E — Longitudinal Control: TECS (Week 24–25)

**Acceptance Criteria:** Altitude hold within ±5m, airspeed hold within ±2 m/s in simulation.

TECS = Total Energy Control System. It coordinates throttle and elevator:
- Throttle controls total energy (kinetic + potential)
- Elevator controls energy distribution (speed vs. altitude)

Steps:
- [ ] Implement TECS in Python:
  - Total energy error: `E_total = 0.5*m*(V² - V_cmd²) + m*g*(h - h_cmd)`
  - Energy distribution: throttle adjusts total, pitch adjusts ratio
- [ ] Simulate: command altitude step (100m → 150m) → observe climb
- [ ] Simulate: command airspeed change (15 → 20 m/s) → observe acceleration
- [ ] Port to STM32, integrate with AHRS + GPS
- [ ] Airspeed from pitot tube: `V = sqrt(2 * differential_pressure / air_density)`

### Sub-Project 5F — Waypoint Navigation Integration (Week 25–26)

**Acceptance Criteria:** Full autonomous mission: takeoff, navigate 3–5 waypoints, loiter, return to launch.

Steps:
- [ ] Implement waypoint state machine:
  ```
  MANUAL → ARM → TAKEOFF → CLIMB → NAV_TO_WP1 → NAV_TO_WP2 → ... → LOITER → RTL → LANDING → DISARM
  ```
- [ ] Waypoint acceptance radius: 30–50m (switch to next when within radius)
- [ ] RC override: pilot can take control at any time (safety switch on TX)
- [ ] Loiter mode: circle at fixed radius (300–500m) and altitude
- [ ] Return to launch: navigate to home position, circle until pilot takes over for landing
- [ ] **First flights: manual only.** Enable autopilot modes incrementally:
  1. Manual flight — verify aircraft flies well
  2. Stabilize mode — autopilot assists with leveling
  3. Altitude hold — test TECS
  4. Waypoint mode — full autonomous

### Deliverable Checklist — Project 5
- [ ] Fixed-wing UAV flies autonomously through 3–5 waypoints
- [ ] Altitude hold within ±10m (±5m goal)
- [ ] Flight time 30+ minutes
- [ ] 200g payload capacity (camera mount)
- [ ] Full telemetry logging and post-flight analysis
- [ ] L1 guidance + TECS working in concert
- [ ] Safety: RC override, failsafe RTL, geofence
- [ ] Mission visualized in Google Earth (KML export of actual flight path)
- [ ] GitHub repo with all code, simulation, flight data
- [ ] Video: full autonomous mission from takeoff to landing approach
- [ ] Comprehensive build document (this IS your portfolio piece)

---

## Phase 1 Completion Criteria

You're done with Phase 1 when ALL of the following are true:

- [ ] You can derive the equations of motion for a rigid body from first principles
- [ ] You can implement a Kalman filter on paper (predict + update steps) without looking anything up
- [ ] You have a working quadcopter flight controller running YOUR code (not ArduPilot, not Betaflight)
- [ ] You have a fixed-wing UAV that can autonomously navigate waypoints
- [ ] You have 5 GitHub repositories with READMEs, flight data, and video evidence
- [ ] You can explain cascaded PID, complementary filters, L1 guidance, and TECS to someone else
- [ ] You have at least 2 crash reports with root cause analysis and fixes documented

**Estimated total time:** 800–1200 hours over 6 months (20–45 hours/week)

---

## Quick Reference: Key Equations

**Complementary Filter:**
`angle = α * (angle + gyro * dt) + (1 - α) * accel_angle`

**Discrete PID:**
`u[k] = Kp * e[k] + Ki * Σe * dt + Kd * (e[k] - e[k-1]) / dt`

**Kalman Predict:**
`x̂⁻ = F * x̂ + B * u`
`P⁻ = F * P * Fᵀ + Q`

**Kalman Update:**
`K = P⁻ * Hᵀ * (H * P⁻ * Hᵀ + R)⁻¹`
`x̂ = x̂⁻ + K * (z - H * x̂⁻)`
`P = (I - K * H) * P⁻`

**Motor Mixing (X-Config):**
`M1 = T - P + R - Y` (FR, CW)
`M2 = T + P + R + Y` (RR, CCW)
`M3 = T + P - R - Y` (RL, CW)
`M4 = T - P - R + Y` (FL, CCW)

**L1 Guidance:**
`a_cmd = 2 * V² * sin(η) / L1`
where `η` = angle between velocity vector and line to reference point

---

> *"Operation Shaheen is a go. Execute."*
