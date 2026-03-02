# Operation Shaheen — Theory Module 3: ArduPilot & Betaflight Source Code Guide

> **Reading other people's flight controllers is how you learn to write your own.**

---

## Why Study Open-Source Flight Controllers?

You're building from scratch in Phase 1 — but you're not the first. ArduPilot and Betaflight represent decades of collective engineering. Studying them teaches you:

1. **Architecture patterns** that work at scale
2. **Safety mechanisms** you'd never think of on your own
3. **Optimizations** learned from millions of flight hours
4. **What NOT to do** (read the git blame and issue trackers — the bugs are as educational as the fixes)

**Rule:** Study the code to understand design decisions. Don't copy-paste. Your implementation should be yours, informed by theirs.

---

## Part 1: Betaflight (Quadcopter Focus)

**Repository:** `https://github.com/betaflight/betaflight`
**Language:** C (with some C++ in newer versions)
**Target:** STM32 F4/F7/H7 microcontrollers (same family you're using!)

### Getting Started

```bash
git clone https://github.com/betaflight/betaflight.git
cd betaflight
# Don't try to build it yet — just read the source
```

### Directory Map — Where to Look

```
betaflight/
├── src/
│   └── main/
│       ├── main.c                    ← Entry point, initialization sequence
│       ├── fc/                       ← Flight controller core
│       │   ├── core.c                ← Main control loop (READ THIS FIRST)
│       │   ├── fc_core.c             ← Task scheduling
│       │   ├── runtime_config.c      ← Arming logic, flight modes
│       │   └── rc_controls.c         ← RC input processing
│       ├── flight/                   ← Flight algorithms
│       │   ├── pid.c                 ← PID controller (THE key file)
│       │   ├── mixer.c              ← Motor mixing
│       │   ├── imu.c                ← Attitude estimation (Mahony filter)
│       │   └── position.c           ← Altitude/position estimation
│       ├── sensors/                  ← Sensor drivers
│       │   ├── gyro.c               ← Gyroscope interface + filtering
│       │   ├── acceleration.c       ← Accelerometer interface
│       │   └── barometer.c          ← Barometer interface
│       ├── drivers/                  ← Hardware abstraction
│       │   ├── accgyro/             ← IMU chip drivers (BMI088, ICM-42688, etc.)
│       │   ├── barometer/           ← Baro chip drivers
│       │   └── pwm_output.c         ← Motor output (PWM, DShot)
│       └── common/
│           ├── filter.c             ← All filter implementations (biquad, PT1, etc.)
│           ├── maths.c              ← Math utilities (fast sin, atan2, etc.)
│           └── time.c               ← Timing utilities
```

### File-by-File Study Guide

#### File 1: `src/main/flight/pid.c` — START HERE

**What it contains:** The entire PID controller implementation.

**What to look for:**
- `pidController()` — the main PID function called every loop iteration
- How error is calculated (setpoint - measurement)
- D-term filtering: multiple filter stages (PT1, biquad) to tame noise
- Anti-gravity feature: boosts I-term during throttle changes to prevent dipping
- I-term relax: reduces I-term windup during fast stick movements
- Feed-forward: adds a term proportional to setpoint CHANGE (not error) — faster response
- Axis-specific tuning: roll/pitch/yaw can have different gains

**Study questions:**
1. How is the D-term computed? (Hint: it's on measurement, not error — why?)
2. What is "D-term low pass filter" and how many stages are there?
3. How does anti-windup work? What triggers it?
4. What is the "feed-forward" term and why does it improve response?
5. How does Betaflight handle the transition between different flight modes?

**Key code patterns to understand:**
```c
// This is a simplified version of what Betaflight does:
// 1. Read setpoint from RC
// 2. Read measurement from gyro
// 3. Compute error
// 4. Apply P, I, D terms with filtering
// 5. Sum and output to mixer

// Note the ORDERING: gyro read → filter → PID → mixer → motor output
// This ordering minimizes latency (gyro-to-motor delay)
```

---

#### File 2: `src/main/flight/mixer.c` — Motor Mixing

**What it contains:** Conversion from roll/pitch/yaw/throttle commands to individual motor outputs.

**What to look for:**
- Motor mixing table (different for X-config, +-config, hex, octo, etc.)
- Throttle scaling and limiting
- Motor output clamping
- Idle motor speed when armed
- Motor output ordering (which motor is which)

**Study questions:**
1. How does Betaflight handle motor saturation (when PID asks for more than 100%)?
2. What happens when one motor hits its limit? (Hint: dynamic mixing / desaturation)
3. How is yaw authority preserved when throttle is high? (Yaw is often sacrificed first)

---

#### File 3: `src/main/flight/imu.c` — Attitude Estimation

**What it contains:** Mahony complementary filter for attitude estimation.

**What to look for:**
- Mahony filter implementation (proportional-integral complementary filter)
- Quaternion representation (not Euler angles internally!)
- Gravity vector estimation from accelerometer
- DCM (Direction Cosine Matrix) computation from quaternion
- How accelerometer data is trusted only when acceleration is near 1g (detects maneuvers)

**Study questions:**
1. Why does Betaflight use Mahony filter instead of Kalman? (Hint: computational cost and it's "good enough" for racing)
2. How does the filter detect "high G" maneuvers and reduce accelerometer trust?
3. Where are quaternions converted to Euler angles? (Only for display and some outer-loop calculations)

---

#### File 4: `src/main/sensors/gyro.c` — Gyroscope Pipeline

**What it contains:** The entire gyro data pipeline from raw reading to filtered output.

**What to look for:**
- Sample rate: 8kHz for the gyro, decimated to control loop rate
- Filtering chain: typically RPM filter → notch filters → lowpass
- RPM filtering: uses motor RPM to dynamically notch out motor harmonics
- Overflow and saturation handling
- Gyro calibration at startup (bias estimation)

**Study questions:**
1. How many filter stages does gyro data pass through? Draw the signal flow diagram.
2. What is RPM filtering and why is it a breakthrough? (Hint: motor vibration frequency changes with throttle)
3. How is gyro bias calibrated? When does calibration happen?
4. What sample rate does the gyro run at vs. the PID loop?

---

#### File 5: `src/main/common/filter.c` — Filter Library

**What it contains:** All digital filter implementations used throughout Betaflight.

**Key filters to understand:**
- **PT1 (first-order low-pass):** Simple IIR filter: `y[n] = α*x[n] + (1-α)*y[n-1]`
- **PT2 (second-order low-pass):** Two PT1s cascaded
- **Biquad:** General second-order IIR filter (can be low-pass, high-pass, notch, bandpass)
- **Notch filter:** Removes specific frequency (used for motor noise)
- **Slew limiter:** Limits rate of change (prevents sudden jumps)

**Study exercise:** Implement each filter type in Python, plot their frequency responses, then compare with Betaflight's C implementation.

---

#### File 6: `src/main/fc/core.c` — The Main Loop

**What it contains:** The scheduler and main control loop.

**What to look for:**
- Task scheduling: which tasks run at which rate
- Loop timing: how jitter is measured and minimized
- Task priorities: gyro read → PID → motor output (critical path, cannot be delayed)
- Overflow handling: what happens if a task takes too long

**Architecture insight:**
```
Betaflight Main Loop (simplified):
┌─────────────────────────────────────────────────┐
│ 8kHz: Gyro Read → Gyro Filter                   │
│ 4-8kHz: PID Calculation → Motor Mixing → Output │
│ 1kHz: Accelerometer Read                         │
│ 100Hz: Barometer, GPS, OSD                       │
│ 50Hz: RC Input Processing                        │
│ 10Hz: Battery Monitoring, Telemetry              │
└─────────────────────────────────────────────────┘
```

---

### Betaflight Study Project

**Goal:** Trace a single gyro sample from hardware read to motor output.

1. Start at `gyro.c` → `gyroUpdate()` — raw value read from SPI
2. Follow through filter chain in `gyro.c`
3. Into `pid.c` → `pidController()` — filtered gyro becomes PID input
4. Into `mixer.c` → `mixerCalculateThrottleAndCurrentMotorEndpoints()` — PID output becomes motor command
5. Into `drivers/pwm_output.c` — motor command becomes hardware PWM signal

**Document this trace.** Draw a diagram. Measure the latency at each stage. This is the critical path that determines your flight controller's performance.

---

## Part 2: ArduPilot (Full Autopilot — Fixed-Wing + Multi-Rotor)

**Repository:** `https://github.com/ArduPilot/ardupilot`
**Language:** C++ (modern, well-structured)
**Vehicles:** ArduCopter, ArduPlane, ArduRover, ArduSub

### Why ArduPilot After Betaflight?

Betaflight is a racing flight controller — fast, simple, focused on manual/acro flying. ArduPilot is a full autopilot — waypoint navigation, GPS, autonomous missions, failsafes. For Operation Shaheen, ArduPilot's architecture is closer to what you're building.

### Directory Map

```
ardupilot/
├── ArduCopter/                 ← Multirotor vehicle code
│   ├── Copter.h               ← Main class definition
│   ├── mode.cpp               ← Flight mode state machine
│   ├── mode_stabilize.cpp     ← Stabilize mode (angle control)
│   ├── mode_althold.cpp       ← Altitude hold mode
│   ├── mode_auto.cpp          ← Autonomous waypoint following
│   └── motors.cpp             ← Motor output logic
├── ArduPlane/                  ← Fixed-wing vehicle code
│   ├── Plane.h
│   ├── navigation.cpp         ← L1 guidance law
│   ├── tecs.cpp               ← Total Energy Control System
│   └── servos.cpp             ← Servo output
├── libraries/
│   ├── AC_AttitudeControl/    ← Attitude controller (shared between vehicles)
│   │   ├── AC_AttitudeControl.cpp
│   │   └── AC_PosControl.cpp  ← Position controller
│   ├── AP_AHRS/               ← Attitude Heading Reference System
│   │   ├── AP_AHRS.cpp        ← AHRS main (Madgwick/EKF selector)
│   │   └── AP_AHRS_DCM.cpp   ← DCM (Direction Cosine Matrix) algorithm
│   ├── AP_NavEKF3/            ← Extended Kalman Filter (the brain)
│   │   ├── AP_NavEKF3.cpp     ← EKF main
│   │   ├── AP_NavEKF3_core.cpp
│   │   ├── AP_NavEKF3_PosVelFusion.cpp  ← GPS fusion
│   │   ├── AP_NavEKF3_MagFusion.cpp     ← Magnetometer fusion
│   │   └── AP_NavEKF3_Measurements.cpp
│   ├── AP_InertialSensor/     ← IMU abstraction layer
│   ├── AP_GPS/                ← GPS driver + NMEA/UBX parsing
│   ├── AP_Baro/               ← Barometer driver
│   ├── AP_Motors/             ← Motor mixing for all frame types
│   ├── AP_Mission/            ← Waypoint/mission management
│   ├── AP_L1_Control/         ← L1 guidance controller
│   │   └── AP_L1_Control.cpp  ← THE L1 implementation (read for Project 5)
│   ├── AP_TECS/               ← Total Energy Control System
│   │   └── AP_TECS.cpp        ← THE TECS implementation (read for Project 5)
│   ├── AP_Arming/             ← Pre-arm checks and arming logic
│   └── AP_Scheduler/          ← Task scheduler
```

### File-by-File Study Guide

#### File 1: `libraries/AP_NavEKF3/AP_NavEKF3_core.cpp` — The EKF

**This is the most important file in ArduPilot.** It's the 24-state Extended Kalman Filter that fuses all sensor data into a single state estimate.

**States (24):**
```
[0-3]   Quaternion (orientation)
[4-6]   Velocity NED
[7-9]   Position NED
[10-12] Gyro biases (drift compensation)
[13-15] Accel biases (scale factor errors)
[16-18] Earth magnetic field
[19-21] Body magnetic field
[22]    Wind velocity N (for fixed-wing)
[23]    Wind velocity E
```

**What to study:**
- State prediction: how IMU data propagates the state forward
- Covariance prediction: how P matrix is propagated
- GPS fusion: how GPS measurements update position/velocity
- Magnetometer fusion: how mag updates heading
- Innovation testing: how ArduPilot detects bad sensor data and rejects it
- Multi-EKF: ArduPilot runs multiple EKF instances and cross-checks them

**Study questions:**
1. How does the EKF handle GPS outages? (Look for `gpsNotAvailable` flag)
2. How are gyro biases estimated? (They drift — the EKF continuously recalibrates)
3. What is "innovation" and how is it used to detect sensor faults?
4. How does the EKF initialize? (It needs GPS lock + valid mag + stationary period)

---

#### File 2: `libraries/AP_L1_Control/AP_L1_Control.cpp` — L1 Guidance

**Critical for Project 5 (Mahogany Bomber).**

**The L1 algorithm:**
1. Find the closest point on the desired path
2. Project a reference point L1 distance ahead on the path
3. Compute the bearing to the reference point
4. Command a lateral acceleration proportional to the bearing error

**Key function:** `update_waypoint()` — computes commanded bank angle to track between two waypoints.

**Study questions:**
1. How is L1 distance computed? (It's proportional to airspeed: `L1 = 1/(π * damping) * L1_period * airspeed`)
2. What happens at waypoint transitions? (Look for "WP radius" logic)
3. How does L1 handle crosswind? (The path-following naturally compensates)
4. What is the difference between L1 and pure pursuit guidance?

---

#### File 3: `libraries/AP_TECS/AP_TECS.cpp` — Total Energy Control

**Also critical for Project 5.**

**The TECS concept:**
- Total energy = kinetic (½mv²) + potential (mgh)
- Throttle controls total energy (add or remove energy from the system)
- Pitch controls energy distribution (trade speed for altitude or vice versa)

**Key functions:**
- `_update_energies()` — computes current and demanded energy states
- `_update_throttle()` — throttle command from total energy error
- `_update_pitch()` — pitch command from energy distribution error

**Study questions:**
1. How does TECS handle a "climb at constant speed" command? (Throttle increases to add energy, pitch increases to direct energy into altitude)
2. What happens when you're too slow AND too low? (Energy emergency — TECS prioritizes speed over altitude because stall kills)
3. How does TECS limit sink rate? (Prevents exceeding structural limits)

---

#### File 4: `ArduCopter/mode.cpp` — Flight Mode State Machine

**Study the state machine pattern — you'll use it in every project.**

```
STABILIZE → ALT_HOLD → LOITER → AUTO → RTL → LAND
    ↑                                         │
    └─────────── DISARM ◄─────────────────────┘
```

**What to look for:**
- How modes transition safely (can't go from DISARM to AUTO directly)
- What checks happen on mode change
- How each mode sets controller targets differently
- Failsafe mode entries (GPS loss → ALT_HOLD, RC loss → RTL)

---

#### File 5: `libraries/AP_Arming/AP_Arming.cpp` — Safety

**Study this carefully. It will inform your safety system in Project 4.**

**Pre-arm checks include:**
- IMU calibration valid
- Compass calibrated and healthy
- GPS lock with sufficient satellites
- Battery voltage above threshold
- RC calibration valid
- EKF healthy (innovations within limits)
- Accelerometer clipping not detected
- No hardware failures

**Study questions:**
1. How many individual checks does ArduPilot perform before allowing arm?
2. What is the difference between pre-arm checks and arm checks?
3. How does ArduPilot handle sensor redundancy? (It often has 2–3 IMUs)

---

### ArduPilot Study Project

**Goal:** Trace an autonomous waypoint mission from start to finish.

1. `AP_Mission` loads waypoint list
2. `mode_auto.cpp` enters AUTO mode, reads first waypoint
3. `AP_L1_Control` computes lateral guidance (bank angle command)
4. `AP_TECS` computes longitudinal guidance (throttle + pitch commands)
5. `AC_AttitudeControl` converts bank/pitch commands to rate commands
6. Rate PID computes servo deflections
7. `servos.cpp` outputs PWM to control surfaces

**Draw this as a block diagram.** Label the inputs and outputs of each block. This is the architecture you're building in Project 5.

---

## Part 3: Key Architectural Lessons

### Lesson 1: Layered Architecture

Both Betaflight and ArduPilot separate concerns into layers:
```
Mission / Navigation  (what to do)
    ↓
Guidance             (where to go)
    ↓
Control              (how to get there)
    ↓
Motor/Servo Mixing   (physical actuation)
    ↓
Hardware Drivers     (talk to chips)
```

Each layer only talks to adjacent layers. This makes the system modular and testable.

### Lesson 2: Sensor Abstraction

Both codebases abstract sensors behind interfaces. The PID controller doesn't know (or care) if you're using an MPU-6050 or a BMI088. It just calls `get_gyro()`. This means you can swap sensors without touching control code.

**Apply this in your code:** Create `imu.h` with a clean API. Driver details stay in `imu.c`.

### Lesson 3: Fail-Safe Philosophy

ArduPilot's safety model: **every failure mode has a defined response.**

| Failure | Response |
|---|---|
| GPS loss | Switch to altitude hold mode |
| RC loss | Return to launch |
| Battery low | Return to launch |
| EKF divergence | Switch to DCM backup AHRS |
| Motor failure (hex+) | Continue on remaining motors |
| Geofence breach | Return to launch |
| Sensor disagreement | Trust the majority (voting) |

**Your Project 4 must have a similar table. Define every failure mode BEFORE first flight.**

### Lesson 4: Configuration, Not Code Changes

Both Betaflight and ArduPilot have hundreds of configurable parameters. PID gains, filter frequencies, failsafe thresholds — all changeable without recompiling. This is critical for field tuning.

**Apply this:** Store all tunable parameters in a `config.h` or EEPROM-backed parameter system. Never hard-code tuning values deep in your control loop.

---

## Study Schedule

| Week | Focus | Time |
|---|---|---|
| Project 1–2 period | Betaflight: `pid.c`, `mixer.c`, `filter.c` | 3 hrs/week |
| Project 3 period | ArduPilot: `AP_NavEKF3` (EKF study) | 4 hrs/week |
| Project 4 period | Betaflight: `core.c`, `gyro.c`, `imu.c` (architecture) | 3 hrs/week |
| Project 4 period | ArduPilot: `AP_Arming` (safety patterns) | 2 hrs/week |
| Project 5 period | ArduPilot: `AP_L1_Control`, `AP_TECS`, `mode_auto.cpp` | 5 hrs/week |

**Total:** ~3–5 hours per week of code reading, parallel to your build projects.

---

## How to Read Large Codebases (Process)

1. **Start with the entry point** (`main.c` or `main()`) and follow the initialization sequence
2. **Identify the main loop** — what runs every iteration?
3. **Trace one data path** end-to-end (gyro sample → motor output)
4. **Read the header files first** — they tell you the API contract (what functions exist, what parameters they take)
5. **Use `git log --oneline -20 <file>`** to see recent changes — they often fix bugs that teach you about edge cases
6. **Read the GitHub issues** for a file — they explain why design decisions were made
7. **Don't try to understand everything** — focus on the subsystem relevant to your current project

---

> *"Good engineers write code. Great engineers read code first."*
