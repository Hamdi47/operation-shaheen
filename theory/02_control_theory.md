# Operation Shaheen — Theory Module 2: Control Theory

> **PID is chapter one. The real book is much longer.**

---

## Roadmap

```
Week 1-2:  Classical Control (transfer functions, poles/zeros, stability)
Week 3-4:  Frequency Domain (Bode plots, Nyquist, gain/phase margins)
Week 5-6:  State-Space Methods (modern control, controllability, observability)
Week 7-8:  Digital Control (discretization, z-transform, sample rate effects)
Week 9-12: Advanced Methods (LQR, MPC, adaptive, robust — introduced, not mastered)
```

**Primary Texts:**
1. *Feedback Control of Dynamic Systems* — Franklin, Powell, Emami-Naeini (Chapters 1–10)
2. *Modern Control Engineering* — Ogata (alternative/supplement)
3. *Model Predictive Control* — Rawlings et al. (for MPC deep dive in Phase 3)

**Software:** MATLAB/Octave (Control System Toolbox) or Python (`control` library: `pip install control`)

---

## Part 1: Classical Control Foundations (Weeks 1–4)

### 1.1 — System Modeling with Transfer Functions

**What is a transfer function?**

It's the Laplace transform of the input-output relationship. For a linear time-invariant system:

```
G(s) = Y(s) / U(s) = Output / Input  (in s-domain)
```

**Drone example — pitch dynamics (simplified):**

If you apply a torque τ to the pitch axis, the pitch angle θ evolves as:
```
I_yy * θ̈ = τ    (Newton's second law for rotation)
```

Taking the Laplace transform:
```
I_yy * s² * Θ(s) = T(s)
G(s) = Θ(s)/T(s) = 1 / (I_yy * s²)
```

This is a **double integrator** — no natural damping, no restoring force. Without a controller, any disturbance causes the drone to keep rotating forever. This is why quadcopters are inherently unstable and NEED active control.

**Problem Set 1.1:**

1. Derive the transfer function for the following systems:
   - Mass-spring-damper: `m*ẍ + b*ẋ + k*x = F` → `G(s) = 1/(ms² + bs + k)`
   - RC circuit (low-pass filter): `G(s) = 1/(RCs + 1)` — this appears in your derivative filter!
   - DC motor: armature voltage → shaft speed

2. **Drone exercise:** Given pitch inertia I_yy = 0.01 kg·m²:
   - Write the open-loop transfer function
   - Add a PD controller: C(s) = Kp + Kd*s
   - Closed-loop: `G_cl(s) = C(s)*G(s) / (1 + C(s)*G(s))`
   - Simplify. What kind of system is it now? (second order with damping)

3. **Python:**
   ```python
   import control
   # Open loop: double integrator
   G = control.tf([1], [0.01, 0, 0])
   # PD controller: Kp=5, Kd=0.5
   C = control.tf([0.5, 5], [1])
   # Closed loop
   G_cl = control.feedback(C * G)
   # Step response
   t, y = control.step_response(G_cl)
   # Plot it — observe overshoot, settling time
   ```

---

### 1.2 — Poles, Zeros, and Stability

**Poles** = roots of the denominator of G(s) = values of s where G(s) → ∞
**Zeros** = roots of the numerator = values of s where G(s) = 0

**Stability rule:** A system is stable if and only if ALL poles have negative real parts (left half of s-plane).

**Pole locations and behavior:**

| Pole Location | System Behavior |
|---|---|
| Real, negative (e.g., s = -3) | Exponential decay, no oscillation |
| Complex, negative real part (e.g., s = -2 ± 3j) | Damped oscillation |
| Pure imaginary (e.g., s = ±3j) | Undamped oscillation (marginally stable) |
| Positive real part | Unstable — exponential growth |
| s = 0 | Integrator — output ramps with constant input |

**Problem Set 1.2:**

1. For the closed-loop system from Problem Set 1.1 (pitch with PD):
   - Find poles as a function of Kp and Kd
   - Plot pole locations for Kp = [1, 5, 10, 20] with Kd = 1
   - Identify: underdamped, critically damped, overdamped regions
   - What Kp makes poles purely real? (critical damping: Kd² = 4*I_yy*Kp)

2. **Root locus by hand:** For `G(s) = 1/(s² + 2s + 5)` with proportional controller K:
   - Open-loop poles at s = -1 ± 2j
   - As K increases from 0 → ∞, where do closed-loop poles go?
   - Sketch the root locus (poles start at open-loop poles, move toward zeros or infinity)

3. **Python root locus:**
   ```python
   import control
   G = control.tf([1], [0.01, 0, 0])  # quadcopter pitch
   control.root_locus(G)  # see where poles go as gain increases
   # This shows you EXACTLY why high gain causes oscillation
   ```

---

### 1.3 — Second-Order System Analysis

Most drone subsystems can be approximated as second-order. Master this and you understand 80% of tuning.

**Standard form:**
```
G(s) = ωn² / (s² + 2ζωn*s + ωn²)
```

Where:
- `ωn` = natural frequency (how fast the system responds)
- `ζ` = damping ratio (how much oscillation)

**Key relationships:**

| ζ (damping ratio) | Behavior | Overshoot |
|---|---|---|
| ζ = 0 | Undamped oscillation | ∞ (never settles) |
| 0 < ζ < 1 | Underdamped (oscillates then settles) | e^(-πζ/√(1-ζ²)) × 100% |
| ζ = 1 | Critically damped (fastest without overshoot) | 0% |
| ζ > 1 | Overdamped (sluggish, no overshoot) | 0% |

**For drones:** ζ ≈ 0.7–0.8 is usually ideal. Some overshoot for speed, but settles quickly.

**Problem Set 1.3:**

1. **By hand:** For your quadcopter pitch axis with PD control (Kp=5, Kd=0.5, I=0.01):
   - Write closed-loop in standard form
   - Calculate ωn and ζ
   - Predict: overshoot %, settling time (ts ≈ 4/(ζωn)), rise time
   - Verify with Python step response simulation
   - Now add Ki = 2 (PID). How does this change the transfer function? (adds a zero and changes the order)

2. **Design by specification:** You want <5% overshoot and <0.5 second settling time.
   - Calculate required ζ (from overshoot formula)
   - Calculate required ωn (from settling time formula)
   - Solve for Kp and Kd that achieve this
   - Simulate to verify

3. **Drone Connection:** When Betaflight shows you "D-term filtering" settings, it's adding a low-pass filter to the derivative. In transfer function terms, this replaces `Kd*s` with `Kd*s/(τs + 1)`. This limits the high-frequency gain of D, reducing noise amplification. Simulate both and see the difference.

---

### 1.4 — Frequency Domain Analysis (Bode Plots)

**Why frequency domain?** Time-domain analysis tells you about step response. Frequency domain tells you about robustness — how the system handles sinusoidal disturbances (vibrations, wind gusts) at different frequencies.

**Bode plot:** Magnitude and phase of G(jω) as a function of frequency ω.

**Key concepts:**

- **Gain crossover frequency (ωgc):** Where |G(jω)| = 1 (0 dB). This is approximately the bandwidth.
- **Phase margin:** 180° + phase at gain crossover. Want >45° for robust stability.
- **Gain margin:** 1/|G(jω)| at phase = -180°. Want >6 dB.
- **Bandwidth:** Frequency range where the system tracks commands. Higher = faster response.

**Problem Set 1.4:**

1. **Bode plot by hand (sketching rules):**
   - First-order: G(s) = 1/(τs + 1) — magnitude drops at -20 dB/decade after ω = 1/τ
   - Second-order: G(s) = ωn²/(s² + 2ζωn*s + ωn²) — drops at -40 dB/decade after ωn, with resonance peak at ωn if ζ < 0.707
   - Integrator: G(s) = 1/s — magnitude drops at -20 dB/decade everywhere, phase is -90° everywhere
   - PID: C(s) = Kp(1 + 1/(Ti*s) + Td*s) — sketch the magnitude. I-term boosts low frequencies, D-term boosts high frequencies.

2. **Python Bode analysis:**
   ```python
   import control
   # Quadcopter pitch: open-loop with PID
   G = control.tf([1], [0.01, 0, 0])
   C = control.tf([0.5, 5, 2], [1, 0])  # PID: Kd*s² + Kp*s + Ki / s
   L = C * G  # open-loop transfer function
   
   control.bode_plot(L)
   gm, pm, wgc, wpc = control.margin(L)
   print(f"Gain margin: {gm:.1f} dB, Phase margin: {pm:.1f}°")
   # If phase margin < 30°, your quad will oscillate!
   ```

3. **Noise sensitivity analysis:**
   - Plot |S(jω)| where S = 1/(1 + L) is the sensitivity function
   - Plot |T(jω)| where T = L/(1 + L) is the complementary sensitivity
   - S + T = 1 (fundamental tradeoff!)
   - S small at low freq = good disturbance rejection
   - T small at high freq = good noise rejection
   - You CANNOT have both everywhere — this is the fundamental limitation of feedback

4. **Drone Connection:** When you see "gyro filter frequency" in Betaflight (typically 100–300 Hz), you're shaping T(jω). Motor noise at 200+ Hz needs to be filtered. But filter too aggressively and you lose phase margin → oscillation. This is the core tuning tradeoff in every flight controller.

---

## Part 2: State-Space Methods (Weeks 5–6)

### 2.1 — State-Space Representation

Transfer functions work for single-input single-output (SISO) systems. Drones are multi-input multi-output (MIMO). State-space handles this naturally.

**Standard form:**
```
ẋ = Ax + Bu    (state equation — dynamics)
y = Cx + Du    (output equation — measurements)
```

Where:
- x = state vector (what you want to know: position, velocity, angles...)
- u = input vector (what you command: motor thrusts, servo positions)
- y = output vector (what you measure: GPS, IMU, barometer)
- A = system dynamics matrix
- B = input matrix
- C = output matrix (what you can measure)
- D = feedthrough matrix (usually zero)

**Quadcopter state-space model (linearized about hover):**

State: `x = [φ, θ, ψ, p, q, r, x, y, z, vx, vy, vz]` (12 states)
- φ, θ, ψ = roll, pitch, yaw angles
- p, q, r = angular rates
- x, y, z = position
- vx, vy, vz = velocity

Input: `u = [T, τx, τy, τz]` (total thrust + three torques)

**Problem Set 2.1:**

1. **Linearized pitch dynamics in state-space:**
   ```
   State: x = [θ, q]  (pitch angle, pitch rate)
   Input: u = τ_pitch  (pitch torque from motors)
   
   ẋ = [0    1 ] x + [  0    ] u
       [0    0 ]     [1/I_yy]
   
   y = [1  0] x  (we measure angle)
   ```
   - Verify: this is equivalent to the transfer function G(s) = 1/(I_yy * s²)
   - Compute eigenvalues of A — both are 0 (marginally stable, double integrator)
   - Add state feedback: u = -Kx = -[k1, k2]*x. New dynamics: A_cl = A - B*K
   - Choose k1, k2 to place eigenvalues at s = -5 ± 5j (fast, slightly underdamped)

2. **Full 6-DOF linearization:** Write out the full 12×12 A matrix for a quadcopter hover. Most entries are 0. The key couplings:
   - θ → ax (pitch causes forward acceleration via gravity component: ax ≈ -g*θ)
   - φ → ay (roll causes lateral acceleration: ay ≈ g*φ)
   - T → az (thrust controls vertical acceleration)

---

### 2.2 — Controllability and Observability

**Controllability:** Can you drive the system from any state to any other state using the inputs?
```
Controllability matrix: C = [B, AB, A²B, ..., Aⁿ⁻¹B]
System is controllable if rank(C) = n (number of states)
```

**Observability:** Can you determine the full state from the outputs (measurements)?
```
Observability matrix: O = [C; CA; CA²; ...; CAⁿ⁻¹]
System is observable if rank(O) = n
```

**Problem Set 2.2:**

1. For the pitch dynamics (2-state model):
   - Compute controllability matrix [B, AB]. Check rank = 2 → controllable ✓
   - Compute observability matrix [C; CA]. Check rank = 2 → observable ✓
   - Now remove the gyroscope: C = [1, 0] (only angle measurement). Still observable?
   - Now remove the angle sensor: C = [0, 1] (only rate measurement). Still observable?
   - Answer: yes to both! One sensor is enough for a 2-state system. But noise performance differs.

2. **Drone Connection:** If your IMU fails and you only have GPS (measures position, not angle), are your angle states observable? Compute the observability matrix and find out. Spoiler: not directly, but if you model the dynamics correctly, you can infer attitude from acceleration patterns. This is how GPS-only attitude estimation works (poorly, but it works).

3. **Design implication:** If a state is not observable, the Kalman filter CANNOT estimate it. Your P matrix for that state will grow without bound. This is why sensor selection matters — you need enough independent sensors to observe all states you want to estimate.

---

### 2.3 — LQR (Linear Quadratic Regulator)

**The optimal full-state feedback controller.** Instead of manually tuning PID gains, LQR computes the "best" gains by minimizing a cost function.

**Cost function:**
```
J = ∫₀^∞ (xᵀQx + uᵀRu) dt
```

- Q matrix: penalizes state deviations (big Q → aggressive tracking)
- R matrix: penalizes control effort (big R → gentle control, saves energy)

**Solution:** u = -Kx where K = R⁻¹BᵀP, and P is the solution to the algebraic Riccati equation.

**Problem Set 2.3:**

1. **LQR for pitch control:**
   ```python
   import control
   import numpy as np
   
   I_yy = 0.01
   A = np.array([[0, 1], [0, 0]])
   B = np.array([[0], [1/I_yy]])
   
   # Tuning: Q penalizes angle error and rate, R penalizes torque
   Q = np.diag([100, 1])   # care about angle more than rate
   R = np.array([[1]])       # moderate control effort penalty
   
   K, S, E = control.lqr(A, B, Q, R)
   print(f"LQR gains: Kp={K[0,0]:.2f}, Kd={K[0,1]:.2f}")
   print(f"Closed-loop eigenvalues: {E}")
   ```

2. **Q and R tuning intuition:**
   - Double Q[0,0] (angle weight) → Kp increases → faster angle response
   - Double R[0,0] (effort weight) → both gains decrease → gentler control
   - Try Q = diag([1, 1]) vs Q = diag([1000, 1]) — plot step responses
   - LQR is "optimal" PID tuning. The gains it gives are a great starting point for manual fine-tuning.

3. **Extend to 6-DOF:** Implement LQR for the full 12-state quadcopter model. Compare LQR gains against your hand-tuned PID gains from Project 4. Are they similar?

4. **Drone Connection:** DJI's flight controllers reportedly use LQR-based gain scheduling. The gains change with flight condition (hover vs. fast forward flight) because the linearized model changes. This is called gain scheduling and it's the bridge between LQR and real-world implementation.

---

## Part 3: Digital Control (Weeks 7–8)

### 3.1 — Continuous to Discrete Conversion

Your control theory is in continuous time (s-domain). Your STM32 runs in discrete time (z-domain). You need to convert.

**Discretization methods:**

| Method | Formula | Accuracy | Use When |
|---|---|---|---|
| Forward Euler | z = 1 + T*s | Low, can be unstable | Never (for control) |
| Backward Euler | z = 1/(1 - T*s) | Low, always stable | Simple filters |
| Bilinear (Tustin) | z = (1 + T*s/2)/(1 - T*s/2) | Good, preserves stability | Default choice |
| Zero-Order Hold (ZOH) | exact for sample-hold | Exact for the model | When using `control.c2d()` |

**Problem Set 3.1:**

1. **Discretize PID:**
   Starting from continuous PID: `C(s) = Kp + Ki/s + Kd*s`
   - Integral term (Ki/s): Forward Euler → `Ki * T * Σe[k]`, Tustin → `Ki * T/2 * (e[k] + e[k-1])`
   - Derivative term (Kd*s): Backward difference → `Kd * (e[k] - e[k-1]) / T`
   - Compare: simulate continuous PID vs. discrete (both methods) on same plant
   - At 1kHz sample rate, difference is tiny. At 100Hz, it matters.

2. **Sample rate effects:**
   - Simulate your pitch controller at 1kHz, 500Hz, 250Hz, 100Hz, 50Hz
   - At what sample rate does performance degrade noticeably? (typically when T > 1/(10*ωn))
   - At what sample rate does it go unstable?
   - Rule of thumb: sample at 10–20× the desired closed-loop bandwidth

3. **Aliasing:** If your motor vibration is at 400Hz and you sample gyro at 500Hz, you get aliased noise at 100Hz — right in your control bandwidth! This is why Betaflight uses 8kHz gyro sampling even though the control loop runs at 4–8kHz.

---

### 3.2 — Z-Transform Basics

**Z-transform** is to discrete systems what Laplace is to continuous.

```
x[k+1] = A_d * x[k] + B_d * u[k]     (discrete state equation)
y[k] = C * x[k]
```

**Key z-transforms:**
- Unit delay: Z{x[k-1]} = z⁻¹ * X(z)
- Accumulator (discrete integral): H(z) = T/(1 - z⁻¹)
- Difference (discrete derivative): H(z) = (1 - z⁻¹)/T

**Problem Set 3.2:**

1. Write the z-transfer function of your discrete PID controller
2. Compute discrete-time poles of your closed-loop system. Stability: all poles inside unit circle (|z| < 1)
3. Map between s-plane and z-plane: `z = e^(sT)`. Where do the s-plane stability regions map to?

---

## Part 4: Advanced Control Methods (Weeks 9–12, introduction)

> These are covered in depth in Phase 3. Here you build intuition and implement simple versions.

### 4.1 — Model Predictive Control (MPC)

**The big idea:** Instead of reacting to current error (PID), MPC **predicts** the future and optimizes the entire trajectory over a horizon.

```
At each timestep:
1. Predict system state N steps into the future using the model
2. Optimize control inputs u[0], u[1], ..., u[N-1] to minimize cost
3. Apply only u[0]
4. Repeat at next timestep (receding horizon)
```

**Why MPC is the future of drone control:**
- Handles constraints naturally (max motor thrust, angle limits, no-fly zones)
- Can optimize for multiple objectives (track path + minimize energy + stay safe)
- Can look ahead (anticipate upcoming waypoint turn instead of reacting late)

**Problem Set 4.1 (introductory):**

1. **1D MPC in Python:**
   ```python
   import numpy as np
   from scipy.optimize import minimize
   
   # System: double integrator (altitude control)
   # State: [height, velocity]
   # Input: acceleration (thrust - gravity)
   # Horizon: N = 20 steps at dt = 0.05s
   
   # Cost: sum of (height_error² + 0.1*acceleration²) over horizon
   # Constraint: -5 < acceleration < 15 m/s² (can't push down as hard as up)
   
   # Implement and compare with PID on altitude step response
   ```

2. **Observe:** MPC smoothly accelerates then decelerates (S-curve profile). PID overshoots because it doesn't plan ahead. This is why MPC gives smoother flights.

3. **Computational cost:** How long does your MPC take to solve? Can it run at 100Hz on your STM32? (Probably not for complex problems — this is why MPC is often run on companion computers like Jetson Nano, not the flight controller MCU.)

---

### 4.2 — Sliding Mode Control (Introduction)

**The idea:** Design a "sliding surface" in state space. Drive the system to the surface, then constrain it to slide along the surface to the target.

**Why it matters for drones:** Extremely robust to disturbances and model uncertainty. Wind gust? Payload change? Sliding mode doesn't care (much).

**Simplified example for pitch:**
```
Sliding surface: s = ė + λ*e  (where e = θ_desired - θ)
Control law: τ = τ_eq + τ_sw
  τ_eq = equivalent control (what nominal model needs)
  τ_sw = -K * sign(s)  (switching term that enforces sliding)
```

**Problem:** The `sign()` function causes chattering (high-frequency switching). Solutions: boundary layer (replace sign with saturation), higher-order sliding mode.

**Problem Set 4.2:**
1. Implement sliding mode for 1-DOF pitch stabilization in Python simulation
2. Compare with PID: apply a sudden disturbance torque (simulating wind gust)
3. Observe: sliding mode recovers faster and with less deviation
4. Implement boundary layer to reduce chattering, observe the tradeoff

---

### 4.3 — Gain Scheduling

**The practical bridge between theory and real drones.**

Your linear models are only valid near the operating point (e.g., hover). At high speed or large angles, the linear approximation breaks down.

**Solution:** Design multiple controllers for different operating regions. Interpolate between them.

```
if airspeed < 5 m/s:
    use hover_gains  (high angle authority, low speed control)
elif airspeed < 20 m/s:
    use transition_gains  (blend)
else:
    use cruise_gains  (low angle authority, aerodynamic control)
```

**Problem Set 4.3:**
1. Linearize quadcopter dynamics at hover (θ ≈ 0) and at 30° pitch (forward flight)
2. Design separate LQR controllers for each operating point
3. Simulate: fly from hover to forward flight. With fixed hover gains, observe degraded performance at 30°. With scheduled gains, observe smooth transition.

---

## Part 5: Control Theory Applied to Flight Controllers

### Reference Architecture (Betaflight/PX4 style)

```
                    ┌──────────────┐
 RC Input ─────────►│  Angle PID   │──── Rate Setpoint ────►┌──────────┐
                    │ (outer loop) │                         │ Rate PID │──── Motor
                    └──────────────┘                         │(inner lp)│    Mixing
                          ▲                                  └──────────┘
                          │                                       ▲
                    Filtered Angle                           Raw Gyro
                    (Complementary/                          (1kHz+)
                     Kalman/Mahony)
```

**Key design decisions and why:**

1. **Why cascaded (inner/outer) instead of single PID?**
   - Inner rate loop rejects disturbances FAST (gyro is fast, no filter delay)
   - Outer angle loop handles slower setpoint tracking
   - You can tune them independently
   - Bandwidth separation: inner loop 3–5× faster than outer loop

2. **Why rate loop uses raw gyro (not filtered angle)?**
   - Gyro directly measures angular rate — no computation needed
   - Any filtering adds delay → reduces phase margin → reduces max gain
   - Inner loop needs to be as fast as possible

3. **Why 1kHz+ control rate?**
   - Motor vibrations are 100–400Hz
   - Control loop needs to be >2× the highest disturbance frequency (Nyquist)
   - More headroom = more stability margin = higher gains possible = better performance

4. **D-term filtering:**
   - Derivative amplifies noise
   - Betaflight uses biquad low-pass filters on D-term (typically 100–150Hz cutoff)
   - This is a tradeoff: less noise vs. more phase lag on D-term

**Problem Set (applied):**
1. In your Project 4 simulation, implement the cascaded architecture
2. Tune inner loop first with outer loop disabled
3. Verify bandwidth separation: inner loop bandwidth should be 3–5× outer loop
4. Add sensor noise to simulation, verify D-term filter is necessary
5. Try removing the inner rate loop — observe how much worse performance gets

---

## Control Theory Reading List (prioritized)

### Must Read (Phase 1):
1. Franklin, Powell, Emami-Naeini — Ch 1–7 (fundamentals + frequency domain)
2. Åström & Murray — "Feedback Systems" (free online, excellent modern treatment)
3. "Quadrotor Control" — Bouabdallah et al. (the classic quadcopter control paper)

### Should Read (Phase 1–2):
4. Ogata — "Modern Control Engineering" (good for state-space)
5. Beard & McLain — Ch 5–6 (autopilot design for small UAS)
6. "PX4 Autopilot: An Open-Source Flight Control Solution" — Meier et al.

### Want to Read (Phase 2–3):
7. Rawlings et al. — "Model Predictive Control" (the MPC bible)
8. Slotine & Li — "Applied Nonlinear Control" Ch 6–7 (sliding mode)
9. "Aggressive Flight with Quadrotors" — Mellinger & Kumar (pushing control limits)

---

## Quick Reference: Control Design Checklist

When designing ANY controller for your drone:

- [ ] **Model:** Write the plant transfer function or state-space model
- [ ] **Requirements:** Define overshoot, settling time, bandwidth, stability margins
- [ ] **Simulate:** Test controller in simulation with realistic noise and disturbances
- [ ] **Analyze:** Check gain margin (>6 dB) and phase margin (>45°) 
- [ ] **Discretize:** Convert to discrete time at your sample rate
- [ ] **Implement:** Code in C with anti-windup, output limits, derivative filtering
- [ ] **Test:** Bench test with controlled inputs before flight test
- [ ] **Log:** Record EVERYTHING during flight test
- [ ] **Iterate:** Analyze logs, adjust, repeat

---

> *"A well-tuned controller is invisible. You only notice control theory when it fails."*
