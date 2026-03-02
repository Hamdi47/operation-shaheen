# Operation Shaheen — Theory Module 1: Mathematics

> **The language your drone speaks. If you can't read it, you can't command it.**

---

## How to Use This File

This is your math study curriculum for Phase 1. It runs **parallel** to your build projects. Don't try to finish all math before building — interleave them. When a project needs a concept, you should have already studied it that week.

**Time commitment:** ~8–10 hours/week for the first 8 weeks, then drops to 3–5 hours/week as you shift to projects.

**Rules:**
1. Do problems by hand first, then verify in Python/MATLAB
2. Every concept has a "Drone Connection" — if you can't explain the connection, you haven't learned it
3. Keep a physical notebook. Write derivations. Draw diagrams. Your hands remember what your eyes forget.

---

## Part 1: Linear Algebra (Weeks 1–4)

**Primary Text:** *Introduction to Linear Algebra* — Gilbert Strang, 5th/6th Edition
**Video Lectures:** MIT 18.06 (Gilbert Strang) — free on MIT OCW and YouTube
**Supplementary:** 3Blue1Brown "Essence of Linear Algebra" (watch first for intuition)

### Week 1: Vectors, Matrices, and Transformations

**Read:** Strang Ch 1 (Vectors) + Ch 2 (Solving Linear Equations)

**Core Concepts:**
- Vector addition, scalar multiplication, dot product, cross product
- Matrix-vector multiplication as linear transformation
- Systems of linear equations: Ax = b
- Gaussian elimination, row echelon form
- Matrix inverse (when it exists and when it doesn't)

**Problem Set (do ALL of these):**

1. **By hand:** Solve a 3×3 system using Gaussian elimination:
   ```
   2x + y - z = 8
   -3x - y + 2z = -11
   -2x + y + 2z = -3
   ```

2. **Rotation matrices:** Write the 2D rotation matrix R(θ). Verify that R(30°) × R(60°) = R(90°). Then verify R(θ)ᵀ = R(-θ) = R(θ)⁻¹. This is why rotation matrices are "orthogonal."

3. **By hand:** Compute the 3D rotation matrix for a 45° rotation about the Z-axis. Apply it to the vector [1, 0, 0]ᵀ. Where does it end up?

4. **Python exercise:**
   ```python
   import numpy as np
   # Create a 3x3 rotation matrix for roll=10°, pitch=20°, yaw=30°
   # Apply it to the gravity vector [0, 0, -9.81]
   # What does the accelerometer "see" when the drone is tilted?
   # This is EXACTLY how you convert between body frame and world frame.
   ```

5. **Drone Connection:** Your IMU measures acceleration in the *body frame* (attached to the drone). Gravity points down in the *world frame*. The rotation matrix R converts between them: `a_body = R * a_world`. Write code to compute what the accelerometer reads at 15° pitch.

**Strang exercises to complete:** Ch 1: Problems 1, 3, 7, 9, 15. Ch 2: Problems 1, 4, 8, 12, 17, 24.

---

### Week 2: Vector Spaces and Linear Independence

**Read:** Strang Ch 3 (Vector Spaces and Subspaces) + Ch 4 (Orthogonality)

**Core Concepts:**
- Vector spaces, subspaces, column space, null space
- Linear independence, span, basis, dimension
- Rank of a matrix (relates to: is your system controllable/observable?)
- Orthogonality, orthogonal projection, least squares
- Gram-Schmidt orthogonalization

**Problem Set:**

1. **By hand:** Given vectors v1 = [1, 2, 3], v2 = [2, 4, 6], v3 = [1, 0, 1], determine which are linearly independent. What's the rank of the matrix [v1 | v2 | v3]?

2. **Least squares — THE foundation of sensor fusion:**
   You have 10 noisy measurements of altitude: [100.2, 99.8, 100.5, 99.7, 100.1, 100.4, 99.9, 100.3, 100.0, 100.2]. The true altitude is constant.
   - Set up as Ax = b where A is a column of ones, x is the estimated altitude, b is your measurements
   - Solve using least squares: x = (AᵀA)⁻¹Aᵀb
   - Verify this gives you the mean (it should — least squares on a constant IS the mean)
   - Now extend: fit a line (altitude = a*t + b) to data where the drone is climbing

3. **Python exercise:**
   ```python
   import numpy as np
   # Generate 100 noisy GPS positions: true position + Gaussian noise
   # Use least squares to estimate the true position
   # Plot raw vs. filtered positions
   # This is the simplest possible "sensor fusion" — you'll extend this to Kalman
   ```

4. **Orthogonal projection:** Project the vector [3, 4] onto the vector [1, 0]. Then onto [1, 1]/√2. Draw both. Understand: the Kalman filter update step IS an orthogonal projection.

5. **Drone Connection:** When your IMU has 6 measurements (3 accel + 3 gyro) but you want 3 outputs (roll, pitch, yaw), you're solving an overdetermined system. Least squares gives you the "best" estimate. This is literally what your complementary filter approximates.

**Strang exercises:** Ch 3: Problems 2, 5, 8, 14, 21. Ch 4: Problems 1, 3, 9, 13, 17.

---

### Week 3: Eigenvalues, Eigenvectors, and Stability

**Read:** Strang Ch 5 (Determinants) + Ch 6 (Eigenvalues and Eigenvectors)

**Core Concepts:**
- Determinant (geometric meaning: volume scaling)
- Eigenvalues and eigenvectors: Ax = λx
- Characteristic polynomial: det(A - λI) = 0
- Diagonalization: A = PDP⁻¹
- Stability criterion: if ALL eigenvalues have negative real parts, the system is stable

**Problem Set:**

1. **By hand:** Find eigenvalues and eigenvectors of:
   ```
   A = [2  1]
       [0  3]
   ```
   Then verify: A * v = λ * v for each eigenpair.

2. **Stability analysis:** The linearized dynamics of a quadcopter's pitch axis can be approximated as:
   ```
   A = [ 0    1  ]
       [-Kp  -Kd ]
   ```
   where Kp and Kd are your PID gains.
   - Find eigenvalues as a function of Kp and Kd
   - For Kp = 10, what value of Kd makes the system critically damped? (eigenvalues real and equal)
   - For Kp = 10, Kd = 1, are the eigenvalues complex? What does that mean physically? (oscillation)
   - Plot eigenvalue locations in the complex plane for Kd = 1, 3, 5, 7, 10 with Kp = 10

3. **Python exercise:**
   ```python
   import numpy as np
   # Create the state transition matrix for your Kalman filter
   # F = [[1, dt], [0, 1]]  (constant velocity model)
   # Compute eigenvalues. They should both be 1 (marginally stable)
   # This means: without measurements, your estimate drifts forever
   # Now compute eigenvalues of (I - K*H)*F after Kalman update
   # They should be inside the unit circle (stable!)
   ```

4. **Drone Connection:** When you tune PID gains, you're placing eigenvalues. High Kp moves eigenvalues left (faster response) but can make them complex (oscillation). High Kd adds damping (pushes eigenvalues toward the real axis). This is the ENTIRE theory behind PID tuning — you're just placing poles.

**Strang exercises:** Ch 5: Problems 1, 5, 10. Ch 6: Problems 1, 3, 6, 9, 15, 22, 28.

---

### Week 4: SVD, Positive Definite Matrices, and Matrix Calculus

**Read:** Strang Ch 7 (Singular Value Decomposition) + review Ch 6

**Core Concepts:**
- SVD: A = UΣVᵀ (every matrix has one)
- Positive definite matrices (eigenvalues > 0)
- Covariance matrices are symmetric positive semi-definite
- Matrix derivatives (you'll need these for EKF Jacobians)
- Trace, matrix norms

**Problem Set:**

1. **Covariance matrices:** Generate 1000 random 2D points from a Gaussian with mean [0, 0] and covariance [[4, 2], [2, 3]].
   - Compute sample covariance matrix, verify it's close to the true one
   - Compute eigenvalues and eigenvectors of covariance matrix
   - Plot the data points + eigenvectors scaled by √eigenvalue — you get an ellipse
   - This ellipse IS the uncertainty ellipse you'll see in GPS/Kalman visualization

2. **Matrix calculus for Kalman:**
   - Prove: d/dx (xᵀAx) = (A + Aᵀ)x (and = 2Ax if A is symmetric)
   - This derivative appears when you derive the Kalman filter gain (minimizing trace of P)

3. **SVD intuition:** Compute SVD of a 3×2 matrix (your sensor measurement matrix H). The rank of H tells you how many states are observable from your measurements.

4. **Drone Connection:** The Kalman filter's P matrix (state covariance) is always symmetric positive semi-definite. If it becomes negative definite (due to numerical errors), your filter diverges. In practice, you enforce symmetry every update step: `P = (P + Pᵀ) / 2`. This is not a hack — it's standard practice.

---

## Part 2: Multivariable Calculus (Weeks 5–7)

**Primary Text:** *Calculus: Early Transcendentals* — Stewart, Chapters 12–16
**Supplementary:** Khan Academy Multivariable Calculus (free, for visual intuition)

### Week 5: Vectors in 3D, Partial Derivatives, Gradient

**Read:** Stewart Ch 12 (Vectors and Geometry) + Ch 14 (Partial Derivatives)

**Core Concepts:**
- Parametric curves in 3D (flight paths ARE parametric curves)
- Partial derivatives, gradient vector
- Chain rule for multivariable functions (critical for Jacobians)
- Directional derivatives

**Problem Set:**

1. A drone flies in a helical path: x(t) = R*cos(ωt), y(t) = R*sin(ωt), z(t) = v_z * t.
   - Compute velocity vector: v(t) = [dx/dt, dy/dt, dz/dt]
   - Compute acceleration vector: a(t) = [d²x/dt², d²y/dt², d²z/dt²]
   - Compute speed |v(t)|
   - What is the centripetal acceleration? (this is what tilts the drone in a turn)

2. **Jacobian matrices:** Given the nonlinear function:
   ```
   f(x, y) = [x² + y, sin(x)*y]
   ```
   Compute the Jacobian: J = [[∂f1/∂x, ∂f1/∂y], [∂f2/∂x, ∂f2/∂y]]
   Evaluate at (1, 0). This is EXACTLY what you do in the EKF — linearize nonlinear dynamics around current state.

3. **Drone Connection:** Your drone's dynamics are nonlinear (sin/cos of angles everywhere). The EKF works by computing the Jacobian of the dynamics at each timestep, creating a local linear approximation. Bad Jacobians = bad EKF = crash.

---

### Week 6: Chain Rule, Optimization, Lagrange Multipliers

**Read:** Stewart Ch 14.5–14.8 (Chain Rule, Optimization)

**Core Concepts:**
- Multivariable chain rule (needed for backpropagation in neural nets AND for EKF)
- Finding minima/maxima (gradient = 0)
- Lagrange multipliers (constrained optimization — MPC uses this)
- Hessian matrix (second derivative test in multiple dimensions)

**Problem Set:**

1. **Chain rule for state propagation:** If state x depends on time, and measurement z depends on state:
   ```
   x(t) = [position(t), velocity(t)]
   z(x) = sqrt(x[0]² + x[1]²)  (range measurement)
   ```
   Compute dz/dt using the chain rule. This is how measurement predictions change over time.

2. **Optimization:** Minimize f(Kp, Kd) = integral of (error²) dt for a simple second-order system. This is the "optimal" PID tuning problem. Set up in Python and use scipy.optimize.minimize.

3. **Lagrange multipliers:** Minimize control effort (u²) subject to the constraint that the drone reaches a target position. This is a baby version of what MPC does.

---

### Week 7: Multiple Integrals, Moment of Inertia

**Read:** Stewart Ch 15 (Multiple Integrals) + Taylor Ch 9–10 (Rigid Body)

**Core Concepts:**
- Double and triple integrals
- Moment of inertia tensor:
  ```
  I = ∫ ρ(r) * (|r|²I₃ - r*rᵀ) dV
  ```
- Parallel axis theorem
- Principal axes of inertia (eigenvectors of I!)

**Problem Set:**

1. **Compute moment of inertia for a quadcopter:**
   Model as 4 point masses (motors, mass m each) at positions (±d, ±d, 0) plus a central body (mass M, uniform disk radius r).
   - Compute full 3×3 inertia tensor
   - Find principal moments Ixx, Iyy, Izz
   - For an X-config: Ixx ≈ Iyy (symmetric). What does this mean for control? (roll and pitch have same dynamics)

2. **Why this matters:** Euler's equation for rotational dynamics:
   ```
   τ = I * α + ω × (I * ω)
   ```
   The cross-product term (gyroscopic effect) is why a quadcopter yaws when you change roll/pitch rapidly. If I is wrong, your controller model is wrong, and you get unexpected coupling.

---

## Part 3: Probability and Statistics for Sensor Fusion (Weeks 4–8, parallel)

**Primary Text:** *Kalman Filter from the Ground Up* — Alex Becker (practical, drone-focused)
**Secondary:** *Probability and Statistics for Engineering and the Sciences* — Devore (Ch 1–6 for fundamentals)

### Key Concepts (study as they appear in projects)

**Gaussian Distribution — Your Best Friend:**
- Everything in sensor fusion assumes Gaussian noise
- PDF: f(x) = (1/√(2πσ²)) * exp(-(x-μ)²/(2σ²))
- Sum of two Gaussians is Gaussian (this is why Kalman filter works!)
- Product of two Gaussians is Gaussian (this is the Kalman update step!)

**Problem Set:**

1. **Characterize your sensors:** Using data from Project 1 (stationary IMU):
   - Compute mean and variance of each axis
   - Plot histogram, overlay Gaussian fit — does it look Gaussian?
   - Compute Allan variance (stability measure for gyroscopes) — plot log-log, identify noise types
   - These numbers become your R matrix (measurement noise) in the Kalman filter

2. **Bayes' theorem — the core of all estimation:**
   ```
   P(state | measurement) = P(measurement | state) * P(state) / P(measurement)
   ```
   The Kalman filter IS Bayes' theorem for Gaussians. Prove:
   - Prior: N(μ₁, σ₁²) — your prediction
   - Likelihood: N(μ₂, σ₂²) — your measurement
   - Posterior: N(μ₃, σ₃²) where μ₃ = (μ₁σ₂² + μ₂σ₁²)/(σ₁² + σ₂²), σ₃² = (σ₁²σ₂²)/(σ₁² + σ₂²)
   - Notice: posterior variance is ALWAYS smaller than both prior and likelihood — fusion always improves!

3. **Covariance and correlation:**
   - Generate correlated noise (accelerometer X and Y are correlated when vibrating)
   - Compute covariance matrix from data
   - Understand: off-diagonal terms in Q and R matrices represent correlated noise

4. **Monte Carlo simulation:**
   ```python
   # Simulate 1000 Kalman filter runs with different random noise
   # Plot the spread of estimates — this IS the covariance matrix P
   # Verify: actual spread matches P from the filter
   # If they don't match, your Q and R are wrong
   ```

---

## Part 4: Quaternion Mathematics (Week 6–8)

> **You MUST learn quaternions. Euler angles have gimbal lock. Every serious flight controller uses quaternions internally.**

**Reference:** "Quaternion kinematics for the error-state Kalman filter" — Joan Solà (free PDF, the definitive reference)

### Core Concepts

**What is a quaternion?**
```
q = w + xi + yj + zk = [w, x, y, z]
where i² = j² = k² = ijk = -1
```

A unit quaternion (|q| = 1) represents a rotation. It's a 4D unit vector.

**Key Operations (implement ALL of these in both Python and C):**

1. **Quaternion multiplication (Hamilton product):**
   ```
   q1 * q2 = [w1*w2 - x1*x2 - y1*y2 - z1*z2,
              w1*x2 + x1*w2 + y1*z2 - z1*y2,
              w1*y2 - x1*z2 + y1*w2 + z1*x2,
              w1*z2 + x1*y2 - y1*x2 + z1*w2]
   ```
   This is how you compose rotations. Not commutative: q1*q2 ≠ q2*q1.

2. **Quaternion conjugate:** q* = [w, -x, -y, -z]. For unit quaternions: q* = q⁻¹.

3. **Rotate a vector by quaternion:**
   ```
   v_rotated = q * [0, v] * q*
   ```

4. **Quaternion to rotation matrix:**
   ```
   R = [[1-2(y²+z²),  2(xy-wz),    2(xz+wy)  ],
        [2(xy+wz),    1-2(x²+z²),  2(yz-wx)   ],
        [2(xz-wy),    2(yz+wx),    1-2(x²+y²) ]]
   ```

5. **Quaternion derivative (for integrating gyro data):**
   ```
   dq/dt = 0.5 * q * [0, ωx, ωy, ωz]
   ```
   where ω is angular velocity from gyroscope.

6. **Quaternion to Euler angles (for display only — never use Euler for computation):**
   ```
   roll  = atan2(2(wy + xz), 1 - 2(y² + z²))
   pitch = asin(2(wx - yz))
   yaw   = atan2(2(wz + xy), 1 - 2(x² + z²))
   ```

**Problem Set:**

1. **Gimbal lock demonstration:** Create a Python visualization:
   - Rotate using Euler angles: pitch from 0° to 90°
   - At pitch = 90°, try to yaw — observe that yaw and roll become the same axis
   - Now do the same rotation with quaternions — no gimbal lock
   - This is why every real flight controller (Betaflight, PX4, ArduPilot) uses quaternions

2. **Implement quaternion AHRS:**
   - Integrate gyro data using quaternion derivative: `q += 0.5 * q * ω * dt`
   - Normalize after each step: `q = q / |q|` (critical — numerical drift breaks unit constraint)
   - Compare against your Euler-angle complementary filter from Project 1

3. **Small angle approximation:**
   - For small rotations, q ≈ [1, θx/2, θy/2, θz/2]
   - This is used in the error-state Kalman filter (the standard in aerospace)
   - Verify: for θ = 5°, compare exact quaternion rotation vs. approximation — error should be tiny

---

## Appendix: Mathematical Notation Quick Reference

| Symbol | Meaning | Drone Context |
|---|---|---|
| x | State vector | [position, velocity, orientation, biases] |
| u | Control input | [throttle, roll_cmd, pitch_cmd, yaw_cmd] |
| z | Measurement | [GPS position, baro altitude, IMU data] |
| F (or A) | State transition matrix | How state evolves over dt |
| B | Control input matrix | How control affects state |
| H | Measurement matrix | What sensors measure of the state |
| P | State covariance | Uncertainty in state estimate |
| Q | Process noise covariance | How much model is wrong per step |
| R | Measurement noise covariance | How noisy sensors are |
| K | Kalman gain | How much to trust measurement vs. prediction |
| I | Moment of inertia tensor | Resistance to angular acceleration |
| τ | Torque vector | What motors produce to rotate drone |
| ω | Angular velocity | Roll rate, pitch rate, yaw rate |
| q | Quaternion | Orientation without gimbal lock |
| R | Rotation matrix | Convert between coordinate frames |

---

> *"The math is not decoration. It is the architecture."*
