# 🦅 Operation Shaheen

**From Student to Chief Engineer — A 36-Month Roadmap to Autonomous UAV Systems**

> *"In defense, vaporware is worse than nothing."*

---

## What Is This?

This repository documents my journey building autonomous UAV systems from scratch. Every project has real hardware, real code, real flight data, and real crash reports.

**Phase 1 (Months 1–6):** Foundations — sensor fusion, control theory, embedded systems, first flight  
**Phase 2 (Months 7–18):** Systems Architecture — custom PCBs, RTOS, power systems  
**Phase 3 (Months 19–36):** Autonomy — GNC, computer vision, swarm intelligence  

Currently executing: **Phase 1**

---

## Repository Structure

```
operation-shaheen/
├── CLAUDE.md                          ← Phase 1 detailed execution plan (Claude Code reference)
├── README.md                          ← You are here
│
├── 01-sensor-whisperer/               ← IMU driver + complementary filter (Weeks 1-3)
├── 02-control-theorist/               ← PID controller + reaction wheel (Weeks 4-8)
├── 03-embedded-kalman/                ← Kalman filter for altitude (Weeks 9-12)
├── 04-flight-controller-zero/         ← Custom quadcopter FC (Weeks 13-24)
├── 05-mahogany-bomber/                ← Fixed-wing autopilot (Weeks 20-26)
│
├── theory/                            ← Study curriculum (math, control, source code guides)
│   ├── 01_mathematics.md
│   ├── 02_control_theory.md
│   ├── 03_ardupilot_betaflight_guide.md
│   └── 04_python_drone_programming.md
│
├── tools/                             ← Shared utilities (serial plotter, log parser)
├── hardware/                          ← BOMs, wiring diagrams, datasheets
└── docs/                              ← Research papers, notes, references
```

---

## Current Progress

| Project | Status | Key Metric |
|---------|--------|------------|
| 01 — Sensor Whisperer | 🔲 Not started | Orientation ±2° |
| 02 — Control Theorist | 🔲 Not started | Settle <2s |
| 03 — Embedded Kalman | 🔲 Not started | Drift <5cm/60s |
| 04 — Flight Controller Zero | 🔲 Not started | Hover <5° drift |
| 05 — Mahogany Bomber | 🔲 Not started | 3-5 waypoints autonomous |

---

## Tech Stack

**Hardware:** STM32F411 (Black Pill), MPU-6050/BMI088, BMP280/MS5611, u-blox GPS  
**Firmware:** C (bare metal → FreeRTOS), STM32 HAL  
**Simulation & Analysis:** Python (NumPy, SciPy, matplotlib, control)  
**Ground Station:** Python (pyserial, pymavlink, DroneKit)  
**Tools:** STM32CubeIDE, KiCad, Git  

---

## How to Use This Repo

**If you're following my journey:** Check the README in each project folder for build logs and results.

**If you're building your own:** Clone the repo, read `CLAUDE.md` for the full execution plan, and start with Project 1. The `theory/` folder has study guides for each subject area.

**If you're using Claude Code:** The `CLAUDE.md` file at root is designed as a Claude Code project file. Reference specific projects and tasks for context-aware assistance.

---

## License

MIT — Build something. Share it. Make it fly.

---

> *Operation Shaheen is a go. Execute.*
