# Git Setup Instructions — Operation Shaheen

## Step 1: Create GitHub Repository

Go to [github.com/new](https://github.com/new) and create a new repository:
- **Name:** `operation-shaheen`
- **Description:** `From Student to Chief Engineer — Autonomous UAV Systems`
- **Visibility:** Public (build in public!)
- **Do NOT** initialize with README (we already have one)

## Step 2: Extract the Repo

Download the `operation-shaheen.tar.gz` file from this chat and extract it:

```bash
# Extract the archive
tar -xzf operation-shaheen.tar.gz
cd operation-shaheen
```

## Step 3: Configure Git

```bash
# Set your identity (use your real name — this goes on your portfolio)
git config user.name "Your Name"
git config user.email "your.email@example.com"
```

## Step 4: Push to GitHub

```bash
# Add GitHub remote (replace YOUR_USERNAME)
git remote add origin https://github.com/YOUR_USERNAME/operation-shaheen.git

# Push
git push -u origin main
```

## Step 5: Verify

Visit `https://github.com/YOUR_USERNAME/operation-shaheen` — you should see the full repo with README rendered.

---

## Branching Strategy

Use branches for each project:

```bash
# When starting a new project:
git checkout -b project/01-sensor-whisperer

# Work on the project, commit often:
git add .
git commit -m "P1: Task 1.2 — I2C communication with MPU-6050 working"

# When project milestone is complete, merge to main:
git checkout main
git merge project/01-sensor-whisperer
git push origin main

# Tag milestones:
git tag -a v0.1 -m "Project 1 complete: Sensor Whisperer"
git push origin v0.1
```

## Commit Message Convention

Format: `P{N}: {Task} — {What you did}`

Examples:
```
P1: Task 1.1 — STM32 blink with TIM2 interrupt working
P1: Task 1.4 — Complementary filter achieving ±2° accuracy
P2: Task 2.3 — PID controller with anti-windup implemented
P4: Sub-project 4H — FIRST HOVER! 15 seconds stable
P4: CRASH REPORT — PID oscillation at high throttle, root cause: D-gain too high
```

## Git LFS (for large files)

If you store videos or large data files:

```bash
# Install Git LFS
git lfs install

# Track large file types
git lfs track "*.mp4"
git lfs track "*.bin"
git lfs track "*.csv"

# Commit the .gitattributes
git add .gitattributes
git commit -m "Configure Git LFS for large files"
```

---

## Using with Claude Code

Place `CLAUDE.md` at your repo root. When using Claude Code:

```bash
# Claude Code will automatically read CLAUDE.md for context
# Reference specific tasks:
claude "Help me with Project 1, Task 1.2 — I2C read from MPU-6050"
claude "Debug my PID controller in 02-control-theorist/firmware/Core/Src/pid.c"
claude "Review my Kalman filter implementation for correctness"
```
