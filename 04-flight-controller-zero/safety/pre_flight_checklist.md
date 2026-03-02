# Pre-Flight Checklist — Flight Controller Zero

**Date:** ___________  
**Location:** ___________  
**Pilot:** ___________  
**Weather:** Wind: ___ m/s | Temp: ___°C | Conditions: ___________

---

## 1. Hardware Inspection
- [ ] Frame integrity — no cracks, all bolts tight
- [ ] Props secure — no play, correct rotation direction (CW/CCW alternating)
- [ ] Props undamaged — no nicks, chips, or cracks
- [ ] Motors spin freely — no grinding or obstruction
- [ ] Wiring secure — no loose connections, no exposed conductors
- [ ] Battery connector — firm, no play
- [ ] FC mounting — vibration dampening intact, no contact with frame

## 2. Battery
- [ ] Battery voltage: ___V (must be >11.1V for 3S, >14.8V for 4S)
- [ ] Battery physical condition — no puffing, no damage
- [ ] Battery secured — velcro/strap tight, cannot shift in flight

## 3. Electronics
- [ ] Power on — FC boots normally, no error LEDs
- [ ] IMU calibration — gyro bias computed at startup (keep still for 5 seconds)
- [ ] Sensor check — accel reads ~1g on Z, gyro reads ~0 when stationary
- [ ] UART telemetry — data streaming to ground station

## 4. RC System
- [ ] TX powered on — correct model selected
- [ ] RX bound — signal LED solid
- [ ] Channel test — move all sticks, verify correct channel mapping:
  - [ ] Throttle (Ch1): up = motors faster
  - [ ] Roll (Ch2): right = right roll
  - [ ] Pitch (Ch3): forward = pitch forward
  - [ ] Yaw (Ch4): right = yaw right
- [ ] Failsafe test: **Turn off TX** → verify motors cut within 1 second
- [ ] Range test: walk 50m away, verify control response

## 5. Software
- [ ] Correct firmware version flashed
- [ ] PID gains set to tested values (document: Kp=___, Ki=___, Kd=___)
- [ ] Angle limits active (max ±45°)
- [ ] Motor idle speed set
- [ ] Arm/disarm verified (throttle low + yaw right/left)

## 6. Environment
- [ ] Wind < 5 m/s
- [ ] Open area — minimum 30m clearance in all directions
- [ ] No people in flight zone
- [ ] No overhead obstructions (power lines, trees)
- [ ] Distance from airport: > 5km

## 7. Pre-Arm
- [ ] Ground station receiving telemetry
- [ ] Data logging started (SD card or UART)
- [ ] Spotter briefed (if applicable)
- [ ] Kill switch assigned and tested (separate TX switch or second person)
- [ ] Emergency plan discussed: if quad flies away → cut throttle immediately

## 8. Arm & Go
- [ ] Stand clear — minimum 3m from quadcopter
- [ ] Arm command sent — motors spin at idle
- [ ] Verify: motors respond to stick input at idle (slight pitch = slight motor change)
- [ ] Increase throttle slowly...

---

## Post-Flight
- [ ] Disarm immediately after landing
- [ ] Battery voltage: ___V (never discharge below 10.5V for 3S)
- [ ] Inspect for damage
- [ ] Download logs
- [ ] Notes: _____________________________________________

---

**ABORT CRITERIA — cut throttle immediately if:**
- Any vibration or unusual sound
- Uncommanded movement
- Loss of control response
- Battery warning
- Any person enters flight zone
