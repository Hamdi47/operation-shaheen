# Project 1: Sensor Whisperer

> Read raw IMU data, implement complementary filter, output real-time orientation.

## Status: 🔲 Not Started

## Goal
Real-time orientation estimate from MPU-6050/BMI088, accurate to ±2°.

## Hardware
- STM32F411 Black Pill
- MPU-6050 (or BMI088) IMU
- USB-UART adapter

## Tasks
- [ ] Task 1.1 — STM32 Bring-Up (LED blink with timer)
- [ ] Task 1.2 — I2C Communication with IMU
- [ ] Task 1.3 — UART Streaming + Python Plotter
- [ ] Task 1.4 — Complementary Filter
- [ ] Task 1.5 — Sampling Rate Optimization

## Deliverables
- [ ] Orientation estimate ±2°
- [ ] 1kHz sampling verified
- [ ] Python real-time plotter
- [ ] Noise characterization data
- [ ] Video demo

## Wiring
```
STM32 PB6  → MPU-6050 SCL (+ 4.7kΩ pullup to 3.3V)
STM32 PB7  → MPU-6050 SDA (+ 4.7kΩ pullup to 3.3V)
STM32 3.3V → MPU-6050 VCC
STM32 GND  → MPU-6050 GND
STM32 PA2  → USB-UART RX (for UART TX output)
```

## Build Log
*Document each session here with date, what you did, what worked, what didn't.*
