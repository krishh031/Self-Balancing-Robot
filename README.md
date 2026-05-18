# Self-Balancing Robot

A two-wheeled self-balancing robot built with Arduino and stepper motors,
using real-time PID control to maintain upright balance.

## How it works
The MPU6050 sensor reads the tilt angle (Y-axis) via I²C.
A PID controller calculates the correction needed every loop cycle
and drives two stepper motors forward or backward to counteract the tilt.
The robot targets 0° — any deviation triggers a motor response.

## Components
- Arduino Nano
- MPU6050 gyroscope + accelerometer (I²C)
- 2x Stepper motors
- Stepper motor driver (A4988)
- 12V battery

## PID Values
| Parameter | Value |
|-----------|-------|
| Kp | 18.0 |
| Ki | 0.6  |
| Kd | 1.2  |

## Features
- Anti-windup on integral term (clamped ±10)
- Gyroscope offset auto-calibration on startup
- Real-time Serial monitor debug output (angle + PID output)
- Speed mapped from PID output to stepper delay (350–2000 µs)

## Libraries used
- Wire.h (built-in)
- MPU6050_tockn


