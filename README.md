# IMU Attitude Estimator

Real-time 3D orientation estimation from raw IMU data using sensor fusion: complementary filter → Kalman filter → quaternions.

> **Status:** 🚧 In progress. Complementary filter implemented in MATLAB (Fall 2026); Kalman filter and dedicated hardware next.

---

## Overview

An inertial measurement unit (IMU) contains a gyroscope and an accelerometer, and sometimes a magnetometer. None of them gives a good attitude estimate alone:

| Sensor | Strength | Weakness |
|---|---|---|
| Accelerometer | No drift (always references gravity) | Noisy; corrupted by motion |
| Gyroscope | Smooth, fast response | Drifts as integration error builds up |
| Magnetometer | Gives heading (yaw) | Sensitive to nearby metal and electronics |

This project fuses them to get an estimate that is both smooth and drift-free. It is the same problem a spacecraft's attitude determination system (ADCS) solves, and it is the foundation for my later projects: a reaction wheel testbed and an ISS pointer.

---

## Method

### 1. Accelerometer tilt
Roll and pitch from the gravity vector:

$$\phi_{acc} = \operatorname{atan2}(a_y,\ a_z), \qquad \theta_{acc} = \operatorname{atan2}\!\left(-a_x,\ \sqrt{a_y^2 + a_z^2}\right)$$

### 2. Gyro integration

$$\phi_{gyro,k} = \phi_{k-1} + \omega_x \,\Delta t$$

### 3. Complementary filter
High-pass the gyro, low-pass the accelerometer:

$$\phi_k = \alpha\left(\phi_{k-1} + \omega_x \Delta t\right) + (1-\alpha)\,\phi_{acc,k}$$

with $\alpha \approx 0.98$.

### 4. Kalman filter *(next)*
Treat attitude and gyro bias as states, propagate with the gyro, and correct with the accelerometer using the optimal gain.

### 5. Quaternions *(planned)*
Full 3D attitude without gimbal lock, plus magnetometer fusion for yaw.

---

## Results

<!-- TODO: replace with numbers from your own run of the script -->

| Estimate | RMS roll error |
|---|---|
| Accelerometer only | TBD |
| Gyro only | TBD |
| Complementary filter | TBD |

<!-- TODO: add a plot, e.g. ![Roll comparison](docs/roll_comparison.png) -->

---

## Hardware

**Phase 1 (current):** smartphone IMU, logged with MATLAB Mobile

**Phase 2 (planned):**
- Teensy 4.1 microcontroller
- Adafruit ISM330DHCX + LIS3MDL 9-DoF IMU
- Reference sensor (BNO085 or VectorNav VN-100) for ground-truth comparison
- microSD logging

---

## Repository structure

```
├── complementary_filter.m   # Main script
├── data/          # Recorded IMU logs (.mat / .csv)
├── firmware/      # Teensy code (Phase 2)
├── docs/          # Plots and write-ups
└── README.md
```

## Running it

```matlab

complementary_filter   % uses simulated data, or load a log from data/
```

---

## Roadmap

- [x] Read raw accelerometer and gyro data
- [x] Accelerometer tilt and gyro integration
- [x] Complementary filter
- [ ] Record and process real phone data
- [ ] Kalman filter with gyro bias estimation
- [ ] Quaternion attitude and 3D visualization
- [ ] Port to Teensy 4.1 and validate against reference sensor

---

**Author:** Jesus Ordaz, UC Irvine
