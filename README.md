# ☕ Hold My Coffee – Drinks Stabilization Platform

**Authors:** Valentin Pletea-Marinescu & Sebastian-Alexandru Matei  
Faculty of Automatic Control and Computer Science / Engineering in Foreign Languages

---

## 🧩 Overview
Hold My Coffee is a **3-DOF active stabilization platform** that keeps a drink level while moving.  
It combines a **Teensy 4.1 main controller** with an **Arduino UNO secondary controller** to manage pitch, roll, and height stabilization using real-time inertial feedback and brushless motor control.

---

## 🚀 Features
- 3 Degrees of Freedom: Pitch, Roll, Vertical height  
- Dual-microcontroller real-time architecture  
- Moteus r4.11 motor controllers with FOC  
- BNO055 IMU + VL53L8CX ToF sensors  
- Cascaded PID control (20 Hz)  
- Modular 3D-printed + aluminum structure  

---

## ⚙️ Hardware
| Component | Description |
|------------|-------------|
| **MCU #1** | Teensy 4.1 – ARM Cortex-M7 @ 600 MHz |
| **MCU #2** | Arduino UNO – Height & servo subsystem |
| **Motors** | mj5208 brushless motors (x3) |
| **Controllers** | Moteus r4.11 (x3) |
| **Sensors** | Adafruit BNO055, VL53L8CX |
| **Power** | 14.8 V LiPo 6000 mAh, 300 A switch + fuses |

---

## 🧠 Control Logic
Cascaded PID control loop with quaternion-to-Euler conversion:
