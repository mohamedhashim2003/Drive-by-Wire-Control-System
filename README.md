# Drive-by-Wire (DbW) Electronic Throttle Control System

A high-fidelity mechatronics simulation of an automotive electronic throttle system. This project replaces traditional mechanical linkages with a digital, closed-loop PID control architecture.

## 🚀 Overview
This system utilizes an **Arduino Uno** as the Electronic Control Unit (ECU) to process driver intent and actuate a 3-axis Cartesian or 3D-printed throttle valve. It features a custom **PyQt5 Dashboard** for real-time data acquisition (DAQ), allowing for live tuning of control parameters and performance mapping.

## 🛠️ Technical Stack
*   **Microcontroller:** Arduino Uno (C++)
*   **Actuation:** L298N H-Bridge + 12V DC Motor
*   **Sensors:** Dual 10kΩ Potentiometers (Pedal Input & TPS Feedback)
*   **Dashboard:** Python 3 + PyQt5 + PyQtGraph
*   **Control Theory:** Custom PID Loop with Anti-Windup and Deadband compensations

## 📊 Key Features

### 1. Closed-Loop PID Control
The system implements a manual PID algorithm that continuously calculates the error between the **Pedal Position** and **Throttle Position Sensor (TPS)** feedback.
*   **Deadband:** Prevents mechanical "hunting" and buzzing near zero-error states.
*   **Integral Limit:** Prevents integral windup during mechanical saturation.

### 2. Dynamic Drive Modes
The Arduino calculates target angles based on selectable mathematical profiles sent from the GUI:
*   **ECO:** Logarithmic mapping for dampened, fuel-efficient response.
*   **NORMAL:** Linear 1:1 pedal-to-throttle mapping.
*   **SPORT:** Square-root mapping for aggressive, front-loaded throttle response.

### 3. Real-Time DAQ Interface
The PyQt5-based control panel provides a high-speed serial interface to visualize the system's step response.
*   **Live Plotting:** Dual-trace graph showing Target vs. Actual angle.
*   **Status Gauges:** Circular arc gauges for immediate visual feedback of sensor data.
*   **Mode Switching:** Direct serial command transmission to the ECU.

## 🔧 Installation & Setup
1.  **Hardware:** Wire the components according to the pin map in `throttle_control.ino`.
2.  **Arduino:** Upload `throttle_control.ino` to your Uno.
3.  **Python:** 
    ```bash
    pip install pyqt5 pyserial pyqtgraph
    ```
4.  **Run:** Execute `python throttle_gui.py`, select your COM port, and click **Connect**.

## 👥 Contributors
*   **Mohamed Khaled Hashim** – Systems Architecture & Control Logic
*   **Ahmed Essam** – PID Tuning & Response Analysis
*   **Abdullah Elsayed Kandeel** – Mechanical Design (3D Printing)
*   **Ibrahim Hamza** – Simulation & Testing
