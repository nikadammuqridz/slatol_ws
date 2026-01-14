# SLATOL: Single-Legged Autonomous Take-Off and Landing

## 📋 Project Overview
This repository utilizes a **Finite State Machine (FSM)** architecture to control a single-legged hopping robot. The core objective is to demonstrate that **Hybrid Event-Based Control (AMC)** is superior to **Reactive Feedback Control (PID)** for high-dynamic maneuvers where sensor latency leads to instability.

**System Requirements:**
* **OS:** Ubuntu 22.04 LTS (Jammy Jellyfish)
* **IDE:** Visual Studio Code (VS Code)
* **Framework:** ROS 2 Humble Hawksbill
* **Simulator:** Gazebo Ignition (Fortress)

---

## ⚙️ VS Code & Workspace Setup (Ubuntu)

### 1. Prerequisites
Ensure you have the ROS 2 development tools installed:
    sudo apt update
    sudo apt install ros-humble-desktop python3-colcon-common-extensions git -y

### 2. Build the Project
Open your terminal (Ctrl+Alt+T) inside this folder:

    colcon build --symlink-install
    source install/setup.bash

### 3. Open in VS Code
code .

Tip: Install the "ROS" and "Python" extensions in VS Code for better syntax highlighting.

## Code Structure & Logic
📂 src/slatol_bringup/slatol_planner.py
Role: The "Brain" of the robot.

Logic: Implements a Finite State Machine (FSM) with phases: STANCE → CROUCH → JUMP → FLIGHT.

Key Comparison Logic:

Mode A (PID): Calculates torque based on (Target_Angle - Current_Angle). Failure Mode: Reacts after error occurs.

Mode B (Hybrid): Calculates torque based on (Pitch_Rate * Inertia_Ratio). Success Mode: Predicts and cancels reaction torque.

Data Logging: Automatically records Time vs Pitch to CSV files for analysis.

📂 src/slatol_description/urdf/slatol.urdf.xacro
Role: The "Body".

Logic: Defines physical properties (Mass = 5kg body) and sensors (IMU @ 100Hz).

🚀 How to Reproduce Results
Experiment 1: The Baseline Failure (PID)
Launch Simulation: ros2 launch slatol_bringup slatol.launch.py

Trigger Jump (New Terminal): ros2 topic pub /slatol/mode std_msgs/msg/String "data: 'PID'" --once

Observation: The robot jumps but tumbles backward due to reaction torque.

Data: Saved to PID_result.csv.

Experiment 2: The Hybrid Success (AMC)
Reset Simulation: Press Ctrl+R in Gazebo.

Trigger Jump: ros2 topic pub /slatol/mode std_msgs/msg/String "data: 'HYBRID'" --once

Observation: The robot jumps and remains upright by counter-rotating its leg.

Data: Saved to HYBRID_result.csv.

Generate Comparison Plot
python3 plot_results.py (Generates Figure_4_1_Comparison.png)