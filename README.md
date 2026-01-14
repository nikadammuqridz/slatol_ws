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
```bash
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions git -y