# VAUV Control System

This repository contains the ROS 2 Humble source code for the **VAUV (VIT Autonomous Underwater Vehicle)**. It is designed to interface with a Pixhawk flight controller via MAVROS to perform automated diving and forward-motion missions.

## 📂 Repository Structure

The `src` directory is organized as a standard ROS 2 workspace source folder:

* **`vauv_control/`**: The primary ROS 2 Python package.
    * **`launch/vauv_launch.py`**: Orchestrates the startup of MAVROS, the mission controller, and the data logger.
    * **`vauv_control/mission_node.py`**: Handles the logic for arming, diving, holding depth, and moving forward.
    * **`vauv_control/logger_node.py`**: Subscribes to telemetry topics and logs data to a time-stamped CSV file.

---
## How to download and use?
1. ```bash
   mkdir -p vauv_ws/src
   cd vauv_ws/src
   ```
2.once inside src :
  ```bash
  git clone https://github.com/VAUV/jetson_test.git
  ```

## 🛠️ Build Instructions

Ensure you are inside your ROS 2 Humble environment (or Docker container) before proceeding.

1.  **Navigate to your workspace root** (one level above `src`):
    ```bash
    cd ~/vauv_ws
    ```

2.  **Build the package**:
    ```bash
    colcon build --packages-select vauv_control
    ```

3.  **Source the setup file**:
    ```bash
    source install/setup.bash
    ```

---

## 🚀 Running the Mission

### 1. Hardware Connection
Ensure the Pixhawk is connected via USB. By default, the system looks for the device at `/dev/ttyACM0`. If your port differs, update the `fcu_url` in the launch file.

### 2. Launching the Nodes
Run the following command to start the entire stack:
```bash
ros2 launch vauv_control vauv_launch.py
