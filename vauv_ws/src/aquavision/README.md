# Aquavision 2.0

ROS 2 Humble autonomous gate traversal system for AUV.

## Environment Setup

> **IMPORTANT**: ROS 2 Humble ships with system OpenCV compiled against NumPy 1.x.
> You **must** use `numpy<2.0` to avoid import errors.

```bash
# Fix NumPy version
pip uninstall numpy -y
pip install "numpy<2"

# Verify
python3 -c "import numpy; print(numpy.__version__); import cv2; print(cv2.__version__)"
```

## Build & Run

```bash
cd ~/vauv_ws  # adjust to your workspace path
colcon build --symlink-install
source install/setup.bash
ros2 launch aquavision aquavision_launch.py
```

## Nodes

| Node | Description |
|------|-------------|
| `vision_node` | YOLOv11 gate detection from camera images |
| `nav_bridge_node` | Pymavlink bridge to Pixhawk (ArduSub) |
| `mission_node` | Deterministic state machine for gate traversal |

## Notes

- All nodes are non-blocking (ROS timers only, no `time.sleep`)
- `nav_bridge_node` will retry MAVLink connection every 2s if Pixhawk is disconnected
- Mission telemetry is logged to `~/aquavision_logs/`
- Do **not** install OpenCV via pip — use the system package
