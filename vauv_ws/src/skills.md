# 📄 skills.md

Aquavision 2.x Engineering Contract
Target Platform: Jetson Orin Nano (JetPack 6.1)

---

# 1. System Environment Contract

All generated code MUST assume:

## Hardware

* NVIDIA Jetson Orin Nano
* CUDA-enabled GPU
* 8GB or 16GB RAM
* USB-connected Pixhawk
* CSI or USB camera

## OS / Middleware
SS
* Ubuntu 22.04 (JetPack 6.1)
* ROS 2 Humble
* Python 3.10
* Colcon build system

## GPU

* CUDA available
* cuDNN available
* TensorRT available
* OpenCV compiled with CUDA support

Code is allowed to use:

* torch with CUDA
* TensorRT
* OpenCV CUDA modules
* PyCUDA (if needed)

But must degrade gracefully if GPU temporarily unavailable.

---

# 2. Architecture Rules (MANDATORY)

## 2.1 Clear Separation of Concerns

We enforce strict layering:

Perception → Mission Logic → Control → MAVLink

No cross-layer logic.

---

## 2.2 State Machine Design

* Only ONE mission_manager ROS node
* States implemented as pure Python classes
* No ROS inside states
* Deterministic fixed-rate loop (20–50 Hz)

State interface:

```python
on_enter()
update(perception_data, dt)
on_exit()
```

---

## 2.3 No Blocking Anywhere

Inside ROS:

* No while True loops
* No sleep()
* No blocking serial calls
* No blocking GPU calls inside callbacks

All heavy processing must:

* Use timers
* Use buffered frames

---

# 3. GPU & Performance Rules

## 3.1 YOLO Inference

Preferred order:

1. TensorRT engine (best performance)
2. Torch CUDA
3. Torch CPU (fallback only)

Do NOT reload model per frame.
Load once in constructor.

Inference must run:

* In timer callback
* Not inside image subscription

---

## 3.2 Frame Handling

* Use latest-frame buffer strategy
* Drop old frames if inference lags
* Never queue unlimited frames

---

## 3.3 Target Performance

* Vision ≥ 15 FPS
* Control loop ≥ 30 Hz
* MAVLink send ≥ 20 Hz
* End-to-end latency < 100ms

---

# 4. Pymavlink Rules (STRICT)

## 4.1 No MAVROS

Do not import MAVROS.
Do not depend on mavros_msgs.

## 4.2 Connection Logic

* Use blocking=False
* No blocking wait_heartbeat()
* Use timer-driven reconnect

## 4.3 Safety

* Clamp manual_control_send values
* Neutral thrust on MAVLink loss
* Neutral thrust on exception

---

# 5. Parameterization Rules

All tuning parameters must be ROS parameters:

* PID gains
* thrust values
* durations
* thresholds
* serial port
* baud rate
* inference confidence threshold
* model path

No hardcoded magic numbers.

---

# 6. Logging & Debugging

Use:

* Python logging module
* ROS logger
* CSV mission logs

Do not use print().

Logs must not crash system.

---

# 7. Jetson-Specific Optimization Guidelines

Allowed optimizations:

* torch.backends.cudnn.benchmark = True
* half precision inference (FP16)
* TensorRT engine generation
* CUDA memory preallocation

Not allowed:

* Memory leaks
* Unbounded GPU tensor growth
* Repeated model instantiation

---

# 8. Threading Policy

Avoid threads unless absolutely required.

If used:

* Must be daemon threads
* Must not block shutdown
* Must not share mutable globals without locks

Prefer single-thread deterministic design.

---

# 9. Safety Requirements

If any of the following occur:

* No perception data for > 2 seconds
* MAVLink disconnect
* State machine exception

System must:

* Send neutral thrust
* Log error
* Continue running (no crash)

---

# 10. Code Quality Requirements

All generated code must:

* Use type hints
* Use docstrings
* Follow PEP8
* Avoid duplication
* Avoid deep nesting
* Avoid over-engineering

Must build cleanly with:

```
colcon build
```

---

# 11. Mission Design Philosophy

We prioritize:

* Determinism
* Debuggability
* Stability over cleverness
* Competition reliability over abstraction

Do not introduce unnecessary frameworks.

---

# 12. What NOT To Do

Do NOT:

* Introduce asyncio
* Introduce third-party state machine libraries
* Introduce complex dependency injection frameworks
* Mix mission logic with ROS communication
* Add unnecessary abstraction layers

---

# 13. Generation Rules for AI

When generating code:

* Assume Jetson Orin Nano
* Use GPU acceleration if helpful
* Ensure code degrades safely
* Never assume hardware always connected
* Never redesign architecture unless requested

