# 📄 architecture.md

Aquavision 2.x System Architecture
Target: Jetson Orin Nano + ROS2 Humble + Pure Pymavlink

---

# 1. Design Philosophy

Aquavision follows these principles:

1. Minimal ROS nodes
2. Deterministic control loop
3. Internal modularity (not inter-node modularity)
4. Low IPC overhead
5. Reusable mission behaviors
6. GPU-accelerated perception
7. Stable MAVLink control

We prefer:

Modular internal state classes
Over
Multiple communicating ROS nodes

Because interprocess communication introduces latency, synchronization complexity, and debugging overhead.

---

# 2. Current Node Architecture (Stable Core)

We intentionally keep only 3 ROS nodes:

| Node              | Responsibility                        |
| ----------------- | ------------------------------------- |
| `vision_node`     | GPU-accelerated YOLO gate detection   |
| `mission_node`    | Deterministic mission + state machine |
| `nav_bridge_node` | Pymavlink control + telemetry         |

No additional nodes unless absolutely required.

---

# 3. Data Flow

```
Camera
  ↓
vision_node
  ↓  (gate_data topic)
mission_node
  ↓  (control_cmd topic)
nav_bridge_node
  ↓
Pixhawk (ArduSub)
```

One-directional deterministic flow.

No circular dependencies.

---

# 4. Internal Modularity Strategy (IMPORTANT)

We do NOT modularize at ROS level.

We modularize inside mission_node using:

```
mission/
    mission_manager.py
    states/
        base_state.py
        descend_state.py
        stabilize_state.py
        search_state.py
        align_state.py
        thrust_state.py
        blind_pass_state.py
    behaviors/
        vertical_control.py
        forward_motion.py
        yaw_control.py
```

States are pure Python classes.

Behaviors are reusable control primitives.

---

# 5. Mission Node Internal Architecture

`mission_node` contains:

### 1️⃣ MissionManager (ROS interface)

* Subscribes to perception
* Owns 30 Hz timer
* Publishes control commands
* Manages state transitions

### 2️⃣ State Classes (Pure Logic)

Each state:

* Has no ROS imports
* Returns control dict
* Signals transitions

### 3️⃣ Behavior Modules (Reusable Primitives)

Reusable motion components:

* descend(duration, thrust)
* ascend(duration, thrust)
* hold_depth()
* yaw_rotate(speed)
* forward_move(speed)
* blind_forward(duration)

These are functions or small classes.

---

# 6. Why This Architecture Is Optimal

## ✅ Low Latency

* Only 3 nodes
* Minimal topic hops
* No extra IPC

## ✅ Modular

* Reuse descend in slalom, qualification, torpedo
* Reuse thrust logic anywhere

## ✅ Deterministic

* Single control timer
* No race conditions

## ✅ Easy to Extend

To add new task:

* Add new state sequence
* Reuse behaviors
* No node redesign

---

# 7. Control Frequency

| Component        | Frequency |
| ---------------- | --------- |
| Vision inference | 15–20 Hz  |
| Mission update   | 30 Hz     |
| MAVLink send     | 20–30 Hz  |

Mission and control loop must be fixed-rate.

No variable timing.

---

# 8. What We Explicitly Avoid

❌ Separate ROS node per state
❌ Behavior nodes
❌ Multiple publishers per mission
❌ Asynchronous mission execution
❌ Complex action servers
❌ Overuse of ROS services

This is a competition AUV, not a distributed cloud system.

---

# 9. Reusability Strategy

Reusable modules should include:

### Motion primitives:

* Descend
* Ascend
* Forward thrust
* Yaw rotate
* Depth hold

### Control utilities:

* PID class
* Low-pass filter
* Deadband handler
* Saturation clamp

### Perception utilities:

* Detection filtering
* Confidence thresholding
* Frame normalization

These should not depend on ROS.

---

# 10. Latency Control Policy

We must:

* Avoid adding new ROS nodes unless necessary
* Avoid heavy message types
* Avoid publishing images unnecessarily
* Avoid chained topic relays

All mission decisions happen inside one node.

---

# 11. Future Extension Strategy

To add new tasks:

Example: Slalom

```
slalom_mission.py
```

Uses:

* descend_state
* search_state
* align_state
* thrust_state
* blind_pass_state

No rewrite required.

---

# 12. Fault Tolerance Strategy

If:

* vision lost
* mavlink lost
* state exception

Mission node must:

* Command neutral thrust
* Log error
* Remain alive

Never crash.

---

# 13. GPU Usage Policy

Vision node may:

* Use CUDA
* Use FP16
* Use TensorRT

Mission node must remain CPU-light.

Do not mix GPU code into mission node.

---

# 14. Final Architectural Rule

We optimize for:

Simplicity > Abstraction
Determinism > Cleverness
Stability > Elegance

