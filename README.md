# ISL Bimanual RX150

ROS2 + Isaac Sim demo scripts for single and dual Interbotix ReactorX 150 (RX150) robot arms.

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble + Interbotix ROS2 packages installed
- Isaac Sim 4.5 (pip install into a venv, e.g. `~/isaaclab`) — optional, for simulation

### ROS2 + Interbotix Install

Follow the [Interbotix ROS2 installation guide](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/software_setup.html) or use the setup script from the [ISL robotics machine setup repo](https://github.com/profshrestha).

Make sure the environment is sourced:
```bash
source /opt/ros/humble/setup.bash
source /opt/interbotix_ws/install/setup.bash
```
Or if set up system-wide:
```bash
source /etc/profile.d/ros2_interbotix.sh
```

### Isaac Sim Install

```bash
python3.10 -m venv ~/isaaclab
source ~/isaaclab/bin/activate
pip install --upgrade pip
pip install 'isaacsim[all,extscache]==4.5.0' --extra-index-url https://pypi.nvidia.com
```

---

## Repository Structure

```
working_scripts/
  rx150_single_sequence.py   # Single arm sequence (self-contained)
  rx150_dual_sequence.py     # Dual arm sequence (self-contained)
motor_configs/
  arm_right.yaml             # Motor config for right arm (set port here)
  arm_left.yaml              # Motor config for left arm (set port here)
isaac_ros/
  isaac_single_rx150_setup.py  # Isaac Sim setup + ROS2 bridge (single arm)
  isaac_dual_rx150_setup.py    # Isaac Sim setup + ROS2 bridge (dual arm)
rx150_usd/
  rx150.usd                  # RX150 arm USD model for Isaac Sim
```

---

## Single Arm

### Simulation only
```bash
python3 working_scripts/rx150_single_sequence.py
```

### Real hardware
```bash
python3 working_scripts/rx150_single_sequence.py --real
```

### With Isaac Sim

**Terminal 1 — Isaac Sim:**
```bash
source ~/isaaclab/bin/activate
isaacsim
```
In Isaac Sim **Script Editor** (`Window > Script Editor`):
- `File > Open` → `isaac_ros/isaac_single_rx150_setup.py` → **Run**
- Hit **PLAY**

**Terminal 2 — Sequence:**
```bash
python3 working_scripts/rx150_single_sequence.py        # sim
python3 working_scripts/rx150_single_sequence.py --real # real hardware
```

---

## Dual Arm

### Simulation only
```bash
python3 working_scripts/rx150_dual_sequence.py
```

### Real hardware

First, check which USB port each arm is on:
```bash
ls /dev/ttyUSB*
```

Update the `port:` field in each motor config:
- `motor_configs/arm_right.yaml` → port for right arm (default `/dev/ttyUSB0`)
- `motor_configs/arm_left.yaml`  → port for left arm  (default `/dev/ttyUSB1`)

Then run:
```bash
python3 working_scripts/rx150_dual_sequence.py --real
```

### With Isaac Sim

**Terminal 1 — Isaac Sim:**
```bash
source ~/isaaclab/bin/activate
isaacsim
```
In Isaac Sim **Script Editor** (`Window > Script Editor`):
- `File > Open` → `isaac_ros/isaac_dual_rx150_setup.py` → **Run**
- Hit **PLAY**

**Terminal 2 — Sequence:**
```bash
python3 working_scripts/rx150_dual_sequence.py        # sim
python3 working_scripts/rx150_dual_sequence.py --real # real hardware
```

---

## Sequence

Both scripts run the same motion pattern (single arm shown):

| Step | Motion |
|------|--------|
| 1 | HOME pose |
| 2 | Waist +90° (face left) |
| 3 | Waist −90° (face right) |
| 4 | Waist 0° (return to center) |
| 5 | SLEEP pose |

The dual arm script moves both arms in sequence before sleeping.

---

## Isaac Sim Pipeline

```
rx150_*_sequence.py (Interbotix SDK)
    ↓
xs_sdk_sim → /arm_*/joint_states   (sim mode)
xs_sdk     → /arm_*/joint_states   (real hardware)
    ↓
Isaac Sim MirrorBridge (subscribes)
    ↓
USD joint drives → Isaac Sim physics + rendering
```

Isaac Sim mirrors both sim and real hardware — same workflow for both.
