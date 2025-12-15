# PCA9685 Arm Control - Simplified Python Robotic Arm Controller

A minimal, reliable Python-only solution for controlling a 6-DOF robotic arm with standard PWM hobby servos via a PCA9685 I2C driver on Raspberry Pi.

**Adapted from**: feetech_arm_control (ROS2/MoveIt/Feetech serial servos)
**Target Platform**: Raspberry Pi 5, Ubuntu 22.04
**Control Method**: Direct PWM via I2C (no ROS2, no MoveIt, no serial bus)

---

## Table of Contents

1. [Overview](#overview)
2. [What Was Removed and Why](#what-was-removed-and-why)
3. [Architecture](#architecture)
4. [Hardware Setup](#hardware-setup)
5. [Software Installation](#software-installation)
6. [Configuration](#configuration)
7. [Usage](#usage)
8. [Calibration](#calibration)
9. [Troubleshooting](#troubleshooting)

---

## Overview

This package provides everything needed for a simple, reliable **Pick & Place demonstration** using:

- **Raspberry Pi 5** (4GB RAM)
- **PCA9685** 16-channel PWM driver (I2C)
- **Standard PWM hobby servos** (SG90, MG996R, or similar)
- **External 5-6V power supply** for servos

### Key Features

- Pure Python implementation (no C++, no ROS2 nodes)
- Direct joint angle control with preset positions
- Smooth motion interpolation with configurable speed
- Interactive calibration utility
- Simulation mode for testing without hardware

---

## What Was Removed and Why

The original feetech_arm_control project was designed for ROS2/MoveIt with Feetech serial servos. Here's what was removed and the rationale:

| Component | Original | Simplified | Why Removed |
|-----------|----------|------------|-------------|
| **ROS2 Infrastructure** | rclpy, sensor_msgs, geometry_msgs | None | Overkill for fixed preset poses. Adds complexity, requires ROS2 installation, increases latency |
| **MoveIt 2** | Motion planning, IK solver, collision avoidance | None | Not needed for known fixed positions. We move joint-by-joint, not in Cartesian space |
| **Feetech SDK** | scservo_sdk, serial bus protocol | PCA9685 I2C | Different hardware (PWM servos vs serial bus servos) |
| **C++ Code** | ee_goal_control.cpp | Python only | User preference, simpler deployment |
| **PyQt5 GUI** | gui_goal_pose.py | CLI only | Not needed for automated demo |
| **URDF/XACRO** | Full robot model, collision geometry | Joint limits only | No RViz visualization, no collision checking needed |
| **Hardware Interface Node** | ROS2 publisher/subscriber | Direct control | No need for ROS2 communication layer |
| **udev Rules** | Serial device mapping | None | I2C doesn't need udev rules |

### What Was Kept (Conceptually)

- **Joint structure**: 6 DOF arm + gripper (adapted to PWM channels)
- **Position limits**: Safety constraints on joint angles
- **Smooth motion**: Interpolation concept (reimplemented for PWM)
- **State machine**: Pick & place sequence logic

---

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    pick_place_demo.py                        │
│                    (High-level sequence)                     │
└─────────────────────────┬───────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────┐
│                    arm_controller.py                         │
│         (Smooth motion, presets, safety limits)              │
└─────────────────────────┬───────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────┐
│                    pca9685_driver.py                         │
│            (I2C communication, PWM generation)               │
└─────────────────────────┬───────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────┐
│                    servo_config.py                           │
│         (Joint limits, PWM parameters, presets)              │
└─────────────────────────────────────────────────────────────┘
```

### File Descriptions

| File | Purpose | Lines |
|------|---------|-------|
| `servo_config.py` | Joint definitions, PWM parameters, preset poses | ~200 |
| `pca9685_driver.py` | Low-level PCA9685 I2C driver | ~250 |
| `arm_controller.py` | High-level arm control with smooth motion | ~350 |
| `pick_place_demo.py` | Complete pick & place demonstration | ~250 |
| `calibration.py` | Interactive servo calibration tool | ~300 |

---

## Hardware Setup

### Wiring Diagram

```
Raspberry Pi 5                    PCA9685
┌─────────────┐                  ┌─────────────────┐
│         3.3V├──────────────────┤VCC              │
│          GND├──────────────────┤GND              │
│    GPIO 2 (SDA)├───────────────┤SDA              │
│    GPIO 3 (SCL)├───────────────┤SCL              │
└─────────────┘                  │                 │
                                 │  V+ ◄───────────┼─── External 5-6V PSU (+)
                                 │ GND ◄───────────┼─── External 5-6V PSU (-)
                                 │                 │
                                 │ CH0 ◄───────────┤─── Base servo signal
                                 │ CH1 ◄───────────┤─── Shoulder servo signal
                                 │ CH2 ◄───────────┤─── Elbow servo signal
                                 │ CH3 ◄───────────┤─── Wrist pitch servo signal
                                 │ CH4 ◄───────────┤─── Wrist roll servo signal
                                 │ CH5 ◄───────────┤─── Wrist yaw servo signal
                                 │ CH6 ◄───────────┤─── Gripper servo signal
                                 └─────────────────┘
```

### Power Supply Requirements

| Component | Voltage | Current |
|-----------|---------|---------|
| PCA9685 Logic | 3.3V (from Pi) | ~10mA |
| Servos (6 joints) | 5-6V | 1-2A per servo under load |
| **Total Servo Power** | **5-6V** | **6-12A recommended** |

**CRITICAL**: Use a dedicated external power supply for servos. Do NOT power servos from the Raspberry Pi!

### Channel Mapping

| Channel | Joint | Typical Servo |
|---------|-------|---------------|
| 0 | Base rotation | MG996R |
| 1 | Shoulder | MG996R |
| 2 | Elbow | MG996R |
| 3 | Wrist pitch | MG90S |
| 4 | Wrist roll | MG90S |
| 5 | Wrist yaw | MG90S |
| 6 | Gripper | SG90 |

---

## Software Installation

### 1. Enable I2C on Raspberry Pi

```bash
sudo raspi-config
# Navigate to: Interface Options -> I2C -> Enable
sudo reboot
```

### 2. Verify I2C Connection

```bash
sudo apt-get install -y i2c-tools
i2cdetect -y 1
# Should show device at address 0x40 (PCA9685 default)
```

### 3. Install Python Dependencies

```bash
cd pca9685_arm_control
pip install -r requirements.txt
```

### 4. Test Installation

```bash
# Test in simulation mode (no hardware required)
python3 pick_place_demo.py --simulate

# Test with hardware
python3 pick_place_demo.py
```

---

## Configuration

### Adjusting Servo Parameters

Edit `servo_config.py` to match your hardware:

```python
ARM_JOINTS = {
    "base": JointConfig(
        name="Base Rotation (Joint 1)",
        servo=ServoConfig(
            channel=0,            # PCA9685 channel
            min_pulse=500,        # Minimum pulse (microseconds)
            max_pulse=2500,       # Maximum pulse (microseconds)
            min_angle=0,          # Angle at min_pulse
            max_angle=180,        # Angle at max_pulse
            inverted=False,       # Reverse direction if True
            default_angle=90,     # Home position
        ),
        soft_min=5,               # Software safety limit (min)
        soft_max=175,             # Software safety limit (max)
        speed_limit=45,           # Degrees per second
    ),
    # ... other joints
}
```

### Adjusting Preset Poses

Edit the `PRESET_POSES` dictionary in `servo_config.py`:

```python
PRESET_POSES = {
    "home": {
        "base": 90,
        "shoulder": 90,
        "elbow": 90,
        "wrist_pitch": 90,
        "wrist_roll": 90,
        "wrist_yaw": 90,
    },
    "pick_approach": {
        "base": 45,
        "shoulder": 45,
        "elbow": 135,
        # ...
    },
    # Add your custom poses here
}
```

### Adjusting Gripper

```python
GRIPPER_OPEN = 150    # Angle for fully open
GRIPPER_CLOSED = 30   # Angle for fully closed
GRIPPER_GRASP = 60    # Angle for gentle grip
```

---

## Usage

### Basic Demo

```bash
# Run one pick & place cycle
python3 pick_place_demo.py

# Run in simulation mode
python3 pick_place_demo.py --simulate

# Run 5 cycles at half speed
python3 pick_place_demo.py --cycles 5 --speed 0.5
```

### Python API

```python
from arm_controller import ArmController

# Initialize
arm = ArmController()
arm.initialize()

# Move to preset position
arm.move_to_preset("home")
arm.move_to_preset("pick_approach", speed_factor=0.5)

# Move to custom angles
arm.move_to_angles({
    "base": 45,
    "shoulder": 60,
    "elbow": 120,
})

# Gripper control
arm.open_gripper()
arm.close_gripper()
arm.grasp()  # Gentle grip

# Cleanup
arm.shutdown()
```

### Custom Pick & Place Sequence

```python
from arm_controller import ArmController
import time

arm = ArmController()
arm.initialize()

try:
    # 1. Start at home
    arm.move_to_preset("home")
    arm.open_gripper()

    # 2. Move to pick location
    arm.move_to_preset("pick_approach")

    # 3. Lower and grasp
    arm.move_to_angles({"shoulder": 35, "elbow": 145})
    arm.grasp()
    time.sleep(0.3)

    # 4. Lift
    arm.move_to_preset("pick_approach")

    # 5. Move to place location
    arm.move_to_preset("place_approach")

    # 6. Lower and release
    arm.move_to_angles({"shoulder": 35, "elbow": 145})
    arm.release()

    # 7. Return home
    arm.move_to_preset("home")

finally:
    arm.shutdown()
```

---

## Calibration

Proper calibration is **essential** for reliable operation.

### Interactive Calibration

```bash
python3 calibration.py
```

Use keyboard controls:
- `0-9`, `a-f`: Select channel (0-15)
- Arrow keys or `w/s`: Adjust pulse width
- `c`: Center (1500μs)
- `m/M`: Min/Max pulse
- `p`: Print calibration data
- `q`: Quit

### Calibration Process

1. **Find center position**: Each joint should be at 90° mechanically
2. **Find minimum**: Move to physical minimum, note pulse value
3. **Find maximum**: Move to physical maximum, note pulse value
4. **Update config**: Edit `servo_config.py` with your values

### Auto-Sweep Test

```bash
# Test channel 0 through full range
python3 calibration.py --sweep 0
```

**WARNING**: Ensure the path is clear before running auto-sweep!

---

## Troubleshooting

### I2C Not Detected

```bash
# Check if I2C is enabled
ls /dev/i2c*
# Should show /dev/i2c-1

# Scan for devices
i2cdetect -y 1
# PCA9685 should appear at 0x40
```

**Solutions**:
- Enable I2C in `raspi-config`
- Check wiring (SDA, SCL connections)
- Verify PCA9685 is powered

### Servos Not Moving

1. **Check power**: Servos need 5-6V external supply
2. **Check connections**: Signal wire to correct channel
3. **Test single servo**:
   ```bash
   python3 calibration.py --simulate
   # Then without --simulate
   ```

### Servo Jitter

**Causes**:
- Insufficient power supply current
- Ground loop (missing common ground)
- PWM frequency issue

**Solutions**:
- Use higher-current power supply (10A+ for 6 servos)
- Connect all grounds together
- Verify 50Hz PWM frequency

### Motion Too Fast/Slow

Adjust in `servo_config.py`:

```python
motion_config = MotionConfig(
    interpolation_steps=50,    # More = smoother
    step_delay=0.02,           # Higher = slower
    max_speed_factor=0.5,      # Lower = slower overall
)
```

### Import Errors

```bash
# If Adafruit libraries fail to import
pip install --upgrade adafruit-blinka
pip install --upgrade adafruit-circuitpython-pca9685

# For Raspberry Pi GPIO
pip install RPi.GPIO
```

---

## Comparison with Original

| Aspect | Original (feetech_arm_control) | Simplified (pca9685_arm_control) |
|--------|-------------------------------|----------------------------------|
| Lines of code | ~1,500+ | ~1,100 |
| Dependencies | ROS2, MoveIt, Feetech SDK, Qt5 | Adafruit PCA9685 only |
| Setup time | Hours (ROS2 workspace) | Minutes (pip install) |
| Motion planning | Full IK, collision avoidance | Direct joint angles |
| Flexibility | Arbitrary Cartesian poses | Fixed preset positions |
| Reliability | Complex, many failure points | Simple, few failure points |
| Use case | Research, flexible manipulation | Demo, fixed pick & place |

---

## License

MIT License - Same as original feetech_arm_control project.

---

## Credits

- Original project: [feetech_arm_control](https://github.com/iltlo/feetech_arm_control)
- Adaptation: Simplified for Raspberry Pi + PCA9685 + PWM servos
