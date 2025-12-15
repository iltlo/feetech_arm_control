"""
Servo Configuration Module

Defines joint limits, PWM parameters, and servo mappings for the robotic arm.
This replaces the complex Feetech serial configuration with simple PWM parameters.

DESIGN DECISION:
- Standard hobby servos (SG90, MG996R, etc.) typically accept PWM signals:
  - Frequency: 50 Hz (20ms period)
  - Pulse width: 500-2500 microseconds (0.5-2.5ms)
  - PCA9685 uses 12-bit resolution (0-4095) for duty cycle
  - At 50Hz: 1ms pulse = ~205 counts, 2ms pulse = ~410 counts
"""

from dataclasses import dataclass
from typing import Dict, Optional


@dataclass
class ServoConfig:
    """
    Configuration for a single servo motor.

    PWM Calculation (for 50Hz, 12-bit resolution):
    - Period = 20ms = 20000 microseconds
    - 1 count = 20000/4096 = ~4.88 microseconds
    - 500us pulse = 500/4.88 = ~102 counts
    - 2500us pulse = 2500/4.88 = ~512 counts

    Attributes:
        channel: PCA9685 channel number (0-15)
        min_pulse: Minimum pulse width in microseconds (typically 500-600)
        max_pulse: Maximum pulse width in microseconds (typically 2400-2500)
        min_angle: Minimum angle in degrees
        max_angle: Maximum angle in degrees
        inverted: If True, angle mapping is reversed
        default_angle: Default/home angle for this servo
    """
    channel: int
    min_pulse: int = 500      # microseconds
    max_pulse: int = 2500     # microseconds
    min_angle: float = 0.0    # degrees
    max_angle: float = 180.0  # degrees
    inverted: bool = False
    default_angle: float = 90.0


@dataclass
class JointConfig:
    """
    Robot joint configuration with safety limits.

    Attributes:
        name: Human-readable joint name
        servo: Servo configuration
        soft_min: Software minimum angle limit (safety margin)
        soft_max: Software maximum angle limit (safety margin)
        speed_limit: Maximum angular velocity in degrees/second
    """
    name: str
    servo: ServoConfig
    soft_min: float = 10.0    # degrees (safety margin from mechanical limit)
    soft_max: float = 170.0   # degrees (safety margin from mechanical limit)
    speed_limit: float = 60.0  # degrees per second (for smooth motion)


# =============================================================================
# ARM JOINT DEFINITIONS
# =============================================================================
# Mapping based on typical 6-DOF arm structure:
# - Joint 1 (Base): Rotates the entire arm horizontally
# - Joint 2 (Shoulder): Lifts/lowers the upper arm
# - Joint 3 (Elbow): Bends the forearm
# - Joint 4 (Wrist Pitch): Tilts the wrist up/down
# - Joint 5 (Wrist Roll): Rotates the wrist
# - Joint 6 (Wrist Yaw): Final orientation adjustment
# - Gripper: Opens/closes the end effector
#
# IMPORTANT: Adjust these values based on your actual servo calibration!
# =============================================================================

ARM_JOINTS: Dict[str, JointConfig] = {
    "base": JointConfig(
        name="Base Rotation (Joint 1)",
        servo=ServoConfig(
            channel=0,
            min_pulse=500,
            max_pulse=2500,
            min_angle=0,
            max_angle=180,
            default_angle=90,  # Center position
        ),
        soft_min=5,
        soft_max=175,
        speed_limit=45,  # Slower for heavy base rotation
    ),
    "shoulder": JointConfig(
        name="Shoulder (Joint 2)",
        servo=ServoConfig(
            channel=1,
            min_pulse=500,
            max_pulse=2500,
            min_angle=0,
            max_angle=180,
            inverted=False,  # Adjust based on servo mounting
            default_angle=90,
        ),
        soft_min=15,
        soft_max=165,
        speed_limit=40,  # Slower, carries most weight
    ),
    "elbow": JointConfig(
        name="Elbow (Joint 3)",
        servo=ServoConfig(
            channel=2,
            min_pulse=500,
            max_pulse=2500,
            min_angle=0,
            max_angle=180,
            default_angle=90,
        ),
        soft_min=10,
        soft_max=170,
        speed_limit=50,
    ),
    "wrist_pitch": JointConfig(
        name="Wrist Pitch (Joint 4)",
        servo=ServoConfig(
            channel=3,
            min_pulse=500,
            max_pulse=2500,
            min_angle=0,
            max_angle=180,
            default_angle=90,
        ),
        soft_min=5,
        soft_max=175,
        speed_limit=60,
    ),
    "wrist_roll": JointConfig(
        name="Wrist Roll (Joint 5)",
        servo=ServoConfig(
            channel=4,
            min_pulse=500,
            max_pulse=2500,
            min_angle=0,
            max_angle=180,
            default_angle=90,
        ),
        soft_min=0,
        soft_max=180,
        speed_limit=90,  # Can be faster, low load
    ),
    "wrist_yaw": JointConfig(
        name="Wrist Yaw (Joint 6)",
        servo=ServoConfig(
            channel=5,
            min_pulse=500,
            max_pulse=2500,
            min_angle=0,
            max_angle=180,
            default_angle=90,
        ),
        soft_min=0,
        soft_max=180,
        speed_limit=90,
    ),
}

GRIPPER_JOINT = JointConfig(
    name="Gripper",
    servo=ServoConfig(
        channel=6,
        min_pulse=500,
        max_pulse=2500,
        min_angle=0,    # Fully closed
        max_angle=180,  # Fully open (adjust based on your gripper)
        default_angle=90,  # Half open
    ),
    soft_min=10,   # Don't fully crush
    soft_max=170,  # Don't over-extend
    speed_limit=120,  # Gripper can be fast
)


# =============================================================================
# PRESET POSES
# =============================================================================
# These define common arm positions for pick & place operations.
# Values are in DEGREES for each joint.
# Adjust these based on your physical setup and workspace!
# =============================================================================

PRESET_POSES = {
    "home": {
        "base": 90,
        "shoulder": 90,
        "elbow": 90,
        "wrist_pitch": 90,
        "wrist_roll": 90,
        "wrist_yaw": 90,
    },
    "ready": {
        "base": 90,
        "shoulder": 60,
        "elbow": 120,
        "wrist_pitch": 90,
        "wrist_roll": 90,
        "wrist_yaw": 90,
    },
    # Pick position: arm extended forward and down
    "pick_approach": {
        "base": 45,       # Rotated toward pick location
        "shoulder": 45,   # Lowered
        "elbow": 135,     # Extended
        "wrist_pitch": 45,  # Angled down
        "wrist_roll": 90,
        "wrist_yaw": 90,
    },
    # Place position: arm extended to the other side
    "place_approach": {
        "base": 135,      # Rotated toward place location
        "shoulder": 45,
        "elbow": 135,
        "wrist_pitch": 45,
        "wrist_roll": 90,
        "wrist_yaw": 90,
    },
}

# Gripper states
GRIPPER_OPEN = 150    # degrees (adjust for your gripper)
GRIPPER_CLOSED = 30   # degrees (adjust for your gripper)
GRIPPER_GRASP = 60    # degrees (gentle grip, adjust based on object size)


def validate_joint_angle(joint_name: str, angle: float, joints: Dict[str, JointConfig] = None) -> float:
    """
    Validate and clamp a joint angle to safe limits.

    Args:
        joint_name: Name of the joint (e.g., "base", "shoulder")
        angle: Desired angle in degrees
        joints: Joint configuration dictionary (defaults to ARM_JOINTS)

    Returns:
        Clamped angle within soft limits

    Raises:
        KeyError: If joint_name is not found
    """
    if joints is None:
        joints = ARM_JOINTS

    if joint_name not in joints:
        raise KeyError(f"Unknown joint: {joint_name}. Valid joints: {list(joints.keys())}")

    config = joints[joint_name]
    clamped = max(config.soft_min, min(config.soft_max, angle))

    if clamped != angle:
        print(f"WARNING: Joint '{joint_name}' angle {angle}° clamped to {clamped}° (limits: {config.soft_min}°-{config.soft_max}°)")

    return clamped
