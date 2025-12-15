"""
Arm Controller Module

High-level robotic arm control with smooth motion interpolation,
preset poses, and safety features.

DESIGN DECISIONS:
1. NO inverse kinematics - Uses direct joint angle control with presets
   - Simple, reliable, easy to calibrate
   - MoveIt/IK is overkill for fixed pick & place positions

2. Smooth motion via linear interpolation with speed limiting
   - Prevents servo jerk and mechanical stress
   - Configurable speed per joint

3. Preset-based operation
   - Define pick/place positions once, use reliably
   - Easy to adjust without code changes
"""

import time
from typing import Dict, Optional, Callable, List
from dataclasses import dataclass, field

from .pca9685_driver import PCA9685Driver
from .servo_config import (
    ARM_JOINTS,
    GRIPPER_JOINT,
    JointConfig,
    ServoConfig,
    PRESET_POSES,
    GRIPPER_OPEN,
    GRIPPER_CLOSED,
    GRIPPER_GRASP,
    validate_joint_angle,
)


@dataclass
class ArmPose:
    """
    Represents a complete arm pose (all joint angles).

    Attributes:
        name: Human-readable pose name
        joints: Dictionary mapping joint names to angles (degrees)
        gripper: Gripper angle (degrees), None means don't change
    """
    name: str
    joints: Dict[str, float]
    gripper: Optional[float] = None

    @classmethod
    def from_preset(cls, preset_name: str, gripper: Optional[float] = None) -> "ArmPose":
        """Create an ArmPose from a preset name."""
        if preset_name not in PRESET_POSES:
            raise ValueError(f"Unknown preset: {preset_name}. Available: {list(PRESET_POSES.keys())}")
        return cls(
            name=preset_name,
            joints=PRESET_POSES[preset_name].copy(),
            gripper=gripper
        )


@dataclass
class MotionConfig:
    """Configuration for motion execution."""
    interpolation_steps: int = 50    # Number of steps for smooth motion
    step_delay: float = 0.02         # Delay between interpolation steps (seconds)
    settle_time: float = 0.1         # Time to wait after motion completes
    max_speed_factor: float = 1.0    # Global speed multiplier (0.1 = 10x slower)


class ArmController:
    """
    High-level controller for the 6-DOF robotic arm.

    This class provides:
    - Smooth motion between positions
    - Preset pose management
    - Gripper control
    - Safety limit enforcement

    Example:
        arm = ArmController()
        arm.initialize()

        # Move to home position
        arm.move_to_preset("home")

        # Open gripper
        arm.open_gripper()

        # Move to custom position
        arm.move_to_angles({"base": 45, "shoulder": 60, "elbow": 120})

        arm.shutdown()
    """

    def __init__(
        self,
        joints: Dict[str, JointConfig] = None,
        gripper: JointConfig = None,
        motion_config: MotionConfig = None,
        simulation_mode: bool = False,
    ):
        """
        Initialize the arm controller.

        Args:
            joints: Joint configuration dictionary (defaults to ARM_JOINTS)
            gripper: Gripper configuration (defaults to GRIPPER_JOINT)
            motion_config: Motion parameters (defaults to MotionConfig())
            simulation_mode: If True, no hardware communication
        """
        self.joints = joints or ARM_JOINTS
        self.gripper = gripper or GRIPPER_JOINT
        self.motion_config = motion_config or MotionConfig()
        self.simulation_mode = simulation_mode

        # Initialize driver
        self._driver = PCA9685Driver(simulation_mode=simulation_mode)

        # Track current positions
        self._current_angles: Dict[str, float] = {}
        self._current_gripper: float = 90.0
        self._initialized = False

        # Callbacks for monitoring
        self._on_motion_start: Optional[Callable] = None
        self._on_motion_complete: Optional[Callable] = None
        self._on_position_update: Optional[Callable] = None

    def initialize(self) -> bool:
        """
        Initialize the arm hardware and move to home position.

        Returns:
            True if successful
        """
        if self._initialized:
            print("Arm already initialized")
            return True

        print("Initializing arm controller...")

        # Initialize PCA9685
        self._driver.initialize()

        # Initialize all joints to their default positions
        for name, config in self.joints.items():
            self._current_angles[name] = config.servo.default_angle
            self._set_joint_direct(name, config.servo.default_angle)

        # Initialize gripper
        self._current_gripper = self.gripper.servo.default_angle
        self._set_gripper_direct(self._current_gripper)

        self._initialized = True
        print("Arm controller initialized")

        # Small delay to let servos settle
        time.sleep(0.5)

        return True

    def shutdown(self):
        """
        Safely shutdown the arm.

        Moves to home position if possible, then disables servos.
        """
        if not self._initialized:
            return

        print("Shutting down arm controller...")

        try:
            # Try to move home first
            self.move_to_preset("home", speed_factor=0.5)
        except Exception as e:
            print(f"Warning: Could not move to home: {e}")

        # Shutdown driver
        self._driver.shutdown()
        self._initialized = False
        print("Arm controller shutdown complete")

    def _set_joint_direct(self, joint_name: str, angle: float):
        """
        Set a joint angle directly (no interpolation).

        Internal method - use move_to_angles for smooth motion.
        """
        if joint_name not in self.joints:
            raise ValueError(f"Unknown joint: {joint_name}")

        config = self.joints[joint_name]
        servo = config.servo

        # Apply safety limits
        angle = validate_joint_angle(joint_name, angle, self.joints)

        # Set servo
        self._driver.set_servo_angle(
            channel=servo.channel,
            angle=angle,
            min_pulse=servo.min_pulse,
            max_pulse=servo.max_pulse,
            min_angle=servo.min_angle,
            max_angle=servo.max_angle,
            inverted=servo.inverted,
        )

        self._current_angles[joint_name] = angle

    def _set_gripper_direct(self, angle: float):
        """Set gripper angle directly (no interpolation)."""
        servo = self.gripper.servo

        # Apply limits
        angle = max(self.gripper.soft_min, min(self.gripper.soft_max, angle))

        self._driver.set_servo_angle(
            channel=servo.channel,
            angle=angle,
            min_pulse=servo.min_pulse,
            max_pulse=servo.max_pulse,
            min_angle=servo.min_angle,
            max_angle=servo.max_angle,
            inverted=servo.inverted,
        )

        self._current_gripper = angle

    def move_to_angles(
        self,
        target_angles: Dict[str, float],
        speed_factor: float = 1.0,
        block: bool = True,
    ):
        """
        Move joints to specified angles with smooth interpolation.

        Args:
            target_angles: Dictionary of joint names to target angles (degrees)
            speed_factor: Motion speed multiplier (0.1-2.0, default 1.0)
            block: If True, wait for motion to complete before returning

        Example:
            arm.move_to_angles({
                "base": 45,
                "shoulder": 60,
                "elbow": 120,
            })
        """
        if not self._initialized:
            raise RuntimeError("Arm not initialized. Call initialize() first.")

        # Clamp speed factor
        speed_factor = max(0.1, min(2.0, speed_factor)) * self.motion_config.max_speed_factor

        # Notify motion start
        if self._on_motion_start:
            self._on_motion_start(target_angles)

        # Get current and target positions for interpolation
        start_angles = self._current_angles.copy()

        # Validate and clamp target angles
        validated_targets = {}
        for name, angle in target_angles.items():
            if name in self.joints:
                validated_targets[name] = validate_joint_angle(name, angle, self.joints)
            else:
                print(f"Warning: Unknown joint '{name}' ignored")

        if not validated_targets:
            print("No valid target angles provided")
            return

        # Calculate number of steps based on max angle change and speed
        max_delta = 0
        for name, target in validated_targets.items():
            delta = abs(target - start_angles.get(name, target))
            max_delta = max(max_delta, delta)

        # More steps for larger movements
        base_steps = self.motion_config.interpolation_steps
        steps = max(10, int(base_steps * (max_delta / 90.0)))  # Scale by angle change
        steps = int(steps / speed_factor)  # Adjust for speed

        # Calculate step delay
        step_delay = self.motion_config.step_delay / speed_factor

        # Perform interpolation
        for i in range(1, steps + 1):
            t = i / steps  # Progress 0.0 -> 1.0

            # Smooth easing (ease-in-out)
            t = self._ease_in_out(t)

            # Interpolate each joint
            for name, target in validated_targets.items():
                start = start_angles.get(name, target)
                current = start + (target - start) * t
                self._set_joint_direct(name, current)

            # Notify position update
            if self._on_position_update:
                self._on_position_update(self._current_angles.copy(), i, steps)

            if block:
                time.sleep(step_delay)

        # Ensure we hit exact target
        for name, target in validated_targets.items():
            self._set_joint_direct(name, target)

        # Settle time
        if block:
            time.sleep(self.motion_config.settle_time)

        # Notify motion complete
        if self._on_motion_complete:
            self._on_motion_complete(self._current_angles.copy())

    @staticmethod
    def _ease_in_out(t: float) -> float:
        """
        Smooth ease-in-out interpolation.

        Creates smooth acceleration and deceleration.
        """
        if t < 0.5:
            return 2 * t * t
        else:
            return 1 - ((-2 * t + 2) ** 2) / 2

    def move_to_pose(self, pose: ArmPose, speed_factor: float = 1.0):
        """
        Move to a complete arm pose.

        Args:
            pose: ArmPose object with joint angles
            speed_factor: Motion speed multiplier
        """
        print(f"Moving to pose: {pose.name}")

        # Move arm joints
        self.move_to_angles(pose.joints, speed_factor)

        # Set gripper if specified
        if pose.gripper is not None:
            self.set_gripper(pose.gripper, speed_factor)

    def move_to_preset(self, preset_name: str, speed_factor: float = 1.0, gripper: Optional[float] = None):
        """
        Move to a preset pose by name.

        Args:
            preset_name: Name of the preset (e.g., "home", "pick_approach")
            speed_factor: Motion speed multiplier
            gripper: Optional gripper angle to set after moving
        """
        pose = ArmPose.from_preset(preset_name, gripper)
        self.move_to_pose(pose, speed_factor)

    def set_gripper(self, angle: float, speed_factor: float = 1.0):
        """
        Set gripper position with smooth motion.

        Args:
            angle: Target gripper angle in degrees
            speed_factor: Motion speed multiplier
        """
        if not self._initialized:
            raise RuntimeError("Arm not initialized")

        # Clamp angle
        angle = max(self.gripper.soft_min, min(self.gripper.soft_max, angle))

        # Interpolate gripper motion
        start = self._current_gripper
        steps = int(30 / speed_factor)
        step_delay = 0.015 / speed_factor

        for i in range(1, steps + 1):
            t = self._ease_in_out(i / steps)
            current = start + (angle - start) * t
            self._set_gripper_direct(current)
            time.sleep(step_delay)

        self._set_gripper_direct(angle)
        time.sleep(0.05)

    def open_gripper(self, speed_factor: float = 1.0):
        """Open the gripper fully."""
        print("Opening gripper")
        self.set_gripper(GRIPPER_OPEN, speed_factor)

    def close_gripper(self, speed_factor: float = 1.0):
        """Close the gripper fully."""
        print("Closing gripper")
        self.set_gripper(GRIPPER_CLOSED, speed_factor)

    def grasp(self, speed_factor: float = 0.7):
        """
        Perform a gentle grasp (for picking up objects).

        Uses a softer grip than full close to avoid crushing.
        """
        print("Grasping")
        self.set_gripper(GRIPPER_GRASP, speed_factor)

    def release(self, speed_factor: float = 1.0):
        """Release object by opening gripper."""
        print("Releasing")
        self.open_gripper(speed_factor)

    def get_current_angles(self) -> Dict[str, float]:
        """Get current joint angles (degrees)."""
        return self._current_angles.copy()

    def get_current_gripper(self) -> float:
        """Get current gripper angle (degrees)."""
        return self._current_gripper

    def emergency_stop(self):
        """
        Emergency stop - immediately disable all servos.

        Use in case of unexpected behavior. Servos will go limp.
        """
        print("EMERGENCY STOP!")

        for config in self.joints.values():
            self._driver.disable_channel(config.servo.channel)

        self._driver.disable_channel(self.gripper.servo.channel)

    def set_motion_callbacks(
        self,
        on_start: Optional[Callable] = None,
        on_complete: Optional[Callable] = None,
        on_update: Optional[Callable] = None,
    ):
        """
        Set callbacks for motion events.

        Args:
            on_start: Called when motion begins, receives target_angles dict
            on_complete: Called when motion ends, receives final_angles dict
            on_update: Called during motion, receives (current_angles, step, total_steps)
        """
        self._on_motion_start = on_start
        self._on_motion_complete = on_complete
        self._on_position_update = on_update


def test_arm_controller():
    """Test function for the arm controller."""
    print("Arm Controller Test")
    print("=" * 40)

    # Use simulation mode for testing without hardware
    arm = ArmController(simulation_mode=True)

    # Set up callbacks for monitoring
    arm.set_motion_callbacks(
        on_start=lambda t: print(f"Motion started: {t}"),
        on_complete=lambda a: print(f"Motion complete: {a}"),
    )

    try:
        arm.initialize()

        print("\n1. Moving to home position...")
        arm.move_to_preset("home")

        print("\n2. Opening gripper...")
        arm.open_gripper()

        print("\n3. Moving to ready position...")
        arm.move_to_preset("ready")

        print("\n4. Moving to pick approach...")
        arm.move_to_preset("pick_approach")

        print("\n5. Grasping...")
        arm.grasp()

        print("\n6. Moving to place approach...")
        arm.move_to_preset("place_approach")

        print("\n7. Releasing...")
        arm.release()

        print("\n8. Moving back home...")
        arm.move_to_preset("home")

        print("\nTest complete!")

    except Exception as e:
        print(f"Error during test: {e}")
        arm.emergency_stop()

    finally:
        arm.shutdown()


if __name__ == "__main__":
    test_arm_controller()
