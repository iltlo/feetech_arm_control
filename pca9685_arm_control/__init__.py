"""
PCA9685 Arm Control - Simplified Python-only robotic arm controller

This package provides a minimal, reliable implementation for controlling
a 6-DOF robotic arm with standard PWM hobby servos via a PCA9685 driver.

Designed for Raspberry Pi 5 with I2C connection.
"""

__version__ = "1.0.0"
__author__ = "Adapted from feetech_arm_control"

from .servo_config import ServoConfig, JointConfig, ARM_JOINTS, GRIPPER_JOINT
from .pca9685_driver import PCA9685Driver
from .arm_controller import ArmController, ArmPose

__all__ = [
    "ServoConfig",
    "JointConfig",
    "ARM_JOINTS",
    "GRIPPER_JOINT",
    "PCA9685Driver",
    "ArmController",
    "ArmPose",
]
