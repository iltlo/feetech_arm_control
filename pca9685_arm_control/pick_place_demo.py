#!/usr/bin/env python3
"""
Pick & Place Demo Script

A complete, reliable demonstration of pick and place operation
using the simplified PCA9685-based arm control.

This script demonstrates:
1. Arm initialization and homing
2. Moving to pick position
3. Grasping an object
4. Moving to place position
5. Releasing the object
6. Returning home

DESIGN DECISIONS:
1. Uses fixed preset positions - no inverse kinematics
   - Positions are calibrated once and stored in servo_config.py
   - Simple, reliable, easy to adjust

2. Smooth motion with speed control
   - Prevents servo jerk and mechanical stress
   - Configurable speed for different parts of the sequence

3. State machine approach
   - Clear sequence of operations
   - Easy to modify or extend
   - Good for debugging

USAGE:
    # Run on Raspberry Pi with PCA9685 connected:
    python3 pick_place_demo.py

    # Run in simulation mode (no hardware):
    python3 pick_place_demo.py --simulate

    # Run with custom cycle count:
    python3 pick_place_demo.py --cycles 5
"""

import argparse
import signal
import sys
import time
from enum import Enum, auto
from typing import Optional

from arm_controller import ArmController, ArmPose, MotionConfig
from servo_config import (
    PRESET_POSES,
    GRIPPER_OPEN,
    GRIPPER_CLOSED,
    GRIPPER_GRASP,
)


class DemoState(Enum):
    """State machine states for pick & place operation."""
    IDLE = auto()
    INITIALIZING = auto()
    HOMING = auto()
    MOVING_TO_PICK = auto()
    LOWERING_TO_PICK = auto()
    GRASPING = auto()
    LIFTING_FROM_PICK = auto()
    MOVING_TO_PLACE = auto()
    LOWERING_TO_PLACE = auto()
    RELEASING = auto()
    LIFTING_FROM_PLACE = auto()
    RETURNING_HOME = auto()
    COMPLETE = auto()
    ERROR = auto()


class PickPlaceDemo:
    """
    Pick & Place demonstration controller.

    Manages the complete pick and place sequence with state tracking,
    error handling, and graceful shutdown.
    """

    def __init__(
        self,
        simulation_mode: bool = False,
        verbose: bool = True,
    ):
        """
        Initialize the demo.

        Args:
            simulation_mode: If True, run without hardware
            verbose: If True, print detailed status messages
        """
        self.simulation_mode = simulation_mode
        self.verbose = verbose
        self.state = DemoState.IDLE
        self._running = False
        self._cycles_completed = 0

        # Configure motion parameters
        motion_config = MotionConfig(
            interpolation_steps=50,
            step_delay=0.02,
            settle_time=0.2,
            max_speed_factor=1.0,
        )

        # Initialize arm controller
        self.arm = ArmController(
            simulation_mode=simulation_mode,
            motion_config=motion_config,
        )

        # Set up signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully."""
        print("\n\nShutdown signal received...")
        self._running = False
        self.shutdown()
        sys.exit(0)

    def _log(self, message: str):
        """Print log message if verbose mode is enabled."""
        if self.verbose:
            print(f"[{self.state.name}] {message}")

    def _set_state(self, new_state: DemoState):
        """Update state with logging."""
        old_state = self.state
        self.state = new_state
        if self.verbose:
            print(f"State: {old_state.name} -> {new_state.name}")

    def initialize(self) -> bool:
        """
        Initialize the arm and prepare for operation.

        Returns:
            True if initialization successful
        """
        self._set_state(DemoState.INITIALIZING)

        try:
            self.arm.initialize()
            self._log("Arm initialized successfully")
            return True

        except Exception as e:
            self._log(f"Initialization failed: {e}")
            self._set_state(DemoState.ERROR)
            return False

    def shutdown(self):
        """Safely shutdown the arm."""
        self._log("Shutting down...")
        self._running = False

        try:
            self.arm.shutdown()
        except Exception as e:
            print(f"Warning during shutdown: {e}")

        self._set_state(DemoState.IDLE)

    def run_pick_place_cycle(self, speed: float = 1.0) -> bool:
        """
        Execute one complete pick and place cycle.

        Args:
            speed: Motion speed multiplier (0.5 = half speed, 2.0 = double)

        Returns:
            True if cycle completed successfully
        """
        try:
            # --- PHASE 1: HOME ---
            self._set_state(DemoState.HOMING)
            self._log("Moving to home position...")
            self.arm.move_to_preset("home", speed_factor=speed)
            self.arm.open_gripper(speed_factor=speed)
            time.sleep(0.3)

            # --- PHASE 2: APPROACH PICK ---
            self._set_state(DemoState.MOVING_TO_PICK)
            self._log("Moving to pick approach position...")
            self.arm.move_to_preset("pick_approach", speed_factor=speed)
            time.sleep(0.2)

            # --- PHASE 3: LOWER TO PICK ---
            self._set_state(DemoState.LOWERING_TO_PICK)
            self._log("Lowering to pick position...")
            # Adjust shoulder/elbow to lower the arm
            # This simulates going down to grab an object
            pick_low = PRESET_POSES["pick_approach"].copy()
            pick_low["shoulder"] = pick_low["shoulder"] - 10  # Lower slightly
            pick_low["elbow"] = pick_low["elbow"] + 10
            self.arm.move_to_angles(pick_low, speed_factor=speed * 0.7)
            time.sleep(0.2)

            # --- PHASE 4: GRASP ---
            self._set_state(DemoState.GRASPING)
            self._log("Closing gripper to grasp object...")
            self.arm.grasp(speed_factor=speed * 0.5)  # Slow grasp
            time.sleep(0.5)  # Hold for grip to settle

            # --- PHASE 5: LIFT FROM PICK ---
            self._set_state(DemoState.LIFTING_FROM_PICK)
            self._log("Lifting from pick position...")
            self.arm.move_to_preset("pick_approach", speed_factor=speed * 0.8)
            time.sleep(0.2)

            # --- PHASE 6: MOVE TO PLACE ---
            self._set_state(DemoState.MOVING_TO_PLACE)
            self._log("Moving to place position...")
            # Go through home to avoid collisions
            self.arm.move_to_preset("home", speed_factor=speed)
            time.sleep(0.1)
            self.arm.move_to_preset("place_approach", speed_factor=speed)
            time.sleep(0.2)

            # --- PHASE 7: LOWER TO PLACE ---
            self._set_state(DemoState.LOWERING_TO_PLACE)
            self._log("Lowering to place position...")
            place_low = PRESET_POSES["place_approach"].copy()
            place_low["shoulder"] = place_low["shoulder"] - 10
            place_low["elbow"] = place_low["elbow"] + 10
            self.arm.move_to_angles(place_low, speed_factor=speed * 0.7)
            time.sleep(0.2)

            # --- PHASE 8: RELEASE ---
            self._set_state(DemoState.RELEASING)
            self._log("Opening gripper to release object...")
            self.arm.release(speed_factor=speed)
            time.sleep(0.3)

            # --- PHASE 9: LIFT FROM PLACE ---
            self._set_state(DemoState.LIFTING_FROM_PLACE)
            self._log("Lifting from place position...")
            self.arm.move_to_preset("place_approach", speed_factor=speed * 0.8)
            time.sleep(0.2)

            # --- PHASE 10: RETURN HOME ---
            self._set_state(DemoState.RETURNING_HOME)
            self._log("Returning to home position...")
            self.arm.move_to_preset("home", speed_factor=speed)
            time.sleep(0.2)

            self._set_state(DemoState.COMPLETE)
            self._cycles_completed += 1
            self._log(f"Cycle {self._cycles_completed} complete!")

            return True

        except Exception as e:
            self._log(f"Error during cycle: {e}")
            self._set_state(DemoState.ERROR)
            return False

    def run(self, cycles: int = 1, speed: float = 1.0, pause_between: float = 1.0):
        """
        Run the pick and place demo for specified number of cycles.

        Args:
            cycles: Number of pick & place cycles to perform
            speed: Motion speed multiplier
            pause_between: Pause duration between cycles (seconds)
        """
        print("\n" + "=" * 50)
        print("    PICK & PLACE DEMO")
        print("=" * 50)
        print(f"Mode: {'SIMULATION' if self.simulation_mode else 'HARDWARE'}")
        print(f"Cycles: {cycles}")
        print(f"Speed: {speed}x")
        print("=" * 50 + "\n")

        if not self.initialize():
            print("Failed to initialize. Exiting.")
            return

        self._running = True

        try:
            for cycle in range(1, cycles + 1):
                if not self._running:
                    print("Demo interrupted.")
                    break

                print(f"\n--- Cycle {cycle}/{cycles} ---")

                success = self.run_pick_place_cycle(speed)

                if not success:
                    print(f"Cycle {cycle} failed!")
                    break

                if cycle < cycles and pause_between > 0:
                    print(f"Pausing for {pause_between}s before next cycle...")
                    time.sleep(pause_between)

            print(f"\n{'=' * 50}")
            print(f"Demo complete! Cycles completed: {self._cycles_completed}/{cycles}")
            print("=" * 50)

        except KeyboardInterrupt:
            print("\n\nDemo interrupted by user.")

        finally:
            self.shutdown()


def main():
    """Main entry point for the pick & place demo."""
    parser = argparse.ArgumentParser(
        description="Pick & Place Demo for PCA9685-based robotic arm",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run one cycle with hardware:
  python3 pick_place_demo.py

  # Run in simulation mode:
  python3 pick_place_demo.py --simulate

  # Run 5 cycles at half speed:
  python3 pick_place_demo.py --cycles 5 --speed 0.5

  # Run 3 cycles with 2 second pause between:
  python3 pick_place_demo.py --cycles 3 --pause 2.0
        """
    )

    parser.add_argument(
        "--simulate", "-s",
        action="store_true",
        help="Run in simulation mode (no hardware required)"
    )

    parser.add_argument(
        "--cycles", "-c",
        type=int,
        default=1,
        help="Number of pick & place cycles to run (default: 1)"
    )

    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="Speed multiplier, 0.1-2.0 (default: 1.0)"
    )

    parser.add_argument(
        "--pause", "-p",
        type=float,
        default=1.0,
        help="Pause between cycles in seconds (default: 1.0)"
    )

    parser.add_argument(
        "--quiet", "-q",
        action="store_true",
        help="Reduce output verbosity"
    )

    args = parser.parse_args()

    # Validate arguments
    if args.cycles < 1:
        parser.error("Cycles must be at least 1")

    if not 0.1 <= args.speed <= 2.0:
        parser.error("Speed must be between 0.1 and 2.0")

    # Run the demo
    demo = PickPlaceDemo(
        simulation_mode=args.simulate,
        verbose=not args.quiet,
    )

    demo.run(
        cycles=args.cycles,
        speed=args.speed,
        pause_between=args.pause,
    )


if __name__ == "__main__":
    main()
