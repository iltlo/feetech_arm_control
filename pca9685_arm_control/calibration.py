#!/usr/bin/env python3
"""
Servo Calibration Utility

Interactive tool to calibrate servo positions and find the correct
pulse widths for your specific hardware setup.

This is CRITICAL for reliable operation because:
1. Different servos have different pulse width ranges
2. Servo mounting direction affects angle mapping
3. Physical limits may differ from servo specifications

USAGE:
    python3 calibration.py

    Then use keyboard commands to adjust positions and save values.
"""

import sys
import time
from typing import Optional

try:
    import termios
    import tty
    TERMINAL_AVAILABLE = True
except ImportError:
    TERMINAL_AVAILABLE = False

from pca9685_driver import PCA9685Driver
from servo_config import ARM_JOINTS, GRIPPER_JOINT


class ServoCalibrator:
    """
    Interactive servo calibration tool.

    Allows you to manually adjust servo positions to find the correct
    PWM values for your specific hardware.
    """

    def __init__(self, simulation_mode: bool = False):
        """Initialize the calibrator."""
        self.simulation_mode = simulation_mode
        self.driver = PCA9685Driver(simulation_mode=simulation_mode)

        # Current calibration state
        self.current_channel = 0
        self.current_pulse = 1500  # Start at center
        self.pulse_step = 10  # Small step for fine adjustment

        # Calibration results
        self.calibration_data = {}

    def initialize(self):
        """Initialize the driver."""
        self.driver.initialize()
        print("Driver initialized")

    def shutdown(self):
        """Shutdown the driver."""
        self.driver.shutdown()
        print("Driver shutdown")

    def set_channel(self, channel: int):
        """Select a channel for calibration."""
        if 0 <= channel <= 15:
            self.current_channel = channel
            print(f"\nSelected channel {channel}")
            self._update_servo()
        else:
            print(f"Invalid channel: {channel} (must be 0-15)")

    def set_pulse(self, pulse_us: int):
        """Set the pulse width directly."""
        self.current_pulse = max(200, min(2800, pulse_us))  # Safety limits
        self._update_servo()

    def adjust_pulse(self, delta: int):
        """Adjust pulse by delta."""
        self.current_pulse = max(200, min(2800, self.current_pulse + delta))
        self._update_servo()

    def _update_servo(self):
        """Apply current pulse to current channel."""
        self.driver.set_servo_pulse(self.current_channel, self.current_pulse)
        print(f"Channel {self.current_channel}: {self.current_pulse}us", end='\r')

    def save_position(self, position_name: str):
        """Save current position with a name."""
        key = f"ch{self.current_channel}_{position_name}"
        self.calibration_data[key] = self.current_pulse
        print(f"\nSaved: {key} = {self.current_pulse}us")

    def print_calibration(self):
        """Print all calibration data."""
        print("\n" + "=" * 50)
        print("CALIBRATION DATA")
        print("=" * 50)

        if not self.calibration_data:
            print("No calibration data saved yet.")
        else:
            for key, value in sorted(self.calibration_data.items()):
                print(f"  {key}: {value}us")

        print("=" * 50)

    def print_help(self):
        """Print help message."""
        print("""
╔═══════════════════════════════════════════════════════════════╗
║                 SERVO CALIBRATION UTILITY                      ║
╠═══════════════════════════════════════════════════════════════╣
║  CHANNEL SELECTION:                                            ║
║    0-9    : Select channel 0-9                                 ║
║    a-f    : Select channel 10-15 (a=10, b=11, ..., f=15)      ║
║                                                                ║
║  PULSE ADJUSTMENT:                                             ║
║    ↑/w    : Increase pulse by 10us (fine)                      ║
║    ↓/s    : Decrease pulse by 10us (fine)                      ║
║    →/d    : Increase pulse by 50us (coarse)                    ║
║    ←/a    : Decrease pulse by 50us (coarse)                    ║
║    +/=    : Increase pulse by 100us (very coarse)              ║
║    -/_    : Decrease pulse by 100us (very coarse)              ║
║                                                                ║
║  PRESETS:                                                      ║
║    c      : Center (1500us)                                    ║
║    m      : Minimum (500us)                                    ║
║    M      : Maximum (2500us)                                   ║
║                                                                ║
║  SAVE/DISPLAY:                                                 ║
║    1      : Save as "min" position                             ║
║    2      : Save as "center" position                          ║
║    3      : Save as "max" position                             ║
║    p      : Print all saved calibration data                   ║
║                                                                ║
║  OTHER:                                                        ║
║    h/?    : Show this help                                     ║
║    q/Esc  : Quit                                               ║
╚═══════════════════════════════════════════════════════════════╝
        """)

    def run_interactive(self):
        """Run interactive calibration session."""
        print("\n" + "=" * 60)
        print("     INTERACTIVE SERVO CALIBRATION")
        print("=" * 60)
        print(f"Mode: {'SIMULATION' if self.simulation_mode else 'HARDWARE'}")
        print("Press 'h' for help, 'q' to quit")
        print("=" * 60)

        self.print_help()
        self._update_servo()

        if not TERMINAL_AVAILABLE:
            print("\nTerminal input not available. Using input mode.")
            self._run_input_mode()
            return

        # Set terminal to raw mode for single key presses
        old_settings = termios.tcgetattr(sys.stdin)

        try:
            tty.setraw(sys.stdin.fileno())

            while True:
                char = sys.stdin.read(1)

                # Handle escape sequences (arrow keys)
                if char == '\x1b':
                    char += sys.stdin.read(2)
                    if char == '\x1b[A':  # Up arrow
                        self.adjust_pulse(10)
                    elif char == '\x1b[B':  # Down arrow
                        self.adjust_pulse(-10)
                    elif char == '\x1b[C':  # Right arrow
                        self.adjust_pulse(50)
                    elif char == '\x1b[D':  # Left arrow
                        self.adjust_pulse(-50)
                    continue

                # Channel selection
                if char.isdigit():
                    self.set_channel(int(char))
                elif char in 'abcdef':
                    self.set_channel(ord(char) - ord('a') + 10)

                # Pulse adjustment
                elif char in 'wW':
                    self.adjust_pulse(10)
                elif char in 'sS':
                    self.adjust_pulse(-10)
                elif char in 'dD':
                    self.adjust_pulse(50)
                elif char in 'aA' and char != 'a':  # 'a' is for channel 10
                    self.adjust_pulse(-50)
                elif char in '+=':
                    self.adjust_pulse(100)
                elif char in '-_':
                    self.adjust_pulse(-100)

                # Presets
                elif char == 'c':
                    self.set_pulse(1500)
                elif char == 'm':
                    self.set_pulse(500)
                elif char == 'M':
                    self.set_pulse(2500)

                # Save positions
                elif char == '!':
                    self.save_position("min")
                elif char == '@':
                    self.save_position("center")
                elif char == '#':
                    self.save_position("max")

                # Display
                elif char == 'p':
                    self.print_calibration()

                # Help
                elif char in 'h?':
                    self.print_help()

                # Quit
                elif char in 'qQ\x03':  # q, Q, or Ctrl+C
                    print("\n\nExiting calibration...")
                    break

        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

        self.print_calibration()

    def _run_input_mode(self):
        """Fallback mode using standard input()."""
        print("\nEnter commands (type 'help' for list, 'quit' to exit):")

        while True:
            try:
                cmd = input("\n> ").strip().lower()

                if not cmd:
                    continue
                elif cmd in ('quit', 'exit', 'q'):
                    break
                elif cmd == 'help':
                    print("Commands: channel N, pulse N, +/- N, center, min, max, save NAME, print, quit")
                elif cmd.startswith('channel '):
                    try:
                        ch = int(cmd.split()[1])
                        self.set_channel(ch)
                    except (ValueError, IndexError):
                        print("Usage: channel N (where N is 0-15)")
                elif cmd.startswith('pulse '):
                    try:
                        pulse = int(cmd.split()[1])
                        self.set_pulse(pulse)
                    except (ValueError, IndexError):
                        print("Usage: pulse N (where N is pulse width in us)")
                elif cmd.startswith('+'):
                    try:
                        delta = int(cmd[1:]) if len(cmd) > 1 else 10
                        self.adjust_pulse(delta)
                    except ValueError:
                        self.adjust_pulse(10)
                elif cmd.startswith('-'):
                    try:
                        delta = int(cmd[1:]) if len(cmd) > 1 else 10
                        self.adjust_pulse(-delta)
                    except ValueError:
                        self.adjust_pulse(-10)
                elif cmd == 'center':
                    self.set_pulse(1500)
                elif cmd == 'min':
                    self.set_pulse(500)
                elif cmd == 'max':
                    self.set_pulse(2500)
                elif cmd.startswith('save '):
                    name = cmd.split(maxsplit=1)[1] if len(cmd.split()) > 1 else "position"
                    self.save_position(name)
                elif cmd == 'print':
                    self.print_calibration()
                else:
                    print(f"Unknown command: {cmd}")

            except KeyboardInterrupt:
                break
            except EOFError:
                break

        self.print_calibration()

    def run_auto_sweep(self, channel: int):
        """
        Run an automatic sweep on a channel to find limits.

        WARNING: This moves the servo through its full range!
        Make sure the path is clear.
        """
        print(f"\nAuto-sweep channel {channel}")
        print("WARNING: Servo will move through full range!")
        print("Press Ctrl+C to stop at any time.")
        print("Starting in 3 seconds...")
        time.sleep(3)

        self.set_channel(channel)

        # Sweep from min to max
        print("\nSweeping up...")
        for pulse in range(500, 2501, 25):
            self.set_pulse(pulse)
            time.sleep(0.05)

        time.sleep(0.5)

        # Sweep back
        print("Sweeping down...")
        for pulse in range(2500, 499, -25):
            self.set_pulse(pulse)
            time.sleep(0.05)

        # Return to center
        print("Returning to center...")
        self.set_pulse(1500)

        print("Sweep complete!")


def main():
    """Main entry point for calibration utility."""
    import argparse

    parser = argparse.ArgumentParser(description="Servo Calibration Utility")
    parser.add_argument("--simulate", "-s", action="store_true",
                        help="Run in simulation mode")
    parser.add_argument("--sweep", type=int, metavar="CHANNEL",
                        help="Run auto-sweep on specified channel")

    args = parser.parse_args()

    calibrator = ServoCalibrator(simulation_mode=args.simulate)

    try:
        calibrator.initialize()

        if args.sweep is not None:
            calibrator.run_auto_sweep(args.sweep)
        else:
            calibrator.run_interactive()

    except KeyboardInterrupt:
        print("\n\nCalibration interrupted.")
    finally:
        calibrator.shutdown()


if __name__ == "__main__":
    main()
