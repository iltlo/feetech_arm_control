"""
PCA9685 PWM Driver Module

Low-level interface for the PCA9685 16-channel PWM driver via I2C.
Designed for Raspberry Pi 5 with standard hobby servos.

DESIGN DECISIONS:
1. Uses the Adafruit CircuitPython PCA9685 library (pip install adafruit-circuitpython-pca9685)
   - Well-maintained, Raspberry Pi 5 compatible
   - Alternative: RPi.GPIO + smbus2 for bare-metal (more complex)

2. Frequency fixed at 50Hz for standard servo PWM
   - Can be adjusted for specific servo requirements

3. 12-bit resolution (0-4095) for duty cycle
   - Provides ~0.044° resolution for 180° servo range

WIRING:
- PCA9685 VCC -> Raspberry Pi 3.3V
- PCA9685 GND -> Raspberry Pi GND
- PCA9685 SDA -> Raspberry Pi GPIO 2 (SDA)
- PCA9685 SCL -> Raspberry Pi GPIO 3 (SCL)
- PCA9685 V+ -> External 5-6V power supply (for servos)
- Servo signal wires -> PCA9685 PWM channels 0-15
"""

import time
from typing import Optional, List
from dataclasses import dataclass


# Constants for PCA9685
PCA9685_ADDRESS = 0x40  # Default I2C address
PWM_FREQUENCY = 50      # Hz - standard for hobby servos
PWM_RESOLUTION = 4096   # 12-bit (0-4095)
PERIOD_US = 20000       # 20ms period at 50Hz


@dataclass
class PWMChannel:
    """Represents a single PWM channel state."""
    channel: int
    pulse_width_us: int = 1500  # Current pulse width in microseconds
    enabled: bool = True


class PCA9685Driver:
    """
    PCA9685 PWM Driver for servo control.

    This class provides a clean interface for controlling servos via the
    PCA9685 16-channel PWM driver. It handles:
    - I2C initialization and communication
    - Pulse width to duty cycle conversion
    - Safe shutdown procedures

    Example:
        driver = PCA9685Driver()
        driver.initialize()
        driver.set_servo_pulse(channel=0, pulse_us=1500)  # Center position
        driver.shutdown()
    """

    def __init__(
        self,
        i2c_address: int = PCA9685_ADDRESS,
        frequency: int = PWM_FREQUENCY,
        simulation_mode: bool = False
    ):
        """
        Initialize the PCA9685 driver.

        Args:
            i2c_address: I2C address of the PCA9685 (default: 0x40)
            frequency: PWM frequency in Hz (default: 50 for servos)
            simulation_mode: If True, no actual hardware communication
        """
        self.i2c_address = i2c_address
        self.frequency = frequency
        self.simulation_mode = simulation_mode
        self._initialized = False
        self._pca = None
        self._i2c = None
        self._channels: List[PWMChannel] = [PWMChannel(i) for i in range(16)]

        # Calculate timing constants
        self.period_us = 1_000_000 // frequency  # Period in microseconds
        self.us_per_count = self.period_us / PWM_RESOLUTION

    def initialize(self) -> bool:
        """
        Initialize the PCA9685 hardware.

        Returns:
            True if initialization successful, False otherwise

        Raises:
            RuntimeError: If I2C communication fails
        """
        if self._initialized:
            print("PCA9685 already initialized")
            return True

        if self.simulation_mode:
            print("[SIMULATION] PCA9685 initialized (no hardware)")
            self._initialized = True
            return True

        try:
            # Import here to allow simulation mode without hardware libraries
            import board
            import busio
            from adafruit_pca9685 import PCA9685

            # Initialize I2C bus
            self._i2c = busio.I2C(board.SCL, board.SDA)

            # Initialize PCA9685
            self._pca = PCA9685(self._i2c, address=self.i2c_address)
            self._pca.frequency = self.frequency

            self._initialized = True
            print(f"PCA9685 initialized at address 0x{self.i2c_address:02X}, frequency={self.frequency}Hz")
            return True

        except ImportError as e:
            raise RuntimeError(
                f"Required library not found: {e}\n"
                "Install with: pip install adafruit-circuitpython-pca9685 board busio"
            )
        except Exception as e:
            raise RuntimeError(
                f"Failed to initialize PCA9685: {e}\n"
                "Check I2C connection and ensure I2C is enabled (sudo raspi-config)"
            )

    def shutdown(self):
        """
        Safely shutdown the PCA9685.

        Disables all PWM outputs to prevent servo jitter.
        """
        if not self._initialized:
            return

        if self.simulation_mode:
            print("[SIMULATION] PCA9685 shutdown")
            self._initialized = False
            return

        try:
            # Disable all channels
            for i in range(16):
                self._pca.channels[i].duty_cycle = 0

            # Deinitialize
            self._pca.deinit()
            self._i2c.deinit()
            print("PCA9685 shutdown complete")

        except Exception as e:
            print(f"Warning during shutdown: {e}")

        finally:
            self._initialized = False
            self._pca = None
            self._i2c = None

    def pulse_us_to_duty_cycle(self, pulse_us: int) -> int:
        """
        Convert pulse width in microseconds to 16-bit duty cycle value.

        The PCA9685 library uses 16-bit values (0-65535) for duty cycle,
        which maps to the 12-bit PWM resolution internally.

        Args:
            pulse_us: Pulse width in microseconds (typically 500-2500)

        Returns:
            16-bit duty cycle value (0-65535)

        Example:
            - At 50Hz: period = 20000us
            - 1500us pulse = 1500/20000 * 65535 = 4915
        """
        # Clamp to valid range
        pulse_us = max(0, min(pulse_us, self.period_us))

        # Calculate duty cycle (16-bit value)
        duty_cycle = int((pulse_us / self.period_us) * 65535)
        return duty_cycle

    def set_servo_pulse(self, channel: int, pulse_us: int):
        """
        Set the pulse width for a servo on a specific channel.

        Args:
            channel: PCA9685 channel (0-15)
            pulse_us: Pulse width in microseconds (typically 500-2500 for servos)

        Raises:
            ValueError: If channel is out of range
            RuntimeError: If driver not initialized
        """
        if not self._initialized:
            raise RuntimeError("PCA9685 not initialized. Call initialize() first.")

        if not 0 <= channel <= 15:
            raise ValueError(f"Channel must be 0-15, got {channel}")

        # Update internal state
        self._channels[channel].pulse_width_us = pulse_us

        if self.simulation_mode:
            print(f"[SIMULATION] Channel {channel}: {pulse_us}us")
            return

        # Convert to duty cycle and set
        duty_cycle = self.pulse_us_to_duty_cycle(pulse_us)
        self._pca.channels[channel].duty_cycle = duty_cycle

    def set_servo_angle(
        self,
        channel: int,
        angle: float,
        min_pulse: int = 500,
        max_pulse: int = 2500,
        min_angle: float = 0,
        max_angle: float = 180,
        inverted: bool = False
    ):
        """
        Set servo position by angle with configurable pulse range.

        This is a convenience method that converts an angle to the
        appropriate pulse width based on servo characteristics.

        Args:
            channel: PCA9685 channel (0-15)
            angle: Desired angle in degrees
            min_pulse: Pulse width at minimum angle (microseconds)
            max_pulse: Pulse width at maximum angle (microseconds)
            min_angle: Angle corresponding to min_pulse
            max_angle: Angle corresponding to max_pulse
            inverted: If True, reverse the angle mapping

        Example:
            # Standard servo (0-180 degrees)
            driver.set_servo_angle(0, 90)  # Center position

            # Inverted servo
            driver.set_servo_angle(1, 90, inverted=True)
        """
        if inverted:
            angle = max_angle - (angle - min_angle)

        # Clamp angle to valid range
        angle = max(min_angle, min(max_angle, angle))

        # Linear interpolation: angle -> pulse
        angle_range = max_angle - min_angle
        pulse_range = max_pulse - min_pulse

        if angle_range == 0:
            pulse_us = min_pulse
        else:
            normalized = (angle - min_angle) / angle_range
            pulse_us = int(min_pulse + normalized * pulse_range)

        self.set_servo_pulse(channel, pulse_us)

    def disable_channel(self, channel: int):
        """
        Disable PWM output on a channel (servo goes limp).

        Args:
            channel: PCA9685 channel (0-15)
        """
        if not self._initialized:
            return

        self._channels[channel].enabled = False

        if self.simulation_mode:
            print(f"[SIMULATION] Channel {channel} disabled")
            return

        self._pca.channels[channel].duty_cycle = 0

    def enable_channel(self, channel: int, pulse_us: Optional[int] = None):
        """
        Re-enable a disabled channel.

        Args:
            channel: PCA9685 channel (0-15)
            pulse_us: Optional pulse width to set (uses last value if None)
        """
        self._channels[channel].enabled = True

        if pulse_us is None:
            pulse_us = self._channels[channel].pulse_width_us

        self.set_servo_pulse(channel, pulse_us)

    def get_channel_state(self, channel: int) -> PWMChannel:
        """
        Get the current state of a PWM channel.

        Args:
            channel: PCA9685 channel (0-15)

        Returns:
            PWMChannel object with current state
        """
        return self._channels[channel]

    def test_sweep(self, channel: int, min_pulse: int = 500, max_pulse: int = 2500, step: int = 50, delay: float = 0.05):
        """
        Test a servo by sweeping through its range.

        Useful for calibration and verifying servo operation.

        Args:
            channel: PCA9685 channel to test
            min_pulse: Starting pulse width (microseconds)
            max_pulse: Ending pulse width (microseconds)
            step: Pulse increment per step
            delay: Delay between steps (seconds)
        """
        print(f"Testing channel {channel}: {min_pulse}us -> {max_pulse}us")

        # Sweep up
        for pulse in range(min_pulse, max_pulse + 1, step):
            self.set_servo_pulse(channel, pulse)
            time.sleep(delay)

        # Sweep down
        for pulse in range(max_pulse, min_pulse - 1, -step):
            self.set_servo_pulse(channel, pulse)
            time.sleep(delay)

        # Center
        center = (min_pulse + max_pulse) // 2
        self.set_servo_pulse(channel, center)
        print(f"Channel {channel} test complete, centered at {center}us")


def test_driver():
    """
    Test function to verify PCA9685 driver operation.

    Run this directly to test hardware connectivity.
    """
    print("PCA9685 Driver Test")
    print("=" * 40)

    # Try hardware first, fall back to simulation
    try:
        driver = PCA9685Driver()
        driver.initialize()
    except RuntimeError as e:
        print(f"Hardware initialization failed: {e}")
        print("Falling back to simulation mode...")
        driver = PCA9685Driver(simulation_mode=True)
        driver.initialize()

    try:
        # Test basic operations
        print("\nSetting channel 0 to center (1500us)...")
        driver.set_servo_pulse(0, 1500)
        time.sleep(0.5)

        print("Setting channel 0 to 90 degrees...")
        driver.set_servo_angle(0, 90)
        time.sleep(0.5)

        print("\nDriver test complete!")

    finally:
        driver.shutdown()


if __name__ == "__main__":
    test_driver()
