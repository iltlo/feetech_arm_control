import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from arm_hardware_interface.motors.feetech import FeetechMotorsBus
import time
from serial.serialutil import SerialException

class HardwareInterface(Node):
    def __init__(self):
        super().__init__('hardware_interface')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)
        self.publisher_ = self.create_publisher(JointState, 'real_joint_states', 10)
        self.timer = self.create_timer(0.02, self.publish_real_joint_states)  # 0.02 -> 50 Hz

        self.motors_bus = FeetechMotorsBus(
            port="/dev/ttyACM0",
            motors={},
        )
        self.motors_bus.connect()

        # Define the mapping between joint names and motor IDs
        self.joint_to_motor_id = {
            "Joint_1": 1,
            "Joint_2": 2,
            "Joint_3": 3,
            "Joint_4": 4,
            "Joint_5": 5,
            "Joint_6": 6,
            "Joint_Gripper": 7,
        }

        # Initialize motors with the correct IDs and model
        self.motor_names = list(self.joint_to_motor_id.keys())
        self.motors_bus.motors = {name: (self.joint_to_motor_id[name], "sts3215") for name in self.motor_names}

        self.configure_motors()
        self.set_torque(True)

    def configure_motors(self):
        for name in self.motor_names:
            self.motors_bus.write("Goal_Speed", 100, name)  # Set speed to 100
            self.motors_bus.write("Acceleration", 50, name)  # Set acceleration to 50

    def set_torque(self, enable):
        torque_value = 1 if enable else 0
        for name in self.motor_names:
            self.motors_bus.write("Torque_Enable", torque_value, name)
        self.get_logger().info(f"Torque {'enabled' if enable else 'disabled'} for all motors.")

    def joint_state_callback(self, msg):
        motor_ids = []
        motor_values = []
        motor_models = []

        for name, position in zip(msg.name, msg.position):
            if name in self.motor_names:
                motor_id = self.joint_to_motor_id[name]
                motor_value = int((-position + 3.14) * (4095 / (2 * 3.14)))  # Convert radians to motor value
                motor_ids.append(motor_id)
                motor_values.append(motor_value)
                motor_models.append("sts3215")

        if motor_ids:
            self.motors_bus.write_with_motor_ids(motor_models, motor_ids, "Goal_Position", motor_values)
            # self.get_logger().info(f"Joint positions: {list(zip(msg.name, motor_values))}")

    def publish_real_joint_states(self):
        motor_ids = list(self.joint_to_motor_id.values())
        motor_models = ["sts3215"] * len(motor_ids)
        try:
            positions = self.motors_bus.read_with_motor_ids(motor_models, motor_ids, "Present_Position")
        except (ConnectionError, SerialException) as e:
            self.get_logger().error(f"Connection error: {e}")
            self.get_logger().info("Attempting to reconnect...")
            while True:
                try:
                    self.motors_bus.reconnect()
                    self.get_logger().info("Reconnected to motors.")
                    return
                except Exception as e:
                    self.get_logger().error(f"Reconnection failed: {e}")
                    time.sleep(1)  # Wait for 1 second before retrying

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.motor_names
        joint_state_msg.position = [((-pos / 4095.0) * (2 * 3.14) - 3.14) for pos in positions]  # Convert motor value to radians

        self.publisher_.publish(joint_state_msg)
        # self.get_logger().info(f"Published real joint states: {positions}")
        self.get_logger().info(f"Published real joint states (radians): {joint_state_msg.position}")

def main(args=None):
    rclpy.init(args=args)
    hardware_interface = HardwareInterface()
    rclpy.spin(hardware_interface)
    hardware_interface.motors_bus.disconnect()
    hardware_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
