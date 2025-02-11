#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton
from PyQt5.QtCore import QThread, pyqtSignal, QObject
import tf2_ros
import tf2_geometry_msgs
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class GoalPosePublisher(Node):
    def __init__(self):
        super().__init__('goal_pose_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'ee_goal_pose', 10)
        self.feedback_sub_ = self.create_subscription(
            PoseStamped, 'planning_feedback', self.feedback_callback, 10)
        self.create_subscription(String, 'planning_state', self.update_planning_state, 10)
        self.create_subscription(String, 'execution_state', self.update_execution_state, 10)
        self.feedback = None

    def publish_goal_pose(self, position, orientation_rpy):
        msg = PoseStamped()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()

        # Set the desired position (in meters)
        msg.pose.position.x = float(position['x'])
        msg.pose.position.y = float(position['y'])
        msg.pose.position.z = float(position['z'])

        # Convert RPY to quaternion
        quaternion = quaternion_from_euler(
            float(orientation_rpy['roll']),
            float(orientation_rpy['pitch']),
            float(orientation_rpy['yaw'])  
        )

        # Set the desired orientation (as a quaternion)
        msg.pose.orientation.x = quaternion[0]
        msg.pose.orientation.y = quaternion[1]
        msg.pose.orientation.z = quaternion[2]
        msg.pose.orientation.w = quaternion[3]

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing goal pose: "%s"' % msg)

    def feedback_callback(self, msg):
        self.feedback = msg

    def update_planning_state(self, msg):
        self.get_logger().info(f'Planning State: {msg.data}')
        self.signals.planning_state_signal.emit(msg.data)

    def update_execution_state(self, msg):
        self.get_logger().info(f'Execution State: {msg.data}')
        self.signals.execution_state_signal.emit(msg.data)

class GoalPosePublisherSignals(QObject):
    planning_state_signal = pyqtSignal(str)
    execution_state_signal = pyqtSignal(str)

class GoalPoseGUI(QWidget):
    def __init__(self, node, signals):
        super().__init__()
        self.node = node
        self.signals = signals
        self.initUI()

        # Connect signals to slots
        self.signals.planning_state_signal.connect(self.update_planning_state_label)
        self.signals.execution_state_signal.connect(self.update_execution_state_label)

    def initUI(self):
        self.setWindowTitle('Set Goal Pose')

        layout = QVBoxLayout()

        # Planning and Execution State Labels
        self.planning_state_label = QLabel('Planning State: Not started')
        self.execution_state_label = QLabel('Execution State: Not started')
        layout.addWidget(self.planning_state_label)
        layout.addWidget(self.execution_state_label)

        # Position inputs
        pos_layout = QHBoxLayout()
        pos_layout.addWidget(QLabel('Position:'))
        self.pos_x = QLineEdit(self)
        self.pos_x.setPlaceholderText('x')
        pos_layout.addWidget(self.pos_x)
        self.pos_y = QLineEdit(self)
        self.pos_y.setPlaceholderText('y')
        pos_layout.addWidget(self.pos_y)
        self.pos_z = QLineEdit(self)
        self.pos_z.setPlaceholderText('z')
        pos_layout.addWidget(self.pos_z)
        layout.addLayout(pos_layout)

        # Orientation inputs (RPY)
        ori_layout = QHBoxLayout()
        ori_layout.addWidget(QLabel('Orientation (RPY):'))
        self.ori_roll = QLineEdit(self)
        self.ori_roll.setPlaceholderText('roll')
        ori_layout.addWidget(self.ori_roll)
        self.ori_pitch = QLineEdit(self)
        self.ori_pitch.setPlaceholderText('pitch')
        ori_layout.addWidget(self.ori_pitch)
        self.ori_yaw = QLineEdit(self)
        self.ori_yaw.setPlaceholderText('yaw')
        ori_layout.addWidget(self.ori_yaw)
        layout.addLayout(ori_layout)

        # Set Pose button
        self.set_pose_button = QPushButton('Set Pose', self)
        self.set_pose_button.clicked.connect(self.set_pose)
        layout.addWidget(self.set_pose_button)

        self.setLayout(layout)

    def set_pose(self):
        position = {
            'x': self.pos_x.text() if self.pos_x.text() else '0',
            'y': self.pos_y.text() if self.pos_y.text() else '0',
            'z': self.pos_z.text() if self.pos_z.text() else '0'
        }
        orientation_rpy = {
            'roll': self.ori_roll.text() if self.ori_roll.text() else '0',
            'pitch': self.ori_pitch.text() if self.ori_pitch.text() else '0',
            'yaw': self.ori_yaw.text() if self.ori_yaw.text() else '0'
        }
        self.node.publish_goal_pose(position, orientation_rpy)

    def update_planning_state_label(self, state):
        self.planning_state_label.setText(f'Planning State: {state}')
        if state == "Failed":
            self.planning_state_label.setStyleSheet("color: red;")
        elif state == "In progress":
            self.planning_state_label.setStyleSheet("color: orange;")
        elif state == "Successful":
            self.planning_state_label.setStyleSheet("color: blue;")

    def update_execution_state_label(self, state):
        self.execution_state_label.setText(f'Execution State: {state}')
        if state == "Failed":
            self.execution_state_label.setStyleSheet("color: red;")
        elif state == "In progress":
            self.execution_state_label.setStyleSheet("color: orange;")
        elif state == "Completed":
            self.execution_state_label.setStyleSheet("color: blue;")

class ROS2Thread(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        rclpy.spin(self.node)

def main(args=None):
    rclpy.init(args=args)
    goal_pose_publisher = GoalPosePublisher()
    signals = GoalPosePublisherSignals()
    goal_pose_publisher.signals = signals

    app = QApplication(sys.argv)
    gui = GoalPoseGUI(goal_pose_publisher, signals)

    ros2_thread = ROS2Thread(goal_pose_publisher)
    ros2_thread.start()

    gui.show()
    app.exec_()

    goal_pose_publisher.destroy_node()
    rclpy.shutdown()
    ros2_thread.wait()

if __name__ == '__main__':
    main()
