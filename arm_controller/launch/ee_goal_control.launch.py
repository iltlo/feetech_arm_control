from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arm_controller',
            executable='ee_goal_control',
            name='ee_goal_control',
            output='screen',
            parameters=[]
        ),
        Node(
            package='arm_controller',
            executable='gui_goal_pose.py',
            name='gui_goal_pose',
            output='screen',
            parameters=[]
        )
    ])
