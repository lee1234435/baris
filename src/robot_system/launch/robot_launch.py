from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_system',
            executable='RobotControllerNode',
            name='RobotControllerNode'
        ),
        Node(
            package='robot_system',
            executable='RobotSystemNode',
            name='RobotSystemNode'
        ),
        Node(
            package='robot_system',
            executable='DispenserNode',
            name='DispenserNode'
        )
    ])