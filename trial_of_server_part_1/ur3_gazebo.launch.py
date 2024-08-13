from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'ur3', '-file', '/home/ros2_ws/src/Universal_Robots_ROS2_Description/urdf/ur3_robot.urdf'],
            output='screen'),
    ])
