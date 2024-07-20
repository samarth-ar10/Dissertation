import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ur_description_package = get_package_share_directory('ur_description')
    ur_gazebo_package = get_package_share_directory('ur_gazebo')

    # Path to the URDF file
    urdf_file = os.path.join(ur_description_package, 'urdf', 'ur3.urdf')

    # Launch Gazebo with the UR3 robot
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ur_gazebo_package, 'launch', 'gazebo.launch.py'))
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'ur3', '-file', urdf_file],
            output='screen'
        )
    ])
