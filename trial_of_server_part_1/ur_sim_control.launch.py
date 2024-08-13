import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    ur_gazebo_dir = get_package_share_directory('ur_simulation_gazebo')
    ur_description_dir = get_package_share_directory('ur_description')

    # RViz
    rviz_config_file = os.path.join(ur_description_dir, 'rviz', 'view_robot.rviz')

    # Launch configuration variables specific to simulation
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    ur_type = LaunchConfiguration('ur_type', default='ur3')
    prefix = LaunchConfiguration('prefix', default='')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware', default='false')
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands', default='false')

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    declare_ur_type_cmd = DeclareLaunchArgument(
        'ur_type',
        default_value='ur3',
        description='Type/series of used UR robot.')
    declare_prefix_cmd = DeclareLaunchArgument(
        'prefix',
        default_value='""',
        description='Prefix of the joint names, useful for multi-robot setup. If changed than also joint names in the controllers\' configuration have to be updated.')
    declare_use_fake_hardware_cmd = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='Use fake hardware mirroring command to its state.')
    declare_fake_sensor_commands_cmd = DeclareLaunchArgument(
        'fake_sensor_commands',
        default_value='false',
        description='Enable fake command interfaces for sensors used for simple simulations.Used only if use_fake_hardware parameter is true.')

    # Specify the actions
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_gazebo_dir, 'launch', 'gazebo.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # Start the robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[os.path.join(ur_description_dir, 'urdf', 'ur.urdf.xacro'),
                   'prefix:=' + prefix,
                   'name:=' + ur_type]
    )

    # Start RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_sim_time)
    )

    # Start the controllers
    load_controllers = [
        'joint_state_broadcaster',
        'joint_trajectory_controller',
    ]

    load_controllers_cmds = []
    for controller in load_controllers:
        load_controllers_cmds += [
            Node(
                package='controller_manager',
                executable='spawner.py',
                arguments=[controller],
                output='screen'),
        ]

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the declared launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_ur_type_cmd)
    ld.add_action(declare_prefix_cmd)
    ld.add_action(declare_use_fake_hardware_cmd)
    ld.add_action(declare_fake_sensor_commands_cmd)

    # Add the actions
    ld.add_action(start_gazebo)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)

    for load_controllers_cmd in load_controllers_cmds:
        ld.add_action(load_controllers_cmd)

    return ld
