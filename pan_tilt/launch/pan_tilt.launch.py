import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('pan_tilt')

    # Process the URDF/Xacro file (handles both plain URDF and Xacro variables)
    urdf_path = os.path.join(package_path, 'urdf', 'pan_tilt.urdf')
    robot_description = xacro.process_file(urdf_path).toxml()

    # Launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the ESP32 micro-ROS agent'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to launch RViz2 for visualization'
    )

    # micro-ROS agent for serial communication
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', LaunchConfiguration('serial_port')]
    )

    # Publishes URDF transforms based on /joint_states
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 30.0
        }]
    )

    # Pan-tilt action server
    pan_tilt_server = Node(
        package='pan_tilt',
        executable='pan_tilt_action_server',
        name='pan_tilt_action_server',
        output='screen'
    )

    # Optional RViz2 node (launches only when rviz:=true)
    rviz_config_path = os.path.join(package_path, 'rviz', 'pan_tilt.rviz')

    # Optional RViz2 node (launches pre-configured when rviz:=true)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    return LaunchDescription([
        serial_port_arg,
        rviz_arg,
        robot_state_publisher,
        pan_tilt_server,
        micro_ros_agent,
        rviz_node
    ])