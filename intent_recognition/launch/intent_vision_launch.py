from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='intent_recognition',
            executable='intent_server',
            name='intent_action_server',
            output='screen',
            parameters=[
                {'camera_topic': '/image_raw'} 
            ]
        )
    ])