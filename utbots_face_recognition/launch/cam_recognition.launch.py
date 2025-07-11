from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch usb_cam node
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0', # for laptop cam, 2 for usb cam
                'image_height': 720,
                'image_width': 1280
            }]
        ),

        # Launch face recognition node
        Node(
            package='utbots_face_recognition',
            executable='recognize',
            name='face_recognition',
            output='screen',
            emulate_tty=True,
        ),
    ])