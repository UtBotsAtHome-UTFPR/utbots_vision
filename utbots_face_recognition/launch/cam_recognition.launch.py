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
                'video_device': '/dev/video2',
                'framerate': 30.0,
                'io_method': 'mmap',
                'frame_id': 'camera',
                'pixel_format': 'mjpeg2rgb',
                'av_device_format': 'YUV422P',
                'image_width': 1280,
                'image_height': 720,
                'camera_name': 'test_camera',
                'camera_info_url': 'file:///home/laser/.ros/camera_info/default_cam.yaml',
                'brightness': -1,
                'contrast': -1,
                'saturation': -1,
                'sharpness': -1,
                'gain': -1,
                'auto_white_balance': True,
                'white_balance': 4000,
                'autoexposure': True,
                'exposure': 100,
                'autofocus': False,
                'focus': -1
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