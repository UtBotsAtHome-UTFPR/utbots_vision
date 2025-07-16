from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yolov8_ros',
            executable='yolo_node',
            name='yolo_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'weights':'/ros2_ws/src/yolov8_ros/weights/best.pt',
                    'camera_topic':'/camera/camera/color/image_raw',
                    'device':'cuda',
                    'conf': 0.25,
                    'draw': True,
                    'target_category':'',
                    'debug':False,
                    'enable_synchronous_startup':True,
                  }
            ]
        ),

        #Node(
        #    package='usb_cam',
        #    executable='usb_cam_node_exe',
        #    name='usb_cam',
        #    parameters=[{
        #        'video_device': '/dev/video2',
        #        'image_width': 640,
        #        'image_height': 480,
        #        'framerate': 30.0,
        #        'pixel_format': 'yuyv'
        #    }],
        #    output='screen'
        #),
    ])   