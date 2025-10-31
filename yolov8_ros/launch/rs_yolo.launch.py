from launch import LaunchDescription
from launch_ros.actions import Node
import os

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    realsense_launch_path = os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch')

    return LaunchDescription([
        Node(
            package='yolov8_ros',
            executable='yolo_node',
            name='yolo_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'weights': '/home/segalle/ros2_ws/src/treinamentoYoloCbr/weights/pesosCBR2025.pt', #/ros2_ws/src/yolov8_ros/weights/best.pt',
                    'camera_topic':'/camera/camera/color/image_raw',
                    'device':'cuda',
                    'conf': 0.25,
                    'draw': True,
                    'target_category':'',
                    'segmentation': False,
                    'debug':False,
                    'enable_synchronous_startup':False,
                  }
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(realsense_launch_path, 'rs_launch.py')),
            #launch_arguments={}
        ),
    ])   