from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'yolov8_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/action', ['action/YOLODetection.action']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='utbots',
    maintainer_email='utbots.home@gmail.com',
    description='YOLOv8 object detection package',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    extras_require={
    'test': ['pytest', 'other-test-deps'],
    },
    entry_points={
        'console_scripts': [
            'yolo_node = yolov8_ros.yolov8_node:main',
        ],
    },
)
