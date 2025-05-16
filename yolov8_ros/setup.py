from setuptools import setup

package_name = 'yolov8_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/action', ['action/YOLODetection.action'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='utbots',
    maintainer_email='utbots.home@gmail.com',
    description='YOLOv8 object detection package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolov8_node = yolov8_ros.yolov8_node:main',
        ],
    },
)
