from setuptools import find_packages, setup

package_name = 'utbots_face_recognition'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'rclpy_action', 'utbots_actions', 'utbots_msgs', 'utbots_dependencies'],
    zip_safe=True,
    maintainer='david',
    maintainer_email='davidsegalle@alunos.utfpr.edu.br',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'recognize = utbots_face_recognition.recognize:main',
        ],
    },
)
