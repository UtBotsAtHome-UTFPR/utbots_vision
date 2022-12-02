# apollo_vision
This stack contains vision related packages such as:
- darknet_ros
- img_converter
- mediapipe_track

## Installation
```git clone --recurse-submodules https://github.com/UtBot-UTFPR/apollo_vision.git```


## darknet_ros

### Dependencies

This software is built on the Robotic Operating System ([ROS]), which needs to be [installed](http://wiki.ros.org) first. Additionally, YOLO for ROS depends on following software:

- [OpenCV](http://opencv.org/) (computer vision library),
- [boost](http://www.boost.org/) (c++ library),
- [freenect_launch](https://github.com/ros-drivers/freenect_stack) (kinect camera package)

### Running

**Webcam**

``
roslaunch darknet_ros darknet_ros.launch
``

**Kinect V1**

``
roslaunch darknet_ros kinect.launch
``

Detailed information can be found in the [link](https://github.com/gustavo-fardo/darknet_ros)
