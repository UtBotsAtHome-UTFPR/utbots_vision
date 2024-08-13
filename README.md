# utbots_vision

This stack contains vision related packages, such as:

- [mediapipe_track](https://github.com/UtBotsAtHome-UTFPR/mediapipe_track)
- [utbots_face_recognition](https://github.com/UtBotsAtHome-UTFPR/utbots_face_recognition)
- vision_tools
- vision_msgs
- yolov8ros

And is dependant on:

- [utbots_dependencies](https://github.com/UtBotsAtHome-UTFPR/utbots_dependencies)

## Installation

```bash
cd ~/catkin_ws/src
git clone https://github.com/UtBotsAtHome-UTFPR/utbots_dependencies.git
git clone --recurse-submodules https://github.com/UtBotsAtHome-UTFPR/utbots_vision.git
cd ../
```

### Dependencies

See the dependencies installation procedure for each package accessing its README.md file or, in some cases, below, in [Packages Description](#packages-description).

### Building

```bash
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
```
### Running

See the usage explanation accessing each package in each package README.md or, in some cases, below, in [Packages Description](#packages-description).

## Packages Description

### freenect_launch
