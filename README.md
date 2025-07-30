# utbots_vision

This stack contains vision related packages, such as:

- [mediapipe_track](https://github.com/UtBotsAtHome-UTFPR/mediapipe_track)
- utbots_face_recognition
- yolov8ros

And is dependant on:

- [utbots_dependencies](https://github.com/UtBotsAtHome-UTFPR/utbots_dependencies)

## Installation

```bash
cd <ros2_ws>/src
git clone --recurse-submodules https://github.com/UtBotsAtHome-UTFPR/utbots_vision.git
cd ../
```

### Dependencies

If utbots_dependencies not already installed:
```bash
cd <ros2_ws>/src
git clone https://github.com/UtBotsAtHome-UTFPR/utbots_dependencies.git
cd ../
```

This packages need the camera drivers for webcams and realsense. First, install:
```bash
sudo apt install ros-humble-usb-cam ros-humble-realsense2-*
```

TODO: Installation of librealsense2

See the dependencies installation procedure for each package accessing its README.md.

### Building

```bash
cd <ros2_ws>
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

### Updating

To push changes to the submodule packages ([mediapipe_track](https://github.com/UtBotsAtHome-UTFPR/mediapipe_track)), you should go to their repository path and perform a simple add, commit and push. After, you have to push the changes to the stack, going back to the stack repository path and doing the following command:

```bash
git submodule update --remote --merge
```
And then, perform a simple add, commit and push in the stack repository.

## Running

See the usage explanation accessing each package in each package README.md.