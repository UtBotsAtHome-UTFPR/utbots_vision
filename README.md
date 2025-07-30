# utbots_vision

![Static Badge](https://img.shields.io/badge/ROS_Foxy-Not_Tested-red)
![Static Badge](https://img.shields.io/badge/ROS_Humble-Tested-green)
![Static Badge](https://img.shields.io/badge/ROS_Jazzy-Not_Tested-red)

This stack contains vision related packages, such as:

- **[mediapipe_track](https://github.com/UtBotsAtHome-UTFPR/mediapipe_track)** - person tracking, skeleton landmarks with 3D estimation
- **utbots_face_recognition** - face training and recognition
- **yolov8ros** - object detection

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

### Running

All packages listed above contain their own README files inside, some configuration is global but refer to them for specific documentation.

All systems for utbots_vision can be installed in a single venv and are generally compatible with the rest of utbots stack (except NLU). To gather how to setup the venv for a given package please refer to the README inside the package itself.

To call the venv sourcing is insufficient (ros2 is weird) and setup.cfg file needs to be altered to contain the path to the venv you've created. Shortcuts such as ~/ don't work so the path must be expanded (this file is in gitignore so changes to it are not pushed).