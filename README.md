# utbots_vision

This stack contains vision related packages, such as:

- [mediapipe_track](https://github.com/UtBotsAtHome-UTFPR/mediapipe_track)
- [utbots_face_recognition](https://github.com/UtBotsAtHome-UTFPR/utbots_face_recognition)
- vision_tools
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
# Install ROS dependencies
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Updating

To push changes to the submodule packages ([mediapipe_track](https://github.com/UtBotsAtHome-UTFPR/mediapipe_track), ~~[utbots_face_recognition](https://github.com/UtBotsAtHome-UTFPR/utbots_face_recognition)~~) you should go to their repository path and perform a simple add, commit and push. After, you have to push the changes to the stack, going back to the stack repository path and doing the following command:

```bash
git submodule update --remote --merge
```
And then, perform a simple add, commit and push in the stack repository.

## Running

See the usage explanation accessing each package in each package README.md or, in some cases, below, in [Packages Description](#packages-description).

## Packages Description

### freenect_stack
- Kinect V1 Driver for ROS1. Outputs RGB sensor_msgs/Image and depth sensor_msgs/Image messages.
- Most used topics are `camera/rgb/image_raw`, `camera/rgb/image_color` and `camera/depth_registered/image_raw`, but several other are available.

#### Installation

Installed already with utbots_vision clone and pull --recurse-submodules.
Can be installed separately with:

```bash
sudo apt update
sudo apt install ros-<distro>-freenect-launch
```

##### Dependencies

You must install the freenect library beforehand:

```bash
sudo apt update
sudo apt install libfreenect-dev
```

#### Running

To launch the driver and its topics, run:

```bash
roslaunch freenect_launch freenect.launch
```

### utbots_face_recognition

Loading everything for the first time may take several seconds and subsequent launches make take some seconds if the computer is bad, have patience. For a significant number of images use cuda

#### Dependencies

This package is dependant on a series on problematic libraries combinations

Select a path (full path, no shortcuts) for the venv and set setup.cfg path to point to it.

```bash
pip uninstall cv_bridge # just in case so it can pull from sudo apt install ros-humble-cv-bridge
pip install deepface
pip install tf-keras
pip install --no-cache-dir --upgrade --ignore-installed opencv-python
pip install numpy==1.26.4 # This will say sucessfully installed numpy 2.2.1. Don't ask me but it works
```

#### Running

```bash
ros2 run usb_cam usb_cam_node_exe
ros2 run utbots_face_recognition recognize
```
