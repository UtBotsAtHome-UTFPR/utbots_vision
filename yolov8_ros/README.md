# yolov8_ros
- ROS package that implements YOLOv8
- Synchronous detection is Enabled/Disabled with a ROS service (disabled by standard)
- Assynchronous detection is possible with actions. Optional target category and success result for finding specific category bounding boxes
- Assynchronous batch detection is possible with actions, returning the detected objects with presence in multiple frames.
- Tested in Ubuntu 22 with ROS Humble

## Installation

### Dependencies and Build

To avoid conflicts between package dependencies, we use virtual environments. Change the virtuelenv path in the `executable` field in `setup.cfg`. *Not the ideal solution, but the current one while we don't use Docker*.

If you haven't installed `virtualenv`:
```bash
pip3 install virtualenv
```

Create and activate env:
```bash
python -m virtualenv <env_path>
source <env_path>/bin/activate
```

Install requirements and build:
```bash
pip3 install -r requirements.txt
cd ../..
colcon build --packages-select yolov8_ros utbots_actions utbots_srvs utbots_msgs \
--allow-overriding utbots_msgs utbots_actions utbots_srvs \
&& source install/setup.bash
```

## Running
To run the Mediapipe pose estimation node:

```bash
ros2 run yolov8_ros yolo_node
```

With launchfiles you can specify the parameter values using any of the arguments in Command Line or other launchfiles (for instance, disabling *draw* could save processing usage):

```bash
'weights':
    Path to the YOLOv8 model weights file.
    (default: '')

'camera_topic':
    The input ROS topic for RGB images.
    (default: '')

'device':
    Device to run inference on. Options: 'cuda' or 'cpu'.
    Defaults to 'cuda' if available.
    (default: 'cuda')

'conf':
    Confidence threshold for filtering detections.
    (default: 0.25)

'draw':
    Whether to draw bounding boxes on the output image.
    (default: true)

'target_category':
    Target class name to filter detections.
    If empty, all classes are allowed.
    (default: '')
```
