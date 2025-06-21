# yolov8_ros
- ROS package that implements YOLOv8
- Synchronous detection is Enabled/Disabled with a ROS service (disabled by standard)
- Assynchronous detection is possible with actions. Optional target category and success result for finding specific category bounding boxes
- Assynchronous batch detection is possible with actions, returning the detected objects with presence in multiple frames.
- Tested in Ubuntu 22 with ROS Humble

## Installation

### Dependencies and Build

If you haven't installed `virtualenv`:
```bash
pip3 install virtualenv
```

In the package directory:
```bash
python -m virtualenv .venv
source .venv/bin/activate
pip3 install -r requirements.txt
cd ../..
colcon build --symlink-install
```

Change the username and workspace name if needed in `setup.cfg`. *Not the ideal solution, but the current one while we don't use Docker*.

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
