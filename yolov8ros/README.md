# yolov8ros
- ROS package that implements [YOLOv8](https://github.com/ultralytics/ultralytics) 
- Synchronous detection is Enabled/Disabled with a ROS service (disabled by standard)
- Assynchronous detection is possible with actions. Optional target category and success result for finding specific category bounding boxes.

## Installation

### Dependencies

This package must be used alongside the [utbots_dependencies](https://github.com/UtBotsAtHome-UTFPR/utbots_dependencies) as it uses some of the message and action definitions. You can do this with:

```bash
cd ~/catkin_ws/src/
git clone https://github.com/UtBotsAtHome-UTFPR/utbots_dependencies.git
cd ..
catkin_make
```

The code runs on Python 3.8. Install YOLO and other Python requirements:

```bash
roscd yolov8ros
pip install -r requirements.txt
```

**OBS:** Recommended Python virtual environment


## Running

First, initialize ROS (if not already):
```bash
roscore
```
Then, run the node:
```bash
rosrun yolov8ros yolov8node.py
```
### Getting detections
The enable control can be made inside a script with a ServiceClient. To enable/disable in the terminal:
```bash
rosservice call /utbots/vision/enable_detection <"False" or "True">                          
```
To trigger a detection response with an action in the terminal. Optionally, it is possible to send a category filter filling `<optional category name>` optional goal:
```bash
rostopic pub /YOLO_detection/goal utbots_actions/YOLODetectionActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  TargetCategory:
    data: '<optional category name>'
  Image:
    header:
      seq: 0
      stamp: {secs: 0, nsecs: 0}
      frame_id: ''
    height: 0
    width: 0
    encoding: ''
    is_bigendian: 0
    step: 0
    data: []"
```
**OBS**: Images are not easy to sent in the terminal, so the optional Image goal should be used only when using ActionClients inside scripts.

To see the action result in the terminal:
```bash
rostopic echo /YOLO_detection/result
```
**OBS**: The Image result isa huge matrix normally, so you may not be able to see the whole result msg