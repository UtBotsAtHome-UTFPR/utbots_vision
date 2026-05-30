# Intent Recognition Node (HRI Module)

This package is part of **utbots_vision**. It implements a module designed to identify human intentions through gesture analysis. It specifically detects a "wave" to lock onto an operator and calculates a pointing vector to identify target objects in 3D space.

## Features
- **Wave Detection (Lock-on):** Uses Fast Fourier Transform (FFT) on hand-oscillation frequency to distinguish human intent from random movement.
- **Pointing Vector Calculation:** Projects a vector from the operator's elbow to wrist using MediaPipe Pose or Yolo Pose.
- **Object Intersection:** Intersects the pointing vector with 3D Bounding Boxes provided by a YOLOv8 node.
- **State Machine:** Manages states (`IDLE`, `SEARCHING`, `LOCKED`) via a ROS 2 Action Server.

## Prerequisites
- **ROS 2 Humble**
- **Python 3.10+**
- **Virtual Environment:** It is highly recommended to use a venv (e.g., `.venvRos2`) to manage specific dependency versions.

### Dependencies
Install the required Python packages:
```bash
pip install numpy<2.0.0 opencv-python mediapipe ultralytics
``` 

# Note:
 NumPy must be version < 2.0.0 to maintain compatibility with cv_bridge in ROS 2 Humble.
# Installation & Build
 - Clone this package into your workspace's src folder.
 - Build the package:
```bash
  colcon build --packages-select intent_recognition
```

## Usage 
1. Launch the Node
```bash
source install/setup.bash
ros2 launch intent_recognition intent_vision_launch.py
```
2. Calling the Action
- The node waits for a goal from the Behavior Tree or terminal:
```bash
ros2 action send_goal /recognize_pointing_action utbots_actions/action/RecognizePointing "{pose_model: 'MEDIAPIPE_POSE'}"
```
## Testing Guide (Webcam & YOLO Integration)
To test the full pipeline (MediaPipe + YOLO) on a laptop without the robot's RealSense camera, follow these steps to ensure all nodes are "listening" to the same image stream.
### Step 1:
Run the Camera NodeIf using a standard laptop webcam, use the usb_cam package:
```bash
ros2 run usb_cam usb_cam_node_exe
```
Default topic: /image_raw
### Step 2: Run the YOLO Node (with Remapping)
The YOLO node usually expects the robot's camera topic. Remap it to your webcam topic so it generates bounding boxes for your environment:
```bash
ros2 run yolov8_ros yolo_node --ros-args -p camera_topic:=/image_raw
```
### Step 3: Run the Intent Node (with Parameters)
Ensure the Intent Node is also looking at the webcam. You can change this in the intent_vision_launch.py file or via command line:
```bash
ros2 launch intent_recognition intent_vision_launch.py camera_topic:=/image_raw
```
### Alternative: Running via CLI (`ros2 run`)
If you prefer to run the node isolated without the launch file, you can use `ros2 run`. You must pass the camera topic parameter via command line arguments using `--ros-args -p`:

```bash
source install/setup.bash
ros2 run intent_recognition intent_server --ros-args -p camera_topic:=/image_raw
```

### Step 4: RViz VisualizationAdd an Image display.
Set the topic to /intent_debug_image.
#### Important: 
Change the Reliability Policy to Best Effort in the Image topic settings to see the stream.
#### Parameters
Parameter: camera_topic           
Type:  string                        
Default: camera/camera/color/image_raw,              
Description: The input image topic for processing.

#### Common Issues:
- If there's no Image in RViz: Ensure the Action is active (SEARCHING or LOCKED) and the QoS is set to Best Effort.
- ModuleNotFoundError: Ensure the sed fix was applied to the install script after colcon build.
- Wrong Arm Detected: Webcams often have a "Mirror/Selfie" effect. The node expects an "Observer" perspective. You may need to use cv2.flip(frame, 1) for local testing.
"""file_path = "/mnt/data/README.md"with open(file_path, "w") as f:f.write(readme_content)print(f"File saved at: {file_path}")

