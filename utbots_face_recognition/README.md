# utbots_face_recognition

Loading everything for the first time may take several seconds and subsequent launches make take some seconds if the computer is bad, have patience.

### Dependencies

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

This package is dependant on a series of problematic libraries combinations

```bash
pip uninstall cv_bridge # just in case so it can pull from sudo apt install ros-humble-cv-bridge
pip install deepface
pip install tf-keras
pip install --no-cache-dir --upgrade --ignore-installed opencv-python
pip install numpy==1.26.4 # This will say sucessfully installed numpy 2.2.1. Don't ask me but it works
```

## Build
```bash
cd <ros2_ws>
colcon build --packages-select utbots_face_recognition utbots_actions utbots_srvs utbots_msgs \
--allow-overriding utbots_msgs utbots_actions utbots_srvs \
&& source install/setup.bash
```

## Running

```bash
ros2 launch utbots_face_recognition cam_recognition.launch.py # This launches the system with usb_cam 
ros2 launch utbots_face_recognition recognition.launch.py # No camera is launched. Remember to remap the topic
```

## Action interfaces

There are two action interfaces present in this package, one for taking pictures for a new face and one for recognizing people in an image.

### New face

The action interface contains number of pictures to be taken, as well as the name of the operator. For feedback it returns the number of pictures taken as well as the last picture, so that the software calling it can have an idea of completion or display the cropped images of the face found.

```bash
# Goal
std_msgs/Int32 n_pictures
std_msgs/String name
---
# Feedback
std_msgs/Int32 pics_taken
sensor_msgs/Image image 
```

### Recognition

The action interfaces contains a goal with an optional field. If an image is provided recognition shall be performed on said image, otherwise, the callback for the image topic is called and recognition is performed in what the camera currently sees. The result contains a marked image with bounding boxes showing the names of the people recognized around their heads, additionally, a bounding box list where each instance contains the coordinates, the class (Person), the id (name) and the pixel coordinates of the face from the person being identified.

```bash
# Goal
sensor_msgs/Image image
---
# Result
utbots_msgs/BoundingBox[] people
sensor_msgs/Image image
```

## Testing and debugging

This package was build to be easy to debug across all levels of development, here they shall be explained.

### Library level

The library being used is deepface, all functions related to it are present in utbots_face_recognition/modules, each file can be run individually with python3 ... and contain a main file consisting of a simple script to test functionality.

### Action level

As mentioned before, there are 2 action interfaces, both launched within the recognize node. For testing the actions one can simply **ros2 run utbots_face_recognition action_tester** while the action server is already running. To change which of the actions is present simply change which **future = action_client.send_ ...** is being called directly in the source code.

### Task level

In utbots_tasks there are state machines in basic_face which have implemented state machines to test new face and recognize, as well as submachine for performing the task, while only keeping the camera on for the important bits (saving resources). All modules can be tested by running basic_face and uncommenting what the user wants to test.