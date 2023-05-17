# vision_tools

## Building
```
cd catkin_ws/src
git clone --recurse-submodules https://github.com/UtBot-UTFPR/img_converter.git
cd ..
catkin_make
```

## Nodes

Before running **vision_tools** nodes, make sure a *roscore* process is being run and there is a source node publishing *sensor_msgs::Image* messages to the */camera/rgb/image_raw* topic eg. *freenect* publishing kinect rgb frames.

- ### img_converter
  Changes the encoding of a sensor_msgs::Image to an OpenCV acceptable RGB8 format
  ```
  rosrun img_converter img_converter
  ```
  The node publishes converted image in */image_converter/output_video*.

- ### detected_person_manager
- ### detected_obj_manager
