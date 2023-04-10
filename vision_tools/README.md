# img_converter
Changes the encoding of a sensor_msgs::Image to an opencv acceptable RGB8 format

## Building
```
cd catkin_ws/src
git clone --recurse-submodules https://github.com/UtBot-UTFPR/img_converter.git
cd ..
catkin_make
```

## Running

Before running **img_converter**, make sure a *roscore* process is being run and there is a source node publishing *sensor_msgs::Image* messages to the */camera/rgb/image_raw* topic eg. *freenect* publishing kinect rgb frames.
```
rosrun img_converter img_converter
```
The node publishes converted image in */image_converter/output_video*.
