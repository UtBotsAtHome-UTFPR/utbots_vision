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
  rosrun vision_tools img_converter
  ```
  The node publishes converted image in */image_converter/output_video*.

- ### detected_person_manager
  Converts every instance of a "person" class bounding box to an Object message that contains their parent image, their roi (in a sensor_msgs/RegionOfInterest message), and the cropped boudning box image. Publishes then the detected persons as an ObjectArray.
  
  Currently also publishes the first detected person Object message and image message.
  ```
  rosrun vision_tools detected_person_manager.py
  ```
  Publishes the person array in */utbots/vision/person/personArray*.
  
  Publishes the selected person's Object message in */utbots/vision/person/selected/object*.
  
  Publishes the selected person's image in */utbots/vision/person/selected/image*.
  
- ### detected_obj_manager
  Converts every instance of a class that is not a "person" to bounding box to an Object message that contains their parent image, their roi (in a sensor_msgs/RegionOfInterest message), and the cropped boudning box image. Publishes then the detected persons as an ObjectArray.
  
  Currently also publishes the first detected object Object message and image message.
  ```
  rosrun vision_tools detected_obj_manager.py
  ```
  Publishes the object array in */utbots/vision/object/objectArray*.
  
  Publishes the selected object's Object message in */utbots/vision/object/selected/object*.
  
  Publishes the selected object's image in */utbots/vision/object/selected/image*.
 
- ### extract_3d_centroid
  Estimates the 3d point for a given object in Objecy format, subscribed in */utbots/vision/object/selected/object*, using depth image and the object's roi. Extracts the distance from the object to the camera by three possible methods: median, outlier removal + mean and spiral filtering; the median is the default method. Transforms the distance to a 3d point and publishes as a PointStamped.
  ```
  rosrun vision_tools extract_3d_centroid.py
  ```
  Publishes the 3d point in */utbots/vision/object/selected/objectPoint*
