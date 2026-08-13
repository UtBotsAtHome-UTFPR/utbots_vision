
# Lidar Tracking Node

This package implements a 2D LiDAR-based tracking system for ROS 2. It converts LiDAR range measurements into a binary image and applies classical computer vision techniques to detect small objects such as human legs or cylindrical obstacles. Detected objects are then tracked over time using the SORT algorithm.

## Architecture & Perception Pipeline

The node converts the LiDAR scan into a 2D binary image so that object detection can be approached as a computer vision problem.

The current pipeline is:

1. **Rasterization (Meters to Pixels)**

   Subscribes to the configured `LaserScan` topic and projects the polar measurements (range and angle) into a 2D Cartesian image.

   The current configuration uses:
   - Image size: `240 x 240` pixels
   - Resolution: `0.05 m/pixel`
   - Robot/sensor center: `(120, 120)`

2. **Pre-Processing (Morphological Closing)**

   Applies morphological closing using `cv2.MORPH_CLOSE` to connect small gaps between nearby LiDAR points and produce more stable contours.

3. **Detection & Size Filtering**

   Uses `cv2.findContours` to identify connected regions in the binary image.

   Bounding boxes are filtered according to configurable width and height limits in order to reject small sensor noise and large objects that do not match the expected target geometry.

4. **Tracking (SORT)**

   Uses the SORT (Simple Online and Realtime Tracking) algorithm to maintain object identities across consecutive LiDAR scans.

   Before being passed to SORT, bounding boxes are artificially expanded using a configurable padding value. This makes the IoU association more tolerant to small changes in the detected bounding boxes.

5. **De-Rasterization**

   The center of each tracked bounding box is converted from pixel coordinates back into metric coordinates relative to the LiDAR frame.

6. **TF2 Broadcasting**

   Each tracked object is currently broadcast as a dynamic TF2 frame using the LiDAR frame as its parent.

   Example:

   ```text
   base_scan
       ├── lidar_leg_1
       ├── lidar_leg_2
       └── lidar_leg_3
   ```

The tracked object frames are intended primarily for spatial integration and debugging during development.

## Action Server

The node is currently being adapted to expose the `TrackPerson` ROS 2 Action interface.

The Action Server is intended to allow a higher-level orchestration component to:

1. Provide an initial indication of the target position or direction.
2. Select the corresponding LiDAR cluster.
3. Lock onto the selected tracker.
4. Receive continuous tracking feedback.
5. Cancel tracking when the task is finished or the target can no longer be followed.

The exact representation of the initial target information is still being evaluated as part of the integration with the team's existing vision pipeline.

## ROS Interfaces

### Subscribes to

- `/scan` (`sensor_msgs/LaserScan`)

The LiDAR topic is configurable through the `lidar_topic` ROS parameter.

### Actions

- `track_person` (`utbots_actions/TrackPerson`)


### TF2

The node broadcasts dynamic frames for the currently tracked LiDAR objects:

```
<lidar_frame> → lidar_leg_<ID>
```

The parent frame is obtained directly from `LaserScan.header.frame_id`.

## Configuration Parameters

The main parameters currently used by the node are:

|Parameter|Current value|Description|
|---|---|---|
|`lidar_topic`|`/scan`|LiDAR input topic|
|`img_size`|`240`|Rasterized image size|
|`resolution`|`0.05`|Meters per pixel|
|`kernel_size`|`3`|Morphological closing kernel|
|`min_width`|`2`|Minimum contour width|
|`max_width`|`8`|Maximum contour width|
|`min_height`|`2`|Minimum contour height|
|`max_height`|`8`|Maximum contour height|
|`padding`|`10`|Bounding box expansion before SORT|
|`max_age`|`5`|SORT tracker maximum age|
|`min_hits`|`1`|SORT minimum hits|
|`iou_threshold`|`0.2`|SORT IoU association threshold|

These values are currently intended to provide a baseline for simulation testing and may be adjusted as the system is evaluated in more realistic environments.

## Requirements

- ROS 2 Humble
- Python 3.x
- OpenCV (`cv2`)
- NumPy
- FilterPy

The SORT implementation requires FilterPy:

```
pip install filterpy
```

## Running and testing:

Open separate terminals for the simulation and tracking node.

### 1. Launch the Simulation

Start the TurtleBot3 simulation in the default world:

```
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### 2. Run the Tracking Node

Build and source the workspace:
```
colcon build --select-packages lidar-tracking
```


```
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Then run:

```
ros2 run lidar_tracking tracking_node
```

An OpenCV window will display the rasterized LiDAR data and detected/tracked objects.

- Green boxes: accepted LiDAR contours
- Red boxes: rejected contours
- Yellow boxes: SORT trackers
- Blue text: tracker IDs





