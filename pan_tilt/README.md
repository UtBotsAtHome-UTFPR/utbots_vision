# pan_tilt

This package contains ROS 2 interfaces and control nodes for a pan-tilt mechanism, providing:

- **PanSweep**: continuously sweeps the pan servo between minimum and maximum angles.
- **TiltSweep**: continuously sweeps the tilt servo between minimum and maximum angles.
- **ScanSequence**: executes a predefined scanning sequence combining pan and tilt movements.
- **MoveToAngles**: moves the pan-tilt mechanism to a specified angular position.
- **URDF integration**: publishes joint states to `/joint_states` for robot state publishing and TF updates.
- **Micro-ROS integration**: publishes servo commands through `/servo_angles` for hardware control. Firmware code available at `./firmware/`.

## Installation

Clone this repository into your ROS 2 workspace:

```bash
cd <ros2_ws>/src
git clone <repository_url>
```

### Dependencies

Make sure ROS 2 Humble is installed and sourced:

```bash
source /opt/ros/humble/setup.bash
```

Install the required dependencies:

```bash
sudo apt update
sudo apt install ros-humble-rosidl-default-generators \
                 ros-humble-rosidl-default-runtime \
                 ros-humble-ament-cmake \
                 ros-humble-ament-cmake-python \
                 python3-empy
```

Install dependencies using `rosdep`:

```bash
cd <ros2_ws>
sudo rosdep init # Only if rosdep has not been initialized previously
rosdep update

rosdep install --from-paths src --ignore-src -r -y
```

> **IMPORTANT!**  
> If you use a Python virtual environment, make sure the Python interpreter used by ROS 2 can access the system ROS 2 Python dependencies, particularly `empy` (`import em`). Otherwise, interface generation may fail with:
> ```text
> ModuleNotFoundError: No module named 'em'
> ```
> For ROS 2 Humble, using the system Python interpreter (`/usr/bin/python3`) is recommended when building ROS interfaces.

Install the `micro_ros_agent` package in your workspace (or a separate workspace):

```bash
cd <ros2_ws>/src
git clone -b humble https://github.com
cd <ros2_ws>
rosdep install --from-paths src --ignore-src -y
colcon build --packages-up-to micro_ros_setup
source install/setup.bash

ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/setup.bash
```

*(Alternatively, you can install it via Snap with `sudo snap install micro-ros-agent`, though serial permissions may require extra snap interface connections).*

## Building

Build the package using:

```bash
cd <ros2_ws>

source /opt/ros/humble/setup.bash

colcon build --packages-up-to pan_tilt --symlink-install
```

Source the workspace:

```bash
source install/setup.bash
```

## Running

### Setup Micro-ROS Firmware

#### Flash Firmware

**Example**: PlatformIO
1) Install the PlatformIO IDE extension for VS Code or the PlatformIO Core CLI.
2) Plug the ESP32 into your computer via USB. Identify the port it connects to (e.g., /dev/ttyUSB0)
3) Navigate to the directory containing your platformio.ini and source code, then build and upload the firmware. With the PlatformIO CLI:

```bash
cd <ros2_ws>/src/pan_tilt/firmware
pio run -t upload
```

#### Configure Serial Port Permissions

Add your user to the `dialout` group to allow read/write access to USB serial devices (log out and log back in for changes to take effect):

```bash
sudo usermod -a -G dialout \$USER
```

Connect your ESP32 via USB and check which port was assigned:

```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null

### Launch Pan-Tilt Mechanism

To launch the pan-tilt mechanism, including its action server and robot state publisher and micro_ros node (required connected ESP to the specified port):

```bash
ros2 launch pan_tilt pan_tilt.launch.py serial_port:=/dev/ttyUSB0
```

The action server provides four actions for controlling the mechanism.

### Available Actions

The following actions are available:

| Action | Interface | Description |
| :--- | :--- | :--- |
| `PanSweep` | `pan_tilt/action/PanSweep` | Continuously sweeps the pan axis. |
| `TiltSweep` | `pan_tilt/action/TiltSweep` | Continuously sweeps the tilt axis. |
| `ScanSequence` | `pan_tilt/action/ScanSequence` | Executes a predefined scanning sequence. |
| `MoveToAngles` | `pan_tilt/action/MoveToAngles` | Moves to a specified pan and tilt position. |

You can verify the available actions using:

```bash
ros2 action list
```

To inspect an action interface:

```bash
ros2 interface show pan_tilt/action/<ActionName>
```

### Send Pan-Tilt Goals

> **OBS:** In C++ or Python, the same goals can be sent using ROS 2 Action Clients.  
> All angular values are expressed in radians, and speed is expressed in radians per second.

#### MoveToAngles

Moves the mechanism to a specified pan and tilt position.

```bash
ros2 action send_goal /pan_tilt/move_to_angles pan_tilt/action/MoveToAngles "{target_pan: 0.5, target_tilt: 0.3, speed: 0.5}" --feedback
```

To return the mechanism to the center:

```bash
ros2 action send_goal /pan_tilt/move_to_angles pan_tilt/action/MoveToAngles "{target_pan: 0.0, target_tilt: 0.0, speed: 0.5}" --feedback
```

The action provides feedback containing the remaining distance to the target:

```yaml
distance_remaining: 0.25
```

The result contains the final position and whether the movement was successful.

#### PanSweep

Continuously sweeps the pan axis between minimum and maximum angles while maintaining a fixed tilt angle.

```bash
ros2 action send_goal /pan_tilt/pan_sweep pan_tilt/action/PanSweep "{min_pan: -0.7, max_pan: 0.7, tilt_angle: 0.0, speed: 0.5}" --feedback
```

The sweep continues indefinitely until canceled or preempted by another action. Feedback contains the current pan and tilt positions:

```yaml
current_pan: 0.5
current_tilt: 0.0
```

#### TiltSweep

Continuously sweeps the tilt axis between minimum and maximum angles while maintaining a fixed pan angle.

```bash
ros2 action send_goal /pan_tilt/tilt_sweep pan_tilt/action/TiltSweep "{min_tilt: -0.5, max_tilt: 0.5, pan_angle: 0.0, speed: 0.5}" --feedback
```

The sweep continues indefinitely until canceled or preempted by another action.

#### ScanSequence

Executes a predefined scanning sequence combining horizontal and vertical movements.

```bash
ros2 action send_goal /pan_tilt/scan_sequence pan_tilt/action/ScanSequence "{pan_amplitude: 0.5, tilt_amplitude: 0.3, speed: 0.5, pause_duration: 1.0}" --feedback
```

The sequence consists of the following stages:

1. Move to the left position.
2. Look up while facing left.
3. Look down while facing left.
4. Return tilt to center.
5. Return pan and tilt to center.
6. Move to the right position.
7. Look up while facing right.
8. Look down while facing right.
9. Return tilt to center.

The mechanism pauses at selected stages according to `pause_duration`. Feedback provides the current stage and joint positions:

```yaml
current_stage: "2_left_look_up"
current_pan: 0.5
current_tilt: 0.3
```

The result indicates whether the complete sequence was executed successfully.

### Action Preemption

Only one motion action is intended to control the mechanism at a time. Sending a new action goal while another is running causes the previous goal to be preempted, allowing the new goal to take control.

For example, you can start a continuous pan sweep:

```bash
ros2 action send_goal /pan_tilt/pan_sweep pan_tilt/action/PanSweep "{min_pan: -0.7, max_pan: 0.7, tilt_angle: 0.0, speed: 0.5}"
```

Then send a new movement goal:

```bash
ros2 action send_goal /pan_tilt/move_to_angles pan_tilt/action/MoveToAngles "{target_pan: 0.0, target_tilt: 0.0, speed: 0.5}"
```

The new goal takes control of the mechanism.

## Monitor Topics

The action server publishes servo commands to `/servo_angles`.

To monitor servo commands:

```bash
ros2 topic echo /servo_angles
```

The message type is:

```text
std_msgs/msg/Float32MultiArray
```

By default, the values are expressed in radians:

```yaml
data:
- 0.5
- 0.3
```

If the `publish_degrees` parameter is enabled, the values are converted into servo angles in degrees, with `90` degrees representing the center position.

The action server also publishes joint states for URDF and TF integration.

To monitor joint states:

```bash
ros2 topic echo /joint_states
```

The message type is:

```text
sensor_msgs/msg/JointState
```

The published joints are:

- `pan_joint`
- `tilt_joint`

## Parameters

The action server provides the following parameters:

| Parameter | Default | Description |
| :--- | :--- | :--- |
| `micro_ros_topic` | `/servo_angles` | Topic used to publish servo commands. |
| `publish_degrees` | `false` | Publishes servo positions in degrees instead of radians. |
| `control_rate_hz` | `50.0` | Motion control update frequency. |

Parameters can be inspected using:

```bash
ros2 param list /pan_tilt_action_server
```

To change a parameter at runtime:

```bash
ros2 param set /pan_tilt_action_server publish_degrees true
```
