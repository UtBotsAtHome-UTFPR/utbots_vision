cmake_minimum_required(VERSION 2.8.3)
project(vision_msgs)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	std_msgs
	sensor_msgs
	geometry_msgs
)

add_message_files(
  FILES
  PersonArray.msg
  PointArray.msg
  Object.msg
  ObjectArray.msg
  Skeleton2d.msg
  BoundingBox.msg
  BoundingBoxes.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
