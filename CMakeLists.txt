cmake_minimum_required(VERSION 2.8.3)
project(judge_target_object)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  actionlib_msgs
  # Added packages:
  darknet_ros_msgs
)

add_message_files(
  FILES
  BoundingBox2D.msg
  BoundingBox3D.msg
  MaskBoundingBox2D.msg
)

add_action_files(
  DIRECTORY
  action
  FILES
  BoundingBox2D.action
  BoundingBox3D.action
  MaskBoundingBox2D.action
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  actionlib_msgs
  message_runtime
  darknet_ros_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
