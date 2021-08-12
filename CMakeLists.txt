cmake_minimum_required(VERSION 3.0.2)
project(judge_target_object)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  actionlib_msgs
  # Added packages:
  #darknet_ros_msgs
)

add_message_files(
  FILES
  BoundingBox2D.msg
)

add_action_files(
  DIRECTORY
  action
  FILES
  BoundingBox2D.action
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
  #darknet_ros_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
