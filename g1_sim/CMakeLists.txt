cmake_minimum_required(VERSION 3.0.2)
project(g1_sim)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf
  urdf
  xacro
)

add_message_files(
  FILES
  MPCxuk.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

