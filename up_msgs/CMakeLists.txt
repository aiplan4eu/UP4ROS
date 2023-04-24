cmake_minimum_required(VERSION 3.0.4)
project(up_msgs)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  actionlib_msgs
  actionlib
  std_msgs
  message_generation
 )
add_action_files(DIRECTORY action)
add_message_files(DIRECTORY msg)
add_service_files(DIRECTORY srv)
generate_messages(DEPENDENCIES std_msgs actionlib_msgs)
catkin_package(CATKIN_DEPENDS actionlib_msgs message_runtime std_msgs)
