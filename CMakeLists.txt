cmake_minimum_required(VERSION 3.0.2)
project(demo_customer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

add_service_files(
  FILES
  AddTwoInts.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES open_face_recognition
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)
