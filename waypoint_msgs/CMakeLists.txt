cmake_minimum_required(VERSION 3.0.2)
project(waypoint_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  message_generation
  sensor_msgs
  uuid_msgs
  nav_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  Waypoint.msg
  WaypointArray.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  sensor_msgs
  uuid_msgs
  nav_msgs
)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES waypoint_msgs
  CATKIN_DEPENDS message_runtime roscpp geometry_msgs std_msgs uuid_msgs nav_msgs
  # DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
