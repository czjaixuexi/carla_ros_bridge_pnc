cmake_minimum_required(VERSION 3.0.2)
project(controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  waypoint_msgs
  carla_msgs
  # reference_line
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  # INCLUDE_DIRS include
  #DEPENDS reference_line
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(controller
  src/controller_node.cpp  
  src/controller.cpp
  src/controller_main.cpp)

target_link_libraries(controller
  ${catkin_LIBRARIES}
)

add_dependencies(controller
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${EIGEN_DEFINITIONS}
)
