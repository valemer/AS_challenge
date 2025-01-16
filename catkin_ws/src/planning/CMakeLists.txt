cmake_minimum_required(VERSION 3.0.2)
project(planning)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14) # Use 17 if required
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_MACOSX_RPATH 0)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  eigen_catkin
  catkin_simple
  mav_msgs
  mav_planning_msgs
  mav_trajectory_generation
  mav_trajectory_generation_ros
  mav_visualization
  roslib
  yaml_cpp_catkin
  fla_msgs
  sensor_msgs
  std_srvs
  geometry_msgs
)

## Find PCL
find_package(PCL 1.7 REQUIRED)

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Link directories
link_directories(${PCL_LIBRARY_DIRS})

## Define macros for PCL
add_definitions(${PCL_DEFINITIONS})

## Add executables
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_executable(state_machine
  src/state_machine.cpp
)

cs_add_executable(trajectory_planner
  src/trajectory_planner_node.cc
)

## Link PCL libraries to the executables
target_link_libraries(state_machine
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(trajectory_planner
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## Install and export
cs_install()
cs_export()
