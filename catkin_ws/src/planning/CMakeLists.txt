cmake_minimum_required(VERSION 3.0.2)
project(planning)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++11)

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
  geometry_msgs # Ensure geometry_msgs is included
)

catkin_simple(ALL_DEPS_REQUIRED)

## Add executables
cs_add_executable(state_machine
  src/state_machine.cpp
)

cs_add_executable(trajectory_planner
  src/trajectory_planner_node.cc
)

## Install and Export
cs_install()
cs_export()

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/manual_control.py
  scripts/cave_explorer_node.py
  scripts/exploration_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

