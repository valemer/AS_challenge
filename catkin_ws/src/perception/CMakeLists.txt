cmake_minimum_required(VERSION 3.0.2)
project(perception)

find_package(catkin REQUIRED COMPONENTS
        catkin_simple
        roscpp
        rospy
        std_msgs
        sensor_msgs
        pcl_conversions
        pcl_ros
        octomap_ros
        octomap_msgs
        nav_msgs
        fla_msgs
)

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_executable(lantern_detection
        src/lantern_detection.cpp
)

cs_add_executable(junction_detection
        src/junction_detection.cpp
)

cs_install()
cs_export()


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


