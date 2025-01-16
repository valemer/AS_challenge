#pragma once

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <eigen_conversions/eigen_msg.h>
#include "fla_msgs/GlobalPath.h"
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include <mav_planning_msgs/PolynomialTrajectory4D.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

class TrajectoryPlanner {
public:
    TrajectoryPlanner();

    void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& pose);

    void setMaxSpeed(double max_v);

    void planTrajectory(const nav_msgs::Path::ConstPtr& globalPath);

    void planTrajectoryInsideCave(const nav_msgs::Path::ConstPtr& plannedPath);

    bool publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory);

private:
    ros::Publisher pub_markers_;
    ros::Publisher pub_trajectory_;
    ros::Subscriber sub_global_path_;
    ros::Subscriber sub_planned_path_;
    ros::Subscriber sub_odom_;

    ros::NodeHandle nh_;
    Eigen::Affine3d current_pose_;
    Eigen::Vector3d current_velocity_;
    Eigen::Vector3d current_angular_velocity_;

    double max_v_; // m/s
    double max_a_; // m/s^2
    double max_ang_v_; // rad/s
    double max_ang_a_; // rad/s^2
};