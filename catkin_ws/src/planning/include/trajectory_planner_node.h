#pragma once

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include "fla_msgs/GlobalPath.h"
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <std_msgs/Float32.h>
#include <math.h>

class TrajectoryPlanner {
public:
    TrajectoryPlanner();

    void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& pose);

    void setMaxSpeed(const std_msgs::Float32::ConstPtr& max_v);

    void planTrajectory(const fla_msgs::GlobalPath::ConstPtr& globalPath);

    bool publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory);

private:
    ros::Publisher pub_markers_;
    ros::Publisher pub_trajectory_;
    ros::Subscriber sub_max_speed_;
    ros::Subscriber sub_global_path_;
    ros::Subscriber sub_odom_;

    ros::NodeHandle nh_;
    Eigen::Affine3d current_pose_;
    Eigen::Vector3d current_velocity_;
    Eigen::Vector3d current_angular_velocity_;

    double max_v_; // m/s
    double max_a_; // m/s^2
    double max_ang_v_;
    double max_ang_a_;

};
