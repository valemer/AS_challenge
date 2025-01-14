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

enum State {
    TAKE_OFF = 0,
    GO = 1,
    EXPLORED = 2,
    FLY_BACK = 3,
    STOP = 4
};

class StateMachine {
public:
    StateMachine();

    void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& pose);

    void mainLoop(const ros::TimerEvent& t);

    bool publishPath(const fla_msgs::GlobalPath& trajectory);

    bool closeToGoal();

    void takeOff();

    void flyToCave();

    void loadAndSendPath(std::string path);

private:
    ros::Publisher pub_global_path_;
    ros::Subscriber sub_odom_;
    ros::ServiceClient client;


    ros::Timer timer_;
    double hz_;

    ros::NodeHandle nh_;
    Eigen::Affine3d current_pose_;
    Eigen::Vector3d current_velocity_;

    State state_ = TAKE_OFF;

    Eigen::Vector3f current_goal_;
    int paths_sent_ = 0;
};
