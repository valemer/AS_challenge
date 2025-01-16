#pragma once

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include "fla_msgs/GlobalPath.h"
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <std_srvs/Empty.h>

enum State {
    TAKE_OFF = 0,
    GO = 1,
    PLAN_FARTHEST_POINT = 2,
    STOP = 3
};

class StateMachine {
public:
    StateMachine();

    void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom);

    void octomapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    void mainLoop(const ros::TimerEvent& t);

    bool publishPath(const fla_msgs::GlobalPath& trajectory);

    bool closeToGoal();

    void takeOff();

    void flyToCave();

    void planFarthestPoint();

    void loadAndSendPath(std::string path);

private:
    ros::Publisher pub_global_path_, pub_start_points_, pub_goal_points_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_octomap_;
    ros::ServiceClient client;

    ros::Timer timer_;
    double hz_;

    ros::NodeHandle nh_;
    Eigen::Affine3d current_pose_;
    Eigen::Vector3d current_velocity_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_;

    State state_ = TAKE_OFF;

    Eigen::Vector3f current_goal_;
    int paths_sent_ = 0;
};