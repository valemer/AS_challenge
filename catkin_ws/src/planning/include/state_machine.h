#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include "fla_msgs/GlobalPath.h"

enum State {
    TAKE_OFF = 0,
    GO = 1,
    EXPLORE = 2,
    FLY_BACK = 3,
    LAND = 4,
    STOP = 5
};

class StateMachine {
public:
    StateMachine();

    void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& pose);

    void mainLoop(const ros::TimerEvent& t);

    void saveWayBack();

    void publishPath(const std::list<Eigen::Vector4d>& path);

    bool closeToGoal();

    void takeOff();

    void flyToCave();

    void loadAndSendPath(const std::string& path);

    void explore();

    void flyBack();

    void land();

private:
    ros::Publisher pub_global_path_;
    ros::Publisher pub_controll_planner;
    ros::Subscriber sub_odom_;
    ros::ServiceClient reset_octomap;


    ros::Timer timer_;
    double hz_;

    ros::NodeHandle nh_;
    Eigen::Affine3d current_pose_;
    Eigen::Vector3d current_velocity_;

    State state_ = TAKE_OFF;

    std::list<Eigen::Vector4d> path_land_;
    std::list<Eigen::Vector4d> path_back_;
    Eigen::Vector3d current_goal_;
    bool paths_sent_ = false;

    // params
    int time_between_states_s;
    double min_dis_waypoint_back;
    double max_dis_close_to_goal;
};
