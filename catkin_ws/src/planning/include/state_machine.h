#pragma once

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <pcl_conversions/pcl_conversions.h>

#include "yaml-cpp/yaml.h"


// Define states for the state machine
enum State {
    TAKE_OFF = 0,
    FLY_TO_CAVE = 1,
    EXPLORE = 2,
    FLY_BACK = 3,
    LAND = 4,
    STOP = 5
};

class StateMachine {
    // ROS Handles
    ros::NodeHandle nh_;

    // Publisher
    ros::Publisher pub_max_v_;
    ros::Publisher pub_global_path_;
    ros::Publisher pub_controll_planner;
    ros::Publisher pub_fly_back_start_points_;
    ros::Publisher pub_fly_back_goal_points_;

    // Subscriber
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_all_lanterns_;
    ros::ServiceClient reset_octomap;

    ros::Timer timer_;

    // State Machine Variables
    State state_;
    double hz_;
    bool paths_sent_ = false;

    // Current State Information
    Eigen::Affine3d current_pose_;
    Eigen::Vector3d current_velocity_;

    Eigen::Vector3d current_goal_;

    // Paths for Navigation
    std::list<Eigen::Vector4d> path_land_;
    std::list<Eigen::Vector4d> path_back_;

    // Lantern Detection Variables
    geometry_msgs::PoseArray detected_lanterns_;
    int lantern_search_num_;

    // Parameters
    int time_between_states_s;
    double max_dis_close_to_goal;
    float max_speed_in_cave_;
    float max_speed_out_cave_;

public:
    StateMachine();

    void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom);
    void lanternCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
    void mainLoop(const ros::TimerEvent& t);

    // Core State Machine Functions
    void takeOff();
    void flyToCave();
    void explore();
    void flyBack();
    void land();

    // Utility Functions
    void publishPath(const std::list<Eigen::Vector4d>& path);
    bool closeToGoal();
    void loadAndSendPath(const std::string& path);
};
