#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include "fla_msgs/GlobalPath.h"
#include <geometry_msgs/PoseArray.h> // For lantern detection
#include <sensor_msgs/PointCloud2.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>

// Define states for the state machine
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

    // ROS Callbacks
    void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& pose);
    void lanternCallback(const geometry_msgs::PoseArray::ConstPtr& msg); // Callback for detected lanterns
    void mainLoop(const ros::TimerEvent& t);

    // Core State Machine Functions
    void takeOff();
    void flyToCave();
    void explore();
    void flyBack();
    void land();

    // Utility Functions
    void saveWayBack();
    void publishPath(const std::list<Eigen::Vector4d>& path);
    bool closeToGoal();
    void loadAndSendPath(const std::string& path);

private:
    // ROS Handles
    ros::NodeHandle nh_;
    ros::Publisher pub_max_v_;
    ros::Publisher pub_global_path_;
    ros::Publisher pub_controll_planner;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_all_lanterns_; // Subscriber for lantern locations
    ros::ServiceClient reset_octomap;
    ros::Timer timer_;

    // State Machine Variables
    State state_ = TAKE_OFF;
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
    int detected_lantern_count_; // Tracks the number of detected lanterns

    // Parameters
    int time_between_states_s;
    double min_dis_waypoint_back;
    double max_dis_close_to_goal;
    float max_speed_in_cave_;
};

