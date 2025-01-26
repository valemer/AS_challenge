#pragma once

#include <ros/ros.h>
#include <thread>

#include <mav_msgs/common.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseArray.h> // For lantern detection
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <std_srvs/Empty.h>

#include "fla_msgs/GlobalPath.h"
#include "yaml-cpp/yaml.h"
#include "ros/package.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Point.h"

//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
#include <visualization_msgs/MarkerArray.h> // For RViz visualization

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

    void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom);
    void octomapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
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
    // ros::Publisher pub_visited_locations_; // Publisher for visited locations
    ros::Publisher pub_fly_back_start_points_;
    ros::Publisher pub_fly_back_goal_points_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_octomap_;
    ros::Subscriber sub_all_lanterns_; // Subscriber for lantern locations
    ros::ServiceClient reset_octomap;
    ros::Timer timer_;

    // State Machine Variables
    State state_ = TAKE_OFF;
    double hz_;
    bool paths_sent_ = false;

    // Current State Information
    Eigen::Affine3d current_pose_;
    Eigen::Affine3d last_pose_;
    Eigen::Vector3d current_velocity_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_;
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

    // flyBack Variables
    double min_distance_to_travel_before_timeout_ = 1; 
    double timeout_ = 60;
};
