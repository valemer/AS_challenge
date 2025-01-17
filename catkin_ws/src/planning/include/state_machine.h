#pragma once

#include <ros/ros.h>
#include <thread>

#include <mav_msgs/common.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>
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

    void mainLoop(const ros::TimerEvent& t);

    void saveWayBack();

    void publishPath(const std::list<Eigen::Vector4d>& path);

    bool closeToGoal();

    void takeOff();

    void flyToCave();

    void planFarthestPoint();

    void loadAndSendPath(const std::string& path);

    void flyBack();

    void land();

private:
    ros::Publisher pub_global_path_, pub_start_points_, pub_goal_points_, pub_markers_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_octomap_;
    ros::ServiceClient reset_octomap;

    ros::Timer timer_;
    double hz_;

    ros::NodeHandle nh_;
    Eigen::Affine3d current_pose_;
    Eigen::Vector3d current_velocity_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_;

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