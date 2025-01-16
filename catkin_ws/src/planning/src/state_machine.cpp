#include "state_machine.h"

#include <thread>
#include "yaml-cpp/yaml.h"
#include "ros/package.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Point.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

StateMachine::StateMachine() :
        hz_(10.0),
        current_pose_(Eigen::Affine3d::Identity()),
        point_cloud_(new pcl::PointCloud<pcl::PointXYZ>())
{
    // Publisher
    pub_global_path_ = nh_.advertise<fla_msgs::GlobalPath>("/global_path", 0);
    pub_start_points_ = nh_.advertise<geometry_msgs::Point>("/start_points", 1);
    pub_goal_points_ = nh_.advertise<geometry_msgs::Point>("/goal_points", 1);
    // Subscriber
    sub_odom_ = nh_.subscribe("/current_state_est", 1, &StateMachine::uavOdomCallback, this);
    sub_octomap_ = nh_.subscribe("/octomap_point_cloud_centers", 1, &StateMachine::octomapCallback, this);

    // Main loop timer
    timer_ = nh_.createTimer(ros::Rate(hz_), &StateMachine::mainLoop, this);

    // Service
    ros::service::waitForService("/octomap_server/reset");
    client = nh_.serviceClient<std_srvs::Empty>("/octomap_server/reset");
}

// Callback to get current Pose of UAV
void StateMachine::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    // Store current position in our planner
    tf::poseMsgToEigen(odom->pose.pose, current_pose_);
}

void StateMachine::octomapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    pcl::fromROSMsg(*msg, *point_cloud_);
    ROS_INFO("Received OctoMap PointCloud2 with %lu points.", point_cloud_->points.size());
}

void StateMachine::mainLoop(const ros::TimerEvent& t) {
    switch (state_) {
        case TAKE_OFF:
            takeOff();
            break;
        case GO:
            flyToCave();
            break;
        case PLAN_FARTHEST_POINT:
            planFarthestPoint();
            break;
    }
}

bool StateMachine::closeToGoal() {
    double dist = sqrt(pow(current_pose_.translation()[0] - current_goal_[0], 2)
                     + pow(current_pose_.translation()[1] - current_goal_[1], 2)
                     + pow(current_pose_.translation()[2] - current_goal_[2], 2));
    return dist < 0.5;
}

void StateMachine::takeOff() {
    if (paths_sent_ < 1) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        ROS_INFO_NAMED("state_machine", "Start take-off!");
        loadAndSendPath("takeoff");
        paths_sent_++;
    } else if (closeToGoal()) {
        ROS_INFO_NAMED("state_machine", "Close to take-off goal!");
        paths_sent_ = 0;
        state_ = GO;
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
}

void StateMachine::flyToCave() {
    if (!paths_sent_) {
        loadAndSendPath("flyToCave");
        paths_sent_ = true;
    } else if (closeToGoal()) {
        paths_sent_ = false;
        state_ = PLAN_FARTHEST_POINT;
        std_srvs::Empty srv;
        if (client.call(srv)) {
            ROS_INFO("OctoMap reset successfully.");
        } else {
            ROS_ERROR("Failed to reset OctoMap.");
        }
    }
}

void StateMachine::planFarthestPoint() {
    if (point_cloud_->empty()) {
        ROS_WARN("No points available in the OctoMap.");
        return;
    }

    double max_distance = 0.0;
    Eigen::Vector3d farthest_point;

    for (const auto& point : point_cloud_->points) {
        Eigen::Vector3d node_center(point.x, point.y, point.z);
        double distance = (node_center - current_pose_.translation()).norm();

        if (distance > max_distance) {
            max_distance = distance;
            farthest_point = node_center;
        }
    }

    if (max_distance > 0.0) {
        ROS_INFO_NAMED("state_machine", "Farthest point found at (%f, %f, %f).",
                       farthest_point.x(), farthest_point.y(), farthest_point.z());

        geometry_msgs::Point start_point, goal_point;

        start_point.x = current_pose_.translation().x();
        start_point.y = current_pose_.translation().y();
        start_point.z = current_pose_.translation().z();
        pub_start_points_.publish(start_point);

        goal_point.x = farthest_point.x();
        goal_point.y = farthest_point.y();
        goal_point.z = farthest_point.z();
        pub_goal_points_.publish(goal_point);

        current_goal_ << farthest_point.x(), farthest_point.y(), farthest_point.z();
    }
}

void StateMachine::loadAndSendPath(std::string path) {
    YAML::Node config = YAML::LoadFile(ros::package::getPath("planning") +
        "/config/state_machine_config.yaml");

    fla_msgs::GlobalPath global_path;
    fla_msgs::GlobalPoint global_point;
    for (const auto& point : config[path]) {
        global_point.point.x = point[0].as<float>();
        global_point.point.y = point[1].as<float>();
        global_point.point.z = point[2].as<float>();
        global_point.orientation = point[3].as<float>();
        global_point.velocity = point[4].as<float>();
        global_point.acceleration = point[5].as<float>();
        global_path.points.push_back(global_point);
    }
    pub_global_path_.publish(global_path);

    current_goal_ << global_point.point.x, global_point.point.y, global_point.point.z;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_machine");
    ROS_INFO_NAMED("state_machine", "State Machine started!");
    StateMachine n;
    ros::spin();
}