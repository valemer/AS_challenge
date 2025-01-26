#include "state_machine.h"

#include <thread>
#include <mav_msgs/common.h>
#include <std_msgs/Bool.h>

#include "yaml-cpp/yaml.h"
#include "ros/package.h"
#include "std_srvs/Empty.h"

#include <geometry_msgs/PoseArray.h> // For lantern locations
#include <std_msgs/Float32.h>

StateMachine::StateMachine() :
        hz_(10.0),
        current_velocity_(Eigen::Vector3d::Zero()),
        current_pose_(Eigen::Affine3d::Identity()),
        point_cloud_(new pcl::PointCloud<pcl::PointXYZ>()),
        time_between_states_s(2),
        min_dis_waypoint_back(5.0),
        max_dis_close_to_goal(1.0),
        max_speed_in_cave_(5.0),
        detected_lantern_count_(0) // Initialize the lantern count
{
    // publisher
    pub_max_v_ = nh_.advertise<std_msgs::Float32>("/max_speed", 1);
    pub_global_path_ = nh_.advertise<fla_msgs::GlobalPath>("/global_path", 1);
    pub_controll_planner = nh_.advertise<std_msgs::Bool>("/control_planner", 1);
    // pub_visited_locations_ = nh_.advertise<visualization_msgs::MarkerArray>("/visited_locations", 1);
    pub_fly_back_start_points_ = nh_.advertise<geometry_msgs::Point>("/fly_back_start_points", 1);
    pub_fly_back_goal_points_ = nh_.advertise<geometry_msgs::Point>("/fly_back_goal_points", 1);


    // subscriber
    sub_odom_ = nh_.subscribe("/current_state_est", 1, &StateMachine::uavOdomCallback, this);
    sub_octomap_ = nh_.subscribe("/octomap_point_cloud_centers", 1, &StateMachine::octomapCallback, this);
    sub_all_lanterns_ = nh_.subscribe("/all_detected_lantern_locations", 1, &StateMachine::lanternCallback, this); // New subscriber for lantern locations

    // Main loop timer
    timer_ = nh_.createTimer(ros::Rate(hz_), &StateMachine::mainLoop, this);

    // Service
    ros::service::waitForService("/octomap_server/reset");
    reset_octomap = nh_.serviceClient<std_srvs::Empty>("/octomap_server/reset");

    // params
    nh_.getParam("state_machine/time_between_states_s", time_between_states_s);
    nh_.getParam("state_machine/min_dis_waypoint_back", min_dis_waypoint_back);
    nh_.getParam("state_machine/max_dis_close_to_goal", max_dis_close_to_goal);
    nh_.getParam("state_machine/max_speed_in_cave", max_speed_in_cave_);
}

// Callback to get current Pose of UAV
void StateMachine::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    // Store current position in our planner
    tf::poseMsgToEigen(odom->pose.pose, current_pose_);
}

void StateMachine::octomapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    pcl::fromROSMsg(*msg, *point_cloud_);
    // ROS_INFO("Received OctoMap PointCloud2 with %lu points.", point_cloud_->points.size());
}

// Callback to track detected lanterns
void StateMachine::lanternCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
    detected_lantern_count_ = msg->poses.size(); // Get the number of detected lanterns
    ROS_INFO("Detected lanterns: %d", detected_lantern_count_);
}

void StateMachine::mainLoop(const ros::TimerEvent& t) {
    // ROS_INFO("current position: (%f, %f, %f)", current_pose_.translation()[0], current_pose_.translation()[1], current_pose_.translation()[2]);
    switch(state_)
    {
        case TAKE_OFF:
            takeOff();
            break;
        case GO:
            flyToCave();
            break;
        case EXPLORE:
            explore();
            saveWayBack();
            break;
        case FLY_BACK:  // New state for flying back
            flyBack();
            break;
        case LAND:
            land();
            break;
    }
    // Trigger FLY_BACK state when 4 or more lanterns are detected
    // if (detected_lantern_count_ >= 4 && state_ != FLY_BACK) {
    //     state_ = FLY_BACK;
    //     ROS_INFO("Triggering FLY_BACK state: 4 or more lanterns detected.");
    // }
}

void StateMachine::saveWayBack()
{
    if (sqrt(
        pow(current_pose_.translation()[0] - path_back_.front()[0], 2)
        + pow(current_pose_.translation()[1] - path_back_.front()[1], 2)
        + pow(current_pose_.translation()[2] - path_back_.front()[2], 2)) > min_dis_waypoint_back)
        {
        path_back_.emplace_front(
            current_pose_.translation()[0],
            current_pose_.translation()[1],
            current_pose_.translation()[2],
            std::fmod(mav_msgs::yawFromQuaternion(
            static_cast<Eigen::Quaterniond>(current_pose_.rotation())) + M_PI, 2 * M_PI));
    }
}

bool StateMachine::closeToGoal() {
    double dist = sqrt(pow(current_pose_.translation()[0] - current_goal_[0], 2)
                     + pow(current_pose_.translation()[1] - current_goal_[1], 2)
                     + pow(current_pose_.translation()[2] - current_goal_[2], 2));
   return dist < max_dis_close_to_goal;
}

void StateMachine::takeOff() {
  if (!paths_sent_) {
      std::this_thread::sleep_for(std::chrono::seconds(time_between_states_s));
      if (current_pose_.translation()[0] == 0
          && current_pose_.translation()[1] == 0
          && current_pose_.translation()[2] == 0)
          return;
      path_land_.emplace_front(
            current_pose_.translation()[0],
            current_pose_.translation()[1],
            current_pose_.translation()[2],
            std::fmod(mav_msgs::yawFromQuaternion(
            static_cast<Eigen::Quaterniond>(current_pose_.rotation())) + M_PI, 2 * M_PI));
      ROS_INFO_NAMED("state_machine", "Start take-off!");
      loadAndSendPath("takeoff");
      paths_sent_ = true;
  } else if (closeToGoal()) {
    ROS_INFO_NAMED("state_machine", "Close to take-off goal!");
    paths_sent_ = false;
    state_ = GO;
    std::this_thread::sleep_for(std::chrono::seconds(time_between_states_s));
  }
}

void StateMachine::flyToCave() {
    if (!paths_sent_) {
        loadAndSendPath("flyToCave");
        paths_sent_ = true;
    } else if (closeToGoal()) {
        paths_sent_ = false;
        state_ = EXPLORE;
        std_msgs::Float32 msg;
        msg.data = max_speed_in_cave_;
        pub_max_v_.publish(msg);
        std_srvs::Empty srv;
        if (reset_octomap.call(srv)) {
            ROS_INFO("OctoMap reset successful.");
        } else {
            ROS_ERROR("Failed to reset OctoMap.");
        }
        std::this_thread::sleep_for(std::chrono::seconds(time_between_states_s));
    }
}

void StateMachine::loadAndSendPath(const std::string& path) {
    YAML::Node config = YAML::LoadFile(ros::package::getPath("planning") +
        "/config/state_machine_config.yaml");

    fla_msgs::GlobalPath global_path;
    fla_msgs::GlobalPoint global_point;

    for (auto it = config[path].begin(); it != config[path].end(); ++it) {
        global_point.point.x = (*it)[0].as<float>();
        global_point.point.y = (*it)[1].as<float>();
        global_point.point.z = (*it)[2].as<float>();
        global_point.orientation = (*it)[3].as<float>();
        global_point.velocity = (*it)[4].as<float>();
        global_point.acceleration = (*it)[5].as<float>();
        global_path.points.push_back(global_point);

        if (path == "takeoff") {
            if (std::next(it) != config[path].end())
                path_land_.emplace_front((*it)[0].as<float>(), (*it)[1].as<float>(), (*it)[2].as<float>(),
                    std::fmod((*it)[3].as<float>() + M_PI, 2 * M_PI));
            else
                path_back_.emplace_front((*it)[0].as<float>(), (*it)[1].as<float>(), (*it)[2].as<float>(),
                    std::fmod((*it)[3].as<float>() + M_PI, 2 * M_PI));
        }


        if (path == "flyToCave" && std::next(it) != config[path].end())
            path_back_.emplace_front((*it)[0].as<float>(), (*it)[1].as<float>(), (*it)[2].as<float>(),
                std::fmod((*it)[3].as<float>() + M_PI, 2 * M_PI));
    }
    pub_global_path_.publish(global_path);

    current_goal_ << global_point.point.x, global_point.point.y, global_point.point.z;
}

void StateMachine::explore() {
    // static std::vector<Eigen::Vector3d> visited_locations;
    // static visualization_msgs::MarkerArray markers;

    // // Add current location to visited locations if far enough from the last point
    // if (visited_locations.empty() || (current_pose_.translation() - visited_locations.back()).norm() > 5.0) {
    //     visited_locations.push_back(current_pose_.translation());
    //     ROS_INFO("ADDED A VISITED POINT");

    //     // Create a marker for the new visited point
    //     visualization_msgs::Marker marker;
    //     marker.header.frame_id = "world";
    //     marker.header.stamp = ros::Time::now();
    //     marker.ns = "visited_locations";
    //     marker.id = visited_locations.size() - 1;
    //     marker.type = visualization_msgs::Marker::SPHERE;
    //     marker.action = visualization_msgs::Marker::ADD;
    //     marker.pose.position.x = current_pose_.translation()[0];
    //     marker.pose.position.y = current_pose_.translation()[1];
    //     marker.pose.position.z = current_pose_.translation()[2];
    //     marker.scale.x = 0.3;
    //     marker.scale.y = 0.3;
    //     marker.scale.z = 0.3;
    //     marker.color.r = 0.0;
    //     marker.color.g = 1.0;
    //     marker.color.b = 0.0;
    //     marker.color.a = 1.0;

    //     markers.markers.push_back(marker);
    // }

    // // Publish the markers to RViz
    // pub_visited_locations_.publish(markers);


    if (!paths_sent_) {
        std_msgs::Bool msg;
        msg.data = true;
        pub_controll_planner.publish(msg);
        paths_sent_ = true;
    } else if (detected_lantern_count_ >= 5) { // TODO: if all lanterns found
    //} else if (path_back_.size() >= 15){
        paths_sent_ = false;
        state_ = FLY_BACK;
    }

}

void StateMachine::flyBack() {
    // ROS_INFO("Flying back to the starting point...");

    std_msgs::Bool msg;
    msg.data = false;
    pub_controll_planner.publish(msg);

    static uint8_t state = 0; // State machine for flying back

    YAML::Node config = YAML::LoadFile(ros::package::getPath("planning") +
        "/config/state_machine_config.yaml");

    static ros::Time last_time = ros::Time::now(); // initialize timer

    // ROS_INFO("state of flyback: %d", state);

    switch(state) {
        case 0: // fly back to entrance of cave
        { 
            ROS_INFO("Flying back to the entrance of the cave...");
            geometry_msgs::Point start_point, goal_point;

            start_point.x = current_pose_.translation().x();
            start_point.y = current_pose_.translation().y();
            start_point.z = current_pose_.translation().z();
            pub_fly_back_start_points_.publish(start_point);

            // Access the "flyToCave" section and get the last element
            if (config["flyToCave"] && config["flyToCave"].size() > 0) {
                size_t last_index = config["flyToCave"].size() - 1;
                auto last_point = config["flyToCave"][last_index];
                goal_point.x = last_point[0].as<float>();
                goal_point.y = last_point[1].as<float>();
                goal_point.z = last_point[2].as<float>();
            } else {
                std::cerr << "flyToCave section not found or is empty in the YAML file." << std::endl;
            }
            pub_fly_back_goal_points_.publish(goal_point);

            current_goal_ << goal_point.x, goal_point.y, goal_point.z;
            
            state = 1; // stop publishing goal points and wait until we reach goal
            last_time = ros::Time::now(); // reset timer
            Eigen::Affine3d last_pose_ = current_pose_; // store current pose

            break;
        }
        case 1:
        {
            if ((current_pose_.translation() - last_pose_.translation()).norm() > min_distance_to_travel_before_timeout_){
                last_time = ros::Time::now(); // reset timer if drone moved
                last_pose_ = current_pose_; // store current pose
            }

            if ((ros::Time::now() - last_time).toSec() > timeout_) { // check if drone is not moving x metres away after x seconds
                ROS_WARN("Drone is not moving. Flying back to the entrance of the cave...");
                state = 0;
            }

            if (closeToGoal()) {
                ROS_INFO("Reached the entrance of the cave. Flying back to the starting point...");
                state = 2;
            }
            break;
        }
        case 2: // fly back to the starting point
        {
            fla_msgs::GlobalPath global_path;
            fla_msgs::GlobalPoint global_point;

            // Iterate through the config in reverse order, excluding the last element because we have flown through it
            for (int i = config["flyToCave"].size() - 2; i >= 0; --i) { // Exclude the last element
                auto point = config["flyToCave"][i];
                global_point.point.x = point[0].as<float>();
                global_point.point.y = point[1].as<float>();
                global_point.point.z = point[2].as<float>();
                global_point.orientation = point[3].as<float>();
                global_point.velocity = point[4].as<float>();
                global_point.acceleration = point[5].as<float>();
                global_path.points.push_back(global_point);
            }

            // Publish the reversed global path
            pub_global_path_.publish(global_path);

            current_goal_ << global_point.point.x, global_point.point.y, global_point.point.z;
            
            last_time = ros::Time::now(); // reset timer
            last_pose_ = current_pose_; // store current pose

            state = 3; // stop publishing goal points and wait until we reach goal
        
            break;
        }
        case 3:
        {
            ROS_INFO("Current position: (%f, %f, %f)", current_pose_.translation().x(), current_pose_.translation().y(), current_pose_.translation().z());
            ROS_INFO("Current goal: (%f, %f, %f)", current_goal_[0], current_goal_[1], current_goal_[2]);

            if ((current_pose_.translation() - last_pose_.translation()).norm() > min_distance_to_travel_before_timeout_){
                last_time = ros::Time::now(); // reset timer if drone moved
                last_pose_ = current_pose_; // store current pose
            }

            if ((ros::Time::now() - last_time).toSec() > timeout_) { // check if drone is not moving x metres away after x seconds
                ROS_INFO("Drone is not moving. Flying back to the starting point...");
                state = 2;
            }

            if (closeToGoal()) {
                ROS_INFO("Reached the starting point. Landing...");
                state = 4;
            }
            break;
        }
        case 4:
        {
            state_ = LAND;
            break;
        }
        default:
        {
            break;

        }
    }

}

void StateMachine::land() {
    if (!paths_sent_) {
        fla_msgs::GlobalPath global_path;
        for (const auto& point : path_land_) {
            fla_msgs::GlobalPoint global_point;
            global_point.point.x = point[0];
            global_point.point.y = point[1];
            global_point.point.z = point[2];
            global_point.orientation = static_cast<float>(point[3]);
            global_point.velocity = -1.0;
            global_point.acceleration = -1.0;
            global_path.points.push_back(global_point);
        }
        pub_global_path_.publish(global_path);
        paths_sent_ = true;

        current_goal_ << global_path.points.back().point.x, global_path.points.back().point.y, global_path.points.back().point.z;
    } else if (closeToGoal()) {
        paths_sent_ = false;
        state_ = STOP;
    }
}

void StateMachine::publishPath(const std::list<Eigen::Vector4d>& path) {
    fla_msgs::GlobalPath global_path;
    for (const auto& point : path) {
        fla_msgs::GlobalPoint global_point;
        global_point.point.x = point[0];
        global_point.point.y = point[1];
        global_point.point.z = point[2];
        global_point.orientation = static_cast<float>(point[3]);
        global_point.velocity = -1.0;
        global_point.acceleration = -1.0;
        global_path.points.push_back(global_point);
    }
    pub_global_path_.publish(global_path);
    paths_sent_ = true;

    current_goal_ << global_path.points.back().point.x, global_path.points.back().point.y, global_path.points.back().point.z;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "state_machine");
    ROS_INFO_NAMED("state_machine", "State Machine started!");
    StateMachine n;
    ros::spin();
}