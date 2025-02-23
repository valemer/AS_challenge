#include "state_machine.h"

#include <thread>
#include <mav_msgs/common.h>
#include <std_msgs/Bool.h>

#include "yaml-cpp/yaml.h"
#include "ros/package.h"
#include "std_srvs/Empty.h"

#include <geometry_msgs/PoseArray.h>
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
        max_speed_out_cave_(15.0),
        detected_lantern_count_(0),
        lantern_search_num_(5)
{
    // publisher
    pub_max_v_ = nh_.advertise<std_msgs::Float32>("/max_speed", 1);
    pub_global_path_ = nh_.advertise<fla_msgs::GlobalPath>("/global_path", 1);
    pub_controll_planner = nh_.advertise<std_msgs::Bool>("/control_planner", 1);
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
    nh_.getParam("state_machine/max_speed_out_cave", max_speed_out_cave_);
    nh_.getParam("state_machine/lantern_search_num", lantern_search_num_);
}

void StateMachine::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    tf::poseMsgToEigen(odom->pose.pose, current_pose_);
}

void StateMachine::octomapCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    fromROSMsg(*msg, *point_cloud_);
}

// Callback to track detected lanterns
void StateMachine::lanternCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
    if (detected_lantern_count_ < static_cast<int>(msg->poses.size())) {
        detected_lantern_count_ = static_cast<int>(msg->poses.size()); // Get the number of detected lanterns
        ROS_INFO("Detected lanterns: %d", detected_lantern_count_);
    }
}

void StateMachine::mainLoop(const ros::TimerEvent& t) {
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
            break;
        case FLY_BACK:
            flyBack();
            break;
        case LAND:
            land();
            break;
        default:
            break;
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


        if (path == "flyToCave") {
            auto point = *config["flyBack"].begin();
            path_back_.emplace_front(point[0].as<float>(), point[1].as<float>(), point[2].as<float>(),
                std::fmod(point[3].as<float>() + M_PI, 2 * M_PI));
        }
    }
    pub_global_path_.publish(global_path);

    current_goal_ << global_point.point.x, global_point.point.y, global_point.point.z;
}

void StateMachine::explore() {
    if (!paths_sent_) {
        std_msgs::Bool msg;
        msg.data = true;
        pub_controll_planner.publish(msg);
        paths_sent_ = true;
    } else if (detected_lantern_count_ >= lantern_search_num_) {
        paths_sent_ = false;
        state_ = FLY_BACK;
    }

}

void StateMachine::flyBack() {
    if (!paths_sent_) {
        ROS_INFO_ONCE("Flying back to the starting point...");
        std_msgs::Bool msg;
        msg.data = false;
        pub_controll_planner.publish(msg);

        std_msgs::Float32 msg2;
        msg2.data = max_speed_out_cave_;
        pub_max_v_.publish(msg2);

        geometry_msgs::Point start_point, goal_point;

        start_point.x = current_pose_.translation().x();
        start_point.y = current_pose_.translation().y();
        start_point.z = current_pose_.translation().z();
        pub_fly_back_start_points_.publish(start_point);

        goal_point.x = path_back_.front()[0];
        goal_point.y = path_back_.front()[1];
        goal_point.z = path_back_.front()[2];

        pub_fly_back_goal_points_.publish(goal_point);

        current_goal_ << goal_point.x, goal_point.y, goal_point.z;

        paths_sent_ = true;
    }

    static bool entrance_goal_reached = false;

    if (closeToGoal()) {
        if (!entrance_goal_reached) {
            entrance_goal_reached = true;

            fla_msgs::GlobalPath global_path;
            fla_msgs::GlobalPoint global_point;

            for (const auto& point : path_back_) {
                global_point.point.x = point[0];
                global_point.point.y = point[1];
                global_point.point.z = point[2];
                global_point.orientation = static_cast<float>(point[3]);
                global_point.velocity = -1;
                global_point.acceleration = -1;
                global_path.points.push_back(global_point);
            }
            global_path.points.back().acceleration = 0.0;
            global_path.points.back().velocity = 0.0;

            pub_global_path_.publish(global_path);

            current_goal_ << global_point.point.x, global_point.point.y, global_point.point.z;
        } else {
            state_ = LAND;
            paths_sent_ = false;
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
        global_path.points.erase(global_path.points.begin());
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