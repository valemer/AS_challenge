#include "state_machine.h"

#include <thread>
#include <mav_msgs/common.h>
#include <std_msgs/Bool.h>

#include "yaml-cpp/yaml.h"
#include "ros/package.h"
#include "std_srvs/Empty.h"

#include <geometry_msgs/PoseArray.h> // For lantern locations

StateMachine::StateMachine() :
        hz_(10.0),
        current_velocity_(Eigen::Vector3d::Zero()),
        current_pose_(Eigen::Affine3d::Identity()),
        time_between_states_s(2),
        min_dis_waypoint_back(5.0),
        max_dis_close_to_goal(1.0),
        detected_lantern_count_(0) // Initialize the lantern count
{
    // publisher
    pub_global_path_ = nh_.advertise<fla_msgs::GlobalPath>("/global_path", 0);
    pub_controll_planner = nh_.advertise<std_msgs::Bool>("/control_planner", 0);

    // subscriber
    sub_odom_ = nh_.subscribe("/current_state_est", 1, &StateMachine::uavOdomCallback, this);
    sub_all_lanterns_ = nh_.subscribe("/all_detected_lantern_locations", 1, &StateMachine::lanternCallback, this); // New subscriber for lantern locations

    // main loop timer
    timer_ = nh_.createTimer(ros::Rate(hz_), &StateMachine::mainLoop, this);

    // Service
    ros::service::waitForService("/octomap_server/reset");
    reset_octomap = nh_.serviceClient<std_srvs::Empty>("/octomap_server/reset");

    // params
    nh_.getParam("state_machine/time_between_states_s", time_between_states_s);
    nh_.getParam("state_machine/min_dis_waypoint_back", min_dis_waypoint_back);
    nh_.getParam("state_machine/max_dis_close_to_goal", max_dis_close_to_goal);
}

// Callback to get current Pose of UAV
void StateMachine::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {

    // store current position in our planner
    tf::poseMsgToEigen(odom->pose.pose, current_pose_);

    // store current velocity
    tf::vectorMsgToEigen(odom->twist.twist.linear, current_velocity_);
}

// Callback to track detected lanterns
void StateMachine::lanternCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
    detected_lantern_count_ = msg->poses.size(); // Get the number of detected lanterns
    ROS_INFO("Detected lanterns: %d", detected_lantern_count_);

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
      saveWayBack();
      break;
  case FLY_BACK:  // New state for flying back
            flyBack();
            break;
    }

    // Trigger FLY_BACK state when 4 or more lanterns are detected
    if (detected_lantern_count_ >= 4 && state_ != FLY_BACK) {
        state_ = FLY_BACK;
        ROS_INFO("Triggering FLY_BACK state: 4 or more lanterns detected.");
    }

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
    if (!paths_sent_) {
        std_msgs::Bool msg;
        msg.data = true;
        pub_controll_planner.publish(msg);
        paths_sent_ = true;
    } else if (false) {
        std_msgs::Bool msg;
        msg.data = false;
        pub_controll_planner.publish(msg);
        paths_sent_ = false;
        state_ = FLY_BACK;
    }
}

void StateMachine::flyBack() {
    ROS_INFO("Flying back to the starting point...");
    if (!paths_sent_) {
        publishPath(path_back_);
    } else if (closeToGoal()) {
        paths_sent_ = false;
        state_ = LAND;
    }
}

void StateMachine::land() {
    if (!paths_sent_) {
        publishPath(path_land_);
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