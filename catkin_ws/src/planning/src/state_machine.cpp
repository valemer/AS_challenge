#include "state_machine.h"

#include "yaml-cpp/yaml.h"
#include "ros/package.h"
#include "std_srvs/Empty.h"

StateMachine::StateMachine() :
        hz_(10.0),
        current_velocity_(Eigen::Vector3d::Zero()),
        current_pose_(Eigen::Affine3d::Identity())
{
    // publisher
    pub_global_path_ = nh_.advertise<fla_msgs::GlobalPath>("/global_path", 0);

    // subscriber
    sub_odom_ = nh_.subscribe("/current_state_est", 1, &StateMachine::uavOdomCallback, this);

    // main loop timer
    timer_ = nh_.createTimer(ros::Rate(hz_), &StateMachine::mainLoop, this);

    // Service
    ros::service::waitForService("/octomap_server/reset");
    client = nh_.serviceClient<std_srvs::Empty>("/octomap_server/reset");
}

// Callback to get current Pose of UAV
void StateMachine::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {

    // store current position in our planner
    tf::poseMsgToEigen(odom->pose.pose, current_pose_);

    // store current velocity
    tf::vectorMsgToEigen(odom->twist.twist.linear, current_velocity_);
}

void StateMachine::mainLoop(const ros::TimerEvent& t) {
  switch(state_) {
    case TAKE_OFF:
      takeOff();
      break;
    case GO:
      flyToCave();
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
  if (paths_sent_ < 3) {
      ROS_INFO_NAMED("state_machine", "Start take-off!");
      loadAndSendPath("takeoff");
      paths_sent_++;
  } else if (closeToGoal()) {
    ROS_INFO_NAMED("state_machine", "Close to take-off goal!");
    paths_sent_ = 0;
    state_ = GO;
  }
}

void StateMachine::flyToCave() {
    if (!paths_sent_) {
        loadAndSendPath("flyToCave");
        paths_sent_ = true;
    } else if (closeToGoal()) {
        paths_sent_ = false;
        state_ = EXPLORED;
        std_srvs::Empty srv;
        if (client.call(srv)) {
            ROS_INFO("OctoMap reset successfully.");
        } else {
            ROS_ERROR("Failed to reset OctoMap.");
        }
    }
}

void StateMachine::loadAndSendPath(std::string path) {
    YAML::Node config = YAML::LoadFile(ros::package::getPath("planning") +
        "/config/state_machine_config.yaml");

    fla_msgs::GlobalPath global_path;
    fla_msgs::GlobalPoint global_point;
    for (const auto& point : config[path]) {
        global_point.point.x = point[0].as<double>();
        global_point.point.y = point[1].as<double>();
        global_point.point.z = point[2].as<double>();
        global_point.orientation = point[3].as<double>();
        global_point.velocity = point[4].as<double>();
        global_point.acceleration = point[5].as<double>();
        global_path.points.push_back(global_point);
    }
    pub_global_path_.publish(global_path);

    current_goal_ << global_point.point.x, global_point.point.y, global_point.point.z;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "state_machine");
    ROS_INFO_NAMED("state_machine", "State Machine started!");
    StateMachine n;
    ros::spin();
}