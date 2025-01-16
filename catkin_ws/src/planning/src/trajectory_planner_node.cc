#include <trajectory_planner_node.h>

#include "ros/package.h"
#include "yaml-cpp/yaml.h"

TrajectoryPlanner::TrajectoryPlanner() :
        max_v_(5),
        max_a_(2),
        current_velocity_(Eigen::Vector3d::Zero()),
        current_pose_(Eigen::Affine3d::Identity())
        {

    nh_.getParam("dynamic_params/max_v", max_v_);
    nh_.getParam("dynamic_params/max_a", max_a_);

    // publisher
    pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0);
    pub_trajectory_ = nh_.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory", 0);

    // subscriber
    sub_global_path_ = nh_.subscribe("/planned_path", 1, &TrajectoryPlanner::planTrajectory, this);
    sub_odom_ = nh_.subscribe("/current_state_est", 1, &TrajectoryPlanner::uavOdomCallback, this);

}

// Callback to get current Pose of UAV
void TrajectoryPlanner::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {

    // store current position in our planner
    tf::poseMsgToEigen(odom->pose.pose, current_pose_);

    // store current velocity
    tf::vectorMsgToEigen(odom->twist.twist.linear, current_velocity_);
}

// Method to set maximum speed.
void TrajectoryPlanner::setMaxSpeed(const double max_v) {
    max_v_ = max_v;
}

// Plans a trajectory from the current position to a goal position
void TrajectoryPlanner::planTrajectory(const nav_msgs::Path::ConstPtr& globalPath) {

    const int dimension = 4;
    mav_trajectory_generation::Vertex::Vector vertices;
    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;

    // Configure start point
    mav_trajectory_generation::Vertex start(dimension);
    Eigen::Vector4d start_pos_4d, start_vel_4d;
    start_pos_4d << current_pose_.translation(),
        mav_msgs::yawFromQuaternion(
            (Eigen::Quaterniond)current_pose_.rotation());
    start_vel_4d << current_velocity_, 0.0;
    start.makeStartOrEnd(start_pos_4d, derivative_to_optimize);
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, start_vel_4d);
    vertices.push_back(start);

    // Add waypoints from global path
    for (size_t i = 0; i < globalPath->poses.size(); ++i) {
        const geometry_msgs::PoseStamped& pose = globalPath->poses[i];
        Eigen::Vector4d pos;
        pos << pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
            mav_msgs::yawFromQuaternion(Eigen::Quaterniond(
                pose.pose.orientation.w, pose.pose.orientation.x,
                pose.pose.orientation.y, pose.pose.orientation.z));

        mav_trajectory_generation::Vertex waypoint(dimension);
        waypoint.addConstraint(mav_trajectory_generation::derivative_order::POSITION, pos);

        vertices.push_back(waypoint);
    }

    // Estimate segment times
    std::vector<double> segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

    // Set up optimization
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;
    const int N = 10;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);
    opt.optimize();

    // Get and publish trajectory
    mav_trajectory_generation::Trajectory trajectory;
    opt.getTrajectory(&trajectory);
    publishTrajectory(trajectory);
}

bool TrajectoryPlanner::publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory){
    visualization_msgs::MarkerArray markers;
    double distance = 0.2;
    std::string frame_id = "world";

    drawMavTrajectory(trajectory, distance, frame_id, &markers);
    pub_markers_.publish(markers);

    mav_planning_msgs::PolynomialTrajectory4D msg;
    trajectoryToPolynomialTrajectoryMsg(trajectory, &msg);
    msg.header.frame_id = "world";
    pub_trajectory_.publish(msg);

    return true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "trajectory_planner_node");
    ROS_INFO_NAMED("trajectory_planner", "Trajectory Planner started!");
    TrajectoryPlanner n;
    ros::spin();
}
