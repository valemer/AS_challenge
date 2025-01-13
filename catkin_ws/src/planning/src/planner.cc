#include <planner.h>
#include "yaml-cpp/yaml.h"
#include "ros/package.h"

BasicPlanner::BasicPlanner(ros::NodeHandle& nh) :
        nh_(nh),
        max_v_(5),
        max_a_(2),
        current_velocity_(Eigen::Vector3d::Zero()),
        current_pose_(Eigen::Affine3d::Identity()),
        hz(10.0){

    nh.getParam("dynamic_params/max_v", max_v_);
    nh.getParam("dynamic_params/max_a", max_a_);

    // create publisher for RVIZ markers
    pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0);
    pub_trajectory_ = nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory", 0);

    // subscriber for Odometry
    sub_odom_ = nh.subscribe("/current_state_est", 1, &BasicPlanner::uavOdomCallback, this);
}

// Callback to get current Pose of UAV
void BasicPlanner::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {

    // store current position in our planner
    tf::poseMsgToEigen(odom->pose.pose, current_pose_);

    // store current velocity
    tf::vectorMsgToEigen(odom->twist.twist.linear, current_velocity_);
}

// Method to set maximum speed.
void BasicPlanner::setMaxSpeed(const double max_v) {
    max_v_ = max_v;
}

// Plans a trajectory from the current position to the a goal position and velocity
// we neglect attitude here for simplicity
bool BasicPlanner::planTrajectory(mav_trajectory_generation::Trajectory* trajectory) {

    // 3 Dimensional trajectory => through carteisan space, no orientation
    const int dimension = 4;

    // Array for all waypoints and their constrains
    mav_trajectory_generation::Vertex::Vector vertices;

    // Optimze up to 4th order derivative (SNAP)
    const int derivative_to_optimize =
            mav_trajectory_generation::derivative_order::SNAP;

    // we have 2 vertices:
    // Start = current position
    // end = desired position and velocity
    mav_trajectory_generation::Vertex start(dimension), middle(dimension);


    /******* Configure start point *******/
    // set start point constraints to current position and set all derivatives to zero
    Eigen::Vector4d start_pos_4d, start_vel_4d;
    start_pos_4d << current_pose_.translation(),
        mav_msgs::yawFromQuaternion(
            (Eigen::Quaterniond)current_pose_.rotation());
    start_vel_4d << current_velocity_, 0.0;
    start.makeStartOrEnd(start_pos_4d,
                         derivative_to_optimize);

    // set start point's velocity to be constrained to current velocity
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                        start_vel_4d);

    // add waypoint to list
    vertices.push_back(start);

    /******* Configure trajectory *******/
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  To Do: Set up trajectory waypoints
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //
    // In this section, you need to
    // - load waypoint definition (pos, vel, acc) per dimension from param file
    // - dynamically set constraints for each (and only where needed)
    // - push waypoints to vertices
    //
    // ~~~~ begin solution
    //
    //     **** FILL IN HERE ***
    YAML::Node config = YAML::LoadFile(ros::package::getPath("basic_waypoint_pkg") +
        "/config/trajectory_config.yaml");

    // Check and load the vector of vectors
    if (config["waypoints"]) {
        for (auto it = config["waypoints"].begin(); it != config["waypoints"].end(); ++it) {
            const auto& waypoint = *it;

            int i = 0;
            Eigen::Vector4d pos;
            double vel = -1;
            double acc = -1;
            for (const auto& value : waypoint) {
                if (i < 4)
                    pos[i] = value.as<double>();
                else if (i < 5)
                    vel = value.as<double>();
                else
                    acc = value.as<double>();
                i++;
            }
            // Check if it's the last element
            if (std::next(it) != config["waypoints"].end()) {
                middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, pos);
                if (vel == 0) {
                    Eigen::Vector4d vel_vec;
                    vel_vec << 0.0, 0.0, 0.0, 0.0;
                    middle.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, vel_vec);
                }

                if (acc == 0) {
                    Eigen::Vector4d acc_vec;
                    acc_vec << 0.0, 0.0, 0.0, 0.0;
                    middle.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, acc_vec);
                }

                vertices.push_back(middle);

                if (vel == 0) {
                    middle.removeConstraint(mav_trajectory_generation::derivative_order::VELOCITY);
                }
                if (acc == 0) {
                    middle.removeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION);
                }
            } else {
                /******* Configure end point *******/
                // set end point constraints to desired position and set all derivatives to zero
                middle.makeStartOrEnd(pos,
                                   derivative_to_optimize);
                Eigen::Vector4d vel_vec;
                vel_vec << 0.0, 0.0, 0.0, 0.0;
                middle.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                                  vel_vec);
                vertices.push_back(middle);
            }
        }
    }
    //
    // ~~~~ end solution
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //                                 end
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // setimate initial segment times
    std::vector<double> segment_times;
    segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

    // Set up polynomial solver with default params
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;

    // set up optimization problem
    const int N = 10;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

    // constrain velocity and acceleration
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);

    // solve trajectory
    opt.optimize();

    // get trajectory as polynomial parameters
    opt.getTrajectory(&(*trajectory));

    return true;
}

bool BasicPlanner::publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory){
    // send trajectory as markers to display them in RVIZ
    visualization_msgs::MarkerArray markers;
    double distance = 0.2; // Distance by which to separate additional markers. Set 0.0 to disable.
    std::string frame_id = "world";

    mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
    pub_markers_.publish(markers);

    // send trajectory to be executed on UAV
    mav_planning_msgs::PolynomialTrajectory4D msg;
    mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory, &msg);
    msg.header.frame_id = "world";
    pub_trajectory_.publish(msg);

    return true;
}
