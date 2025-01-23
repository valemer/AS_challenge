/*
*	File: rrt_star_interactive_node.cpp
*	---------------
*   rrt* interactive with separate start and goal points listeners
*/
#include <ros/ros.h>
#include <global_planner/rrtStarOctomap.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <thread>

using std::cout;
using std::endl;

ros::Publisher plannedPathPub; // Publisher for the planned path
bool newStart = false;         // Flag for new start point
bool newGoal = false;          // Flag for new goal point
std::vector<double> startPoint{0, 0, 0};
std::vector<double> goalPoint{0, 0, 0};

// Callback for receiving start points
void startPointsCB(const geometry_msgs::Point::ConstPtr& msg) {
    startPoint[0] = msg->x;
    startPoint[1] = msg->y;
    startPoint[2] = msg->z; // Read height dynamically
    newStart = true;
    cout << "[Planner Node]: Received new start point: (" << startPoint[0] << ", " << startPoint[1] << ", " << startPoint[2] << ")" << endl;
}

// Callback for receiving goal points
void goalPointsCB(const geometry_msgs::Point::ConstPtr& msg) {
    goalPoint[0] = msg->x;
    goalPoint[1] = msg->y;
    goalPoint[2] = msg->z; // Read height dynamically
    newGoal = true;
    cout << "[Planner Node]: Received new goal point: (" << goalPoint[0] << ", " << goalPoint[1] << ", " << goalPoint[2] << ")" << endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "RRT*_test_node");
    ros::NodeHandle nh;

    // Subscribers for start and goal points
    ros::Subscriber startSub = nh.subscribe("/fly_back_start_points", 1, startPointsCB);
    ros::Subscriber goalSub = nh.subscribe("/fly_back_goal_points", 1, goalPointsCB);

    // Publisher for the planned path
    plannedPathPub = nh.advertise<nav_msgs::Path>("/planned_path", 1);

    const int N = 3; // Dimension
    globalPlanner::rrtStarOctomap<N> rrtStarPlanner(nh);
    cout << rrtStarPlanner << endl;

    ros::Rate r(10);
    while (ros::ok()) {
        if (newStart && newGoal) {
            // Update the octomap
            rrtStarPlanner.updateMap();
            // Update planner with new start and goal points
            rrtStarPlanner.updateStart(startPoint);
            rrtStarPlanner.updateGoal(goalPoint);
            newStart = false; // Reset start flag
            newGoal = false;  // Reset goal flag
            nav_msgs::Path path;
            rrtStarPlanner.makePlan(path);
            // Publish the planned path
            plannedPathPub.publish(path);
            cout << "[Planner Node]: Planned path published." << endl;
        }


        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
