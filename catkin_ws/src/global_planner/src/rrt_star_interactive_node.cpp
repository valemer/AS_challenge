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
#include <mutex>

using std::cout;
using std::endl;

ros::Publisher plannedPathPub; // Publisher for the planned path
std::mutex planningMutex;      // Mutex for thread safety
bool newStart = false;         // Flag for new start point
bool newGoal = false;          // Flag for new goal point
std::vector<double> startPoint{0, 0, 0};
std::vector<double> goalPoint{0, 0, 0};

// Callback for receiving start points
void startPointsCB(const geometry_msgs::Point::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(planningMutex);
    startPoint[0] = msg->x;
    startPoint[1] = msg->y;
    startPoint[2] = msg->z; // Read height dynamically
    newStart = true;
    cout << "[Planner Node]: Received new start point: (" << startPoint[0] << ", " << startPoint[1] << ", " << startPoint[2] << ")" << endl;
}

// Callback for receiving goal points
void goalPointsCB(const geometry_msgs::Point::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(planningMutex);
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
    ros::Subscriber startSub = nh.subscribe("/start_points", 1, startPointsCB);
    ros::Subscriber goalSub = nh.subscribe("/goal_points", 1, goalPointsCB);

    // Publisher for the planned path
    plannedPathPub = nh.advertise<nav_msgs::Path>("/planned_path", 1);

    const int N = 3; // Dimension
    globalPlanner::rrtStarOctomap<N> rrtStarPlanner(nh);
    cout << rrtStarPlanner << endl;

    ros::Rate r(10);
    while (ros::ok()) {
        bool readyToPlan = false;
        {
            std::lock_guard<std::mutex> lock(planningMutex);
            if (newStart && newGoal) {
                // Update planner with new start and goal points
                rrtStarPlanner.updateStart(startPoint);
                rrtStarPlanner.updateGoal(goalPoint);

                newStart = false; // Reset start flag
                newGoal = false;  // Reset goal flag
                readyToPlan = true;
            }
        }

        if (readyToPlan) {
            // Generate waypoint path
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
