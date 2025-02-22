/*
 * CaveExplorerNode.cpp
 *
 * A C++ translation of the original Python "cave explorer" code,
 * plus a small addition to visualize each node itself as an arrow.
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Header.h>

#include <fla_msgs/GlobalPath.h>
#include <fla_msgs/GlobalPoint.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sensor_msgs/point_cloud2_iterator.h>
#include <limits>
#include <cmath>
#include <Eigen/Dense>
#include <thread>
#include <mutex>

class CaveExplorerNode
{
public:
  CaveExplorerNode()
  : running_(true),
    current_position_(Eigen::Vector3d::Zero()),
    current_velocity_(Eigen::Vector3d::Zero()),
    current_orientation_(Eigen::Matrix3d::Identity()),
    goal_point_set_(false)
  {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // Load parameters
    pnh.param("max_radius", max_radius_, 10.0);
    pnh.param("min_radius", min_radius_, 3.0);
    pnh.param("sampling_angle_deg", sampling_angle_deg_, 10);
    pnh.param("max_sampling_angle_deg", max_sampling_angle_deg_, 50);
    pnh.param("max_planning_distance", max_planning_distance_, 55.0);

    // Subscribers
    odom_sub_ = nh.subscribe("current_state_est", 1, &CaveExplorerNode::odomCallback, this);
    point_cloud_sub_ = nh.subscribe("/octomap_point_cloud_centers", 1, &CaveExplorerNode::pointCloudCallback, this);
    control_sub_ = nh.subscribe("control_planner", 1, &CaveExplorerNode::controlCallback, this);
    goal_point_sub_ = nh.subscribe("/goal_point", 1, &CaveExplorerNode::goalPointCallback, this);

    // Publishers
    path_pub_ = nh.advertise<visualization_msgs::MarkerArray>("path_marker_array", 1);
    global_path_pub_ = nh.advertise<fla_msgs::GlobalPath>("global_path", 1);
    marker_pub_ = nh.advertise<visualization_msgs::Marker>("sphere", 10);

    ROS_INFO("CaveExplorerNode initialized. Waiting for position update...");

    // Start exploration in a separate thread (like Python code calling explore() in __init__)
    explore_thread_ = std::thread(&CaveExplorerNode::explore, this);
  }

  ~CaveExplorerNode()
  {
    if(explore_thread_.joinable())
    {
      explore_thread_.join();
    }
  }

private:
  // goal_point_callback
  void goalPointCallback(const fla_msgs::GlobalPoint& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if(msg.point.x == 0.0 && msg.point.y == 0.0 && msg.point.z == 0.0)
    {
      goal_point_set_ = false;
      ROS_INFO("Goal point removed");
    }
    else
    {
      goal_point_ = msg;
      goal_point_set_ = true;
      ROS_INFO("Goal point received");
    }
  }

  // odom_callback
  void odomCallback(const nav_msgs::Odometry& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    current_position_(0) = msg.pose.pose.position.x;
    current_position_(1) = msg.pose.pose.position.y;
    current_position_(2) = msg.pose.pose.position.z;

    // Convert quaternion to rotation matrix
    tf2::Quaternion q(msg.pose.pose.orientation.x,
                      msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z,
                      msg.pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    for(int i = 0; i < 3; i++)
    {
      for(int j = 0; j < 3; j++)
      {
        current_orientation_(i, j) = m[i][j];
      }
    }

    current_velocity_(0) = msg.twist.twist.linear.x;
    current_velocity_(1) = msg.twist.twist.linear.y;
    current_velocity_(2) = msg.twist.twist.linear.z;

    ROS_INFO_ONCE("Position updated for the first time. Exploration will begin.");
  }

  // point_cloud_callback
  void pointCloudCallback(const sensor_msgs::PointCloud2& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // Mimic python: "if len(self.cloud) > 0: return"
    if(!cloud_.empty())
    {
      return;
    }

    // Read the entire point cloud
    std::vector<Eigen::Vector3d> full_cloud;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

    for(; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      float x = *iter_x;
      float y = *iter_y;
      float z = *iter_z;
      full_cloud.push_back(Eigen::Vector3d(x, y, z));
    }

    if(!positionIsValid())
    {
      // If we don't have a valid current position, just store the full cloud
      // and return.
      cloud_ = full_cloud;
      return;
    }

    // Filter points in front of the drone in some bounding region
    Eigen::Vector3d forward_vector = current_orientation_ * Eigen::Vector3d(1.0, 0.0, 0.0);

    std::vector<Eigen::Vector3d> filtered_cloud;
    for(const auto& point : full_cloud)
    {
      Eigen::Vector3d relative_position = point - current_position_;
      double dot_forward = relative_position.dot(forward_vector);

      // The python code used:
      // if (dot_forward > -50 and abs(dot_y) <= 100 and abs(dot_z) <= 100)
      double dot_y = relative_position.dot(Eigen::Vector3d(0.0, 1.0, 0.0));
      double dot_z = relative_position.dot(Eigen::Vector3d(0.0, 0.0, 1.0));

      if(dot_forward > -50.0 &&
         std::fabs(dot_y) <= 100.0 &&
         std::fabs(dot_z) <= 100.0)
      {
        filtered_cloud.push_back(point);
      }
    }

    cloud_ = filtered_cloud;
  }

  // control callback
  void controlCallback(const std_msgs::Bool& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = msg.data;
  }

  bool positionIsValid() const
  {
    // Basic check to see if we likely have an updated odom (Python used None).
    return (odom_sub_.getNumPublishers() > 0);
  }

  geometry_msgs::Point toGeometryMsgPoint(const Eigen::Vector3d& point)
  {
    geometry_msgs::Point p;
    p.x = point(0);
    p.y = point(1);
    p.z = point(2);
    return p;
  }

  void pubSphere(const Eigen::Vector3d& point, double radius)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();

    marker.ns = "sphere_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = point(0);
    marker.pose.position.y = point(1);
    marker.pose.position.z = point(2);
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = 2.0 * radius;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    marker_pub_.publish(marker);
  }

  // Arrow from father_point to point (path edges)
  void addArrowMarker(const Eigen::Vector3d& point,
                      const Eigen::Vector3d& father_point,
                      int id)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "path";
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    geometry_msgs::Point start = toGeometryMsgPoint(father_point);
    geometry_msgs::Point end   = toGeometryMsgPoint(point);

    marker.points.push_back(start);
    marker.points.push_back(end);

    // arrow scale
    marker.scale.x = 0.1; // shaft diameter
    marker.scale.y = 0.2; // head diameter
    marker.scale.z = 0.2; // head length

    marker.color.a = 0.8;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;

    path_markers_.markers.push_back(marker);
  }

  // Arrow that represents a node's own orientation
  // We'll place it at the node's position, and point it in "orientation" angle
  // (on the XY plane).
  void addNodeArrowMarker(const Eigen::Vector3d& point,
                          double orientation_angle,
                          int id)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "node_orientation";
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    geometry_msgs::Point start = toGeometryMsgPoint(point);
    // We'll define a small arrow of length 1 in the orientation direction
    double arrow_len = 1.0;
    double dx = arrow_len * std::cos(orientation_angle);
    double dy = arrow_len * std::sin(orientation_angle);

    geometry_msgs::Point end;
    end.x = point(0) + dx;
    end.y = point(1) + dy;
    end.z = point(2); // same Z

    marker.points.push_back(start);
    marker.points.push_back(end);

    // A bit smaller arrow for orientation
    marker.scale.x = 0.05;  // shaft diameter
    marker.scale.y = 0.1;   // head diameter
    marker.scale.z = 0.1;   // head length

    marker.color.a = 0.8;
    marker.color.r = 1.0f;  // red for orientation arrow
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;

    path_markers_.markers.push_back(marker);
  }

  double calculateMaxRadius(const Eigen::Vector3d& point)
  {
    if(cloud_.empty())
    {
      return max_radius_;
    }

    double nearest_distance = std::numeric_limits<double>::infinity();
    for(const auto& cpt : cloud_)
    {
      double dist = (cpt - point).norm();
      if(dist < nearest_distance)
      {
        nearest_distance = dist;
      }
    }
    double candidate_radius = nearest_distance - 0.1;
    return (candidate_radius < max_radius_) ? candidate_radius : max_radius_;
  }

  // Sample on a sphere around "center" in directions around "direction"
  std::vector<std::pair<Eigen::Vector3d, double>> sampleSphereDirected(
      const Eigen::Vector3d& center,
      const Eigen::Vector3d& direction,
      double radius,
      double max_angle_deg,
      double step_deg)
  {
    std::vector<std::pair<Eigen::Vector3d, double>> sampled_points;

    Eigen::Vector3d dir_norm = direction.normalized();
    double max_angle_rad = M_PI * (max_angle_deg / 180.0);
    double step_rad = M_PI * (step_deg / 180.0);

    for(double yaw = -max_angle_rad; yaw <= max_angle_rad + 1e-9; yaw += step_rad)
    {
      for(double pitch = -max_angle_rad; pitch <= max_angle_rad + 1e-9; pitch += step_rad)
      {
        // Rotation around z-axis (yaw)
        Eigen::Matrix3d R_yaw;
        R_yaw << cos(yaw), -sin(yaw), 0,
                 sin(yaw),  cos(yaw), 0,
                      0,         0,    1;

        // Rotation around y-axis (pitch)
        Eigen::Matrix3d R_pitch;
        R_pitch << cos(pitch),  0, sin(pitch),
                   0,           1,        0,
                  -sin(pitch), 0, cos(pitch);

        Eigen::Vector3d rotated_direction = R_yaw * (R_pitch * dir_norm);
        Eigen::Vector3d rotated_point = center + rotated_direction * radius;

        double max_radius_sample = calculateMaxRadius(rotated_point);
        sampled_points.push_back(std::make_pair(rotated_point, max_radius_sample));

        // Publish debug spheres
        pubSphere(rotated_point, max_radius_sample);
      }
    }
    return sampled_points;
  }

  bool closeToGoal(const Eigen::Vector3d& node_position, const Eigen::Vector3d& goal)
  {
    if(!goal_point_set_)
      return false;
    double dist = (goal - node_position).norm();
    return dist < max_radius_;
  }

  // We'll store info about each node
  struct PointNode
  {
    Eigen::Vector3d position;
    double radius;
    PointNode* father;
  };

  // Reconstruct path from final node to the beginning, then build a GlobalPath
  fla_msgs::GlobalPath reconstructPath(PointNode* point_node,
                                       double forced_orientation = 0.0,
                                       bool use_forced = false)
  {
    fla_msgs::GlobalPath global_path;
    global_path.header.stamp = ros::Time::now();
    global_path.header.frame_id = "world";

    std::vector<PointNode*> path_nodes;
    PointNode* current_node = point_node;

    while(current_node != nullptr)
    {
      path_nodes.push_back(current_node);
      current_node = current_node->father;
    }
    std::reverse(path_nodes.begin(), path_nodes.end());

    // Python logic: for i in range(1, len(path_nodes)-1)
    //   ...
    for(size_t i = 1; i + 1 < path_nodes.size(); i++)
    {
      PointNode* node = path_nodes[i];
      PointNode* next_node = path_nodes[i + 1];

      // father->child arrow
      addArrowMarker(next_node->position, node->position, i);

      // orientation
      double orient = std::atan2(next_node->position(1) - node->position(1),
                                 next_node->position(0) - node->position(0));
      // also visualize node itself as an arrow
      addNodeArrowMarker(node->position, orient, 100000 + i); // offset ID

      // populate global path
      fla_msgs::GlobalPoint gp;
      gp.point = toGeometryMsgPoint(node->position);
      gp.orientation = orient;
      gp.velocity = -1.0;
      gp.acceleration = -1.0;
      global_path.points.push_back(gp);
    }

    // If we have at least one point, optionally override orientation
    if(!global_path.points.empty() && use_forced)
    {
      global_path.points.back().orientation = forced_orientation;
    }

    return global_path;
  }

  void explore()
  {
    ros::Rate rate(10); // 10 Hz
    static PointNode* best_node_static = nullptr;

    while(ros::ok())
    {
      {
        std::lock_guard<std::mutex> lock(mutex_);

        if(!positionIsValid() || cloud_.empty())
        {
          ROS_INFO_ONCE("Waiting for initial position update or point cloud...");
        }
        else if(!running_)
        {
          // do nothing if not running
        }
        else
        {
          // If we had a best node from a previous iteration, check distance
          if(best_node_static != nullptr)
          {
            double dis = (current_position_ - best_node_static->father->position).norm();
            if(dis > 5.0)
            {
              // mimic the Python logic that just "continue"
              continue;
            }
          }

          Eigen::Vector3d start_position = current_position_;
          Eigen::Matrix3d start_orientation = current_orientation_;

          double base_dis_goal = 2.0 * max_planning_distance_;
          Eigen::Vector3d goal_point_vec;

          bool have_goal = goal_point_set_;
          if(have_goal)
          {
            goal_point_vec = Eigen::Vector3d(goal_point_.point.x,
                                             goal_point_.point.y,
                                             goal_point_.point.z);
          }
          else
          {
            // If we have no superior goal, place it far forward
            // (the original logic uses +X from orientation)
            Eigen::Vector3d forward = start_orientation * Eigen::Vector3d(1.0, 0.0, 0.0);
            goal_point_vec = start_position + base_dis_goal * forward;
          }

          // Build a list (vector) of PointNodes
          std::vector<PointNode*> point_nodes;
          PointNode* start_node = new PointNode();
          start_node->position = start_position;
          start_node->father = nullptr;
          start_node->radius = 0.0;
          point_nodes.push_back(start_node);

          start_node->radius = calculateMaxRadius(start_node->position);

          PointNode* best_node_local = start_node;
          double total_distance = 0.0;

          while(total_distance < max_planning_distance_)
          {
            if(closeToGoal(best_node_local->position, goal_point_vec))
            {
              // replicate python approach:
              PointNode* goal_node = new PointNode();
              goal_node->position = goal_point_vec;
              goal_node->father = best_node_local;
              goal_node->radius = 10.0;
              point_nodes.push_back(goal_node);

              // The python code then adds a second node
              PointNode* father_for_next = goal_node;
              PointNode* next_node = new PointNode();

              double cosval = std::cos(goal_point_.orientation);
              double sinval = std::sin(goal_point_.orientation);
              Eigen::Vector3d add_vec(10.0 * cosval,
                                      10.0 * sinval,
                                      goal_point_.point.z);
              next_node->position = best_node_local->position + add_vec;
              next_node->father = father_for_next;
              next_node->radius = 10.0;
              point_nodes.push_back(next_node);

              goal_point_set_ = false; // remove the global goal
              break;
            }

            // define forward direction
            Eigen::Vector3d forward_direction;
            if(best_node_local->father == nullptr)
            {
              // no father => use +X in local frame
              forward_direction = start_orientation * Eigen::Vector3d(1.0, 0.0, 0.0);
            }
            else
            {
              // father->child direction
              Eigen::Vector3d diff =
                  best_node_local->position - best_node_local->father->position;
              forward_direction = diff.normalized();
            }

            double used_max_sampling_angle_deg =
                (have_goal) ? (max_sampling_angle_deg_ + 100.0) : max_sampling_angle_deg_;

            auto sampled_points = sampleSphereDirected(best_node_local->position,
                                                       forward_direction,
                                                       best_node_local->radius,
                                                       used_max_sampling_angle_deg,
                                                       sampling_angle_deg_);

            double best_value = -std::numeric_limits<double>::infinity();
            PointNode* candidate_best = nullptr;

            for(auto& sp : sampled_points)
            {
              Eigen::Vector3d sp_point = sp.first;
              double sp_max_radius = sp.second;
              if(sp_max_radius < min_radius_)
              {
                continue;
              }

              double distance_to_goal = (sp_point - goal_point_vec).norm();
              double distance_from_start = (sp_point - start_position).norm();

              // value = 3 * distance_from_start - 5 * distance_to_goal + 18 * sp_max_radius
              double value = 3.0 * distance_from_start
                           - 5.0 * distance_to_goal
                           + 18.0 * sp_max_radius;

              if(value > best_value)
              {
                best_value = value;
                PointNode* new_node = new PointNode();
                new_node->position = sp_point;
                new_node->father = best_node_local;
                new_node->radius = sp_max_radius;
                candidate_best = new_node;
              }
            }

            if(candidate_best == nullptr)
            {
              // no valid sample => break
              break;
            }

            double dist_sp = (start_position - candidate_best->position).norm();
            if(dist_sp < 0.1)
            {
              candidate_best->radius = candidate_best->radius - 1.0;
              ROS_WARN("Start radius needs to be smaller!");
            }

            best_node_local = candidate_best;
            point_nodes.push_back(candidate_best);

            total_distance += (best_node_local->position - start_position).norm();
          }

          // Reconstruct and publish
          fla_msgs::GlobalPath global_path = reconstructPath(best_node_local);
          if(!global_path.points.empty())
          {
            global_path_pub_.publish(global_path);
            path_pub_.publish(path_markers_);
          }
          else
          {
            ROS_WARN("No path found.");
          }

          // Clear for the next iteration
          path_markers_.markers.clear();
          cloud_.clear();

          best_node_static = best_node_local;
        }
      }
      ros::spinOnce();
      rate.sleep();
    }
  }

private:
  // ROS
  ros::Subscriber odom_sub_;
  ros::Subscriber point_cloud_sub_;
  ros::Subscriber control_sub_;
  ros::Subscriber goal_point_sub_;

  ros::Publisher path_pub_;
  ros::Publisher global_path_pub_;
  ros::Publisher marker_pub_;

  // Parameters
  double max_radius_;
  double min_radius_;
  double max_planning_distance_;
  int sampling_angle_deg_;
  int max_sampling_angle_deg_;

  // Current state
  Eigen::Vector3d current_position_;
  Eigen::Vector3d current_velocity_;
  Eigen::Matrix3d current_orientation_;

  // PointCloud
  std::vector<Eigen::Vector3d> cloud_;

  // Visualization
  visualization_msgs::MarkerArray path_markers_;

  // Running state
  bool running_;

  // Goal point
  fla_msgs::GlobalPoint goal_point_;
  bool goal_point_set_;

  // Thread
  std::thread explore_thread_;
  std::mutex mutex_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cave_explorer");
  CaveExplorerNode node;
  ros::spin();
  return 0;
}
