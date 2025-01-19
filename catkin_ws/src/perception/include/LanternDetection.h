#pragma once

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h> // for all lantern locations this is added
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <unordered_map>
#include <cmath>


struct TrackedObject {
    geometry_msgs::Point position;
    std::vector<geometry_msgs::Point> detections;
};

class LanternDetectionNode {
private:
    ros::NodeHandle nh_;

    message_filters::Subscriber<sensor_msgs::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::PointCloud2> sync_;
    ros::Publisher pub_all_lanterns_;  // Publisher for all detected lantern locations
    ros::Publisher pub_world_coordinates_;
    ros::Publisher pub_markers_; // For RViz visualization
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::unordered_map<unsigned long, TrackedObject> tracked_objects_;
    double min_distance_; // Minimum distance to consider two objects separate
    int min_detections_;   // Minimum detections to consider a valid object
    int min_yellow_points_;

public:
    LanternDetectionNode();

    static double distance(const geometry_msgs::Point& a, const geometry_msgs::Point& b);

    void updateTrackedObjects(const geometry_msgs::Point& detected_point);

    geometry_msgs::Point computeCentroid(const std::vector<geometry_msgs::Point>& points);

    void publishMarkers();

    void callback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
};
