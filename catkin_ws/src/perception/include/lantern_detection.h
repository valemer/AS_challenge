#pragma once

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <unordered_map>


struct TrackedObject {
    geometry_msgs::Point position;
    std::vector<geometry_msgs::Point> detections;
};

class LanternDetectionNode {
    ros::NodeHandle nh_;

    // Subscriber
    message_filters::Subscriber<sensor_msgs::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;

    // Publisher
    ros::Publisher pub_all_lanterns_;
    ros::Publisher pub_world_coordinates_;
    ros::Publisher pub_markers_;

    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::PointCloud2> sync_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::unordered_map<unsigned long, TrackedObject> tracked_objects_;

    double max_distance_same_detection_;            // max distance in m for same detection
    double min_distance_new_detection_;             // min distance in m for new detection
    int min_detections_;                            // min detections at location to be a valid detection
    int min_detection_points_in_img_;               // min number of detection points in camera image

public:
    /**
     * Constructor
     */
    LanternDetectionNode();

    /**
     * Helper function to compute distance between two points
     *
     * @param a Point
     * @param b Point
     * @return double distance in m
     */
    static double distance(const geometry_msgs::Point& a, const geometry_msgs::Point& b);

    /**
     * Update function for new detected point in global list of detections grouped by location
     *
     * @param detected_point
     */
    void updateTrackedObjects(const geometry_msgs::Point& detected_point);

    /**
     * Helper function to compute Centroid of cloud cluster
     *
     * @param points vector ot Point
     * @return Point as centroid of points
     */
    static geometry_msgs::Point computeCentroid(const std::vector<geometry_msgs::Point>& points);

    /**
     * Helper function to publish detections to RVIZ
     */
    void publishMarkers();

    /**
     * Callback to get synced Image with lantern segmentation and Point cloud
     *
     * @param image_msg Ptr to Image
     * @param cloud_msg Ptr to PointCloud2
     */
    void callback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
};
