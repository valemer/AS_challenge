#include "lantern_detection.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>


LanternDetectionNode::LanternDetectionNode() :
sync_(image_sub_, cloud_sub_, 10), tf_listener_(tf_buffer_),
max_distance_same_detection_(10.0),
min_distance_new_detection_(40.0),
min_detections_(10),
min_detection_points_in_img_(50){

    // Subscriptions
    image_sub_.subscribe(nh_, "/unity_ros/Quadrotor/Sensors/SemanticCamera/image_raw", 1);
    cloud_sub_.subscribe(nh_, "/camera/depth/points", 1);
    sync_.registerCallback(boost::bind(&LanternDetectionNode::callback, this, _1, _2));

    // Publishers
    pub_world_coordinates_ = nh_.advertise<geometry_msgs::PointStamped>("/lantern_detection_world_coordinates", 1);
    pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/lantern_detection_marker_array", 1);
    pub_all_lanterns_ = nh_.advertise<geometry_msgs::PoseArray>("/all_detected_lantern_locations", 1);

    // Params
    nh_.getParam("lantern_detection/max_distance_same_detection", max_distance_same_detection_);
    nh_.getParam("lantern_detection/min_distance_new_detection", min_distance_new_detection_);
    nh_.getParam("lantern_detection/min_detections", min_detections_);
    nh_.getParam("lantern_detection/min_yellow_points", min_detection_points_in_img_);
}

double LanternDetectionNode::distance(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
}

void LanternDetectionNode::updateTrackedObjects(const geometry_msgs::Point& detected_point) {
    bool matched = false;
    for (auto& [id, obj] : tracked_objects_) {
        // Check if detection in same region
        if (distance(obj.position, detected_point) < max_distance_same_detection_) {
            obj.detections.push_back(detected_point);
            obj.position = computeCentroid(obj.detections);
            matched = true;
            break;
        }
        // Check if too close to be considered as new detection
        if (distance(obj.position, detected_point) < min_distance_new_detection_) {
            return;
        }
    }

    // Add new detection
    if (!matched) {
        const auto new_id = tracked_objects_.size();
        tracked_objects_[new_id] = {detected_point, {detected_point}};
    }
}

geometry_msgs::Point LanternDetectionNode::computeCentroid(const std::vector<geometry_msgs::Point>& points) {
    geometry_msgs::Point centroid;
    centroid.x = 0.0;
    centroid.y = 0.0;
    centroid.z = 0.0;
    for (const auto& point : points) {
        centroid.x += point.x;
        centroid.y += point.y;
        centroid.z += point.z;
    }
    centroid.x /= points.size();
    centroid.y /= points.size();
    centroid.z /= points.size();
    return centroid;
}

void LanternDetectionNode::publishMarkers() {
    visualization_msgs::MarkerArray marker_array;

    geometry_msgs::PoseArray all_lanterns_msg;
    all_lanterns_msg.header.stamp = ros::Time::now();
    all_lanterns_msg.header.frame_id = "world";

    for (const auto& [id, obj] : tracked_objects_) {
        if (obj.detections.size() >= min_detections_) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.ns = "yellow_points";
            marker.id = id;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = obj.position.x;
            marker.pose.position.y = obj.position.y;
            marker.pose.position.z = obj.position.z;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 5.0;
            marker.scale.y = 5.0;
            marker.scale.z = 5.0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker_array.markers.push_back(marker);

            geometry_msgs::Pose pose;
            pose.position = obj.position;
            pose.orientation.w = 1.0;
            all_lanterns_msg.poses.push_back(pose);
        }
    }

    pub_markers_.publish(marker_array);
    pub_all_lanterns_.publish(all_lanterns_msg);
}

void LanternDetectionNode::callback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    cv_bridge::CvImagePtr cv_image;
    try {
        cv_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    fromROSMsg(*cloud_msg, *cloud);

    cv::Mat hsv_image;
    cvtColor(cv_image->image, hsv_image, cv::COLOR_BGR2HSV);

    // segmentation color
    cv::Scalar lower_yellow(20, 100, 100);
    cv::Scalar upper_yellow(30, 255, 255);

    // create mask based on 'yellow'
    cv::Mat mask;
    inRange(hsv_image, lower_yellow, upper_yellow, mask);

    // detect yellow regions
    std::vector<std::vector<cv::Point>> contours;
    findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double max_area = 0;
    std::vector<cv::Point> largest_contour;

    // choose the biggest region
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > max_area) {
            max_area = area;
            largest_contour = contour;
        }
    }

    if (max_area < min_detection_points_in_img_)
        return;

    // project point of biggest point in 3d
    if (!largest_contour.empty()) {
        cv::Moments moments = cv::moments(largest_contour);
        if (moments.m00 > 0) {
            int x = static_cast<int>(moments.m10 / moments.m00);
            int y = static_cast<int>(moments.m01 / moments.m00);
            circle(cv_image->image, cv::Point(x, y), 5, CV_RGB(255, 0, 0), -1);
            ROS_DEBUG("Biggest yellow point at: (%d, %d)", x, y);

            int width = cloud->width;
            int index = y * width + x;
            if (index >= 0 && index < cloud->points.size()) {
                pcl::PointXYZ pcl_point = cloud->points[index];

                if (!std::isnan(pcl_point.x) && !std::isnan(pcl_point.y) && !std::isnan(pcl_point.z)) {
                    geometry_msgs::PointStamped lidar_point, world_point;
                    lidar_point.header.frame_id = "Quadrotor/Sensors/DepthCamera";
                    lidar_point.point.x = pcl_point.x;
                    lidar_point.point.y = pcl_point.y;
                    lidar_point.point.z = pcl_point.z;

                    try {
                        world_point = tf_buffer_.transform(lidar_point, "world", ros::Duration(1.0));
                        pub_world_coordinates_.publish(world_point);

                        updateTrackedObjects(world_point.point);
                        publishMarkers();
                    } catch (tf2::TransformException& ex) {
                        ROS_WARN("TF Exception: %s", ex.what());
                    }
                }
            }
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lantern_detection_node");
    LanternDetectionNode node;
    ros::spin();
    return 0;
}
