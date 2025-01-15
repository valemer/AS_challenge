#include "LanternDetection.h"


LanternDetectionNode::LanternDetectionNode() :
sync_(image_sub_, cloud_sub_, 10), tf_listener_(tf_buffer_),
min_distance_(10.0),
min_detections_(20),
min_yellow_points_(50) {

    // Subscriptions
    image_sub_.subscribe(nh_, "/unity_ros/Quadrotor/Sensors/SemanticCamera/image_raw", 1);
    cloud_sub_.subscribe(nh_, "/camera/depth/points", 1);
    sync_.registerCallback(boost::bind(&LanternDetectionNode::callback, this, _1, _2));

    // Publisher
    pub_world_coordinates_ = nh_.advertise<geometry_msgs::PointStamped>("/lantern_detection_world_coordinates", 1);
    pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("/lantern_detection_marker_array", 1);

    // Params
    nh_.getParam("lantern_detection/min_distance", min_distance_);
    nh_.getParam("lantern_detection/min_detections", min_detections_);
    nh_.getParam("lantern_detection/min_yellow_points", min_yellow_points_);
}

double LanternDetectionNode::distance(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
}

void LanternDetectionNode::updateTrackedObjects(const geometry_msgs::Point& detected_point) {
    bool matched = false;
    for (auto& [id, obj] : tracked_objects_) {
        if (distance(obj.position, detected_point) < min_distance_) {
            obj.detections.push_back(detected_point);
            obj.position = computeCentroid(obj.detections);
            matched = true;
            break;
        }
    }

    if (!matched) {
        int new_id = tracked_objects_.size();
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
            marker.scale.x = 0.5; // Big red ball
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker_array.markers.push_back(marker);
        }
    }

    pub_markers_.publish(marker_array);
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
    pcl::fromROSMsg(*cloud_msg, *cloud);

    cv::Mat hsv_image;
    cv::cvtColor(cv_image->image, hsv_image, cv::COLOR_BGR2HSV);

    cv::Scalar lower_yellow(20, 100, 100);
    cv::Scalar upper_yellow(30, 255, 255);

    cv::Mat mask;
    cv::inRange(hsv_image, lower_yellow, upper_yellow, mask);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);



    double max_area = 0;
    std::vector<cv::Point> largest_contour;

    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > max_area) {
            max_area = area;
            largest_contour = contour;
        }
    }

    ROS_INFO("Largest contur: %f", max_area);

    if (max_area < min_yellow_points_)
        return;

    if (!largest_contour.empty()) {
        cv::Moments moments = cv::moments(largest_contour);
        if (moments.m00 > 0) {
            int x = static_cast<int>(moments.m10 / moments.m00);
            int y = static_cast<int>(moments.m01 / moments.m00);
            cv::circle(cv_image->image, cv::Point(x, y), 5, CV_RGB(255, 0, 0), -1);
            ROS_DEBUG("Biggest yellow point at: (%d, %d)", x, y);

            int width = cloud->width;
            int height = cloud->height;
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
