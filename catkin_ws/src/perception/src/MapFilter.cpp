#include "MapFilter.h"

DynamicHeightFilterNode::DynamicHeightFilterNode()
        : nh_("~"), resolution_(2.0), grid_size_(100.0), width_(grid_size_ / resolution_), height_(grid_size_ / resolution_) {

        // Subscribe to UAV odometry
        odom_sub_ = nh_.subscribe("/current_state_est", 1, &DynamicHeightFilterNode::uavOdomCallback, this);

        // Subscribe to point cloud
        cloud_sub_ = nh_.subscribe("/octomap_point_cloud_centers", 1, &DynamicHeightFilterNode::cloudCallback, this);

        // Publish OccupancyGrid
        occupancy_grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/occupancy_grid", 1);

        // Publish filtered point cloud
        filtered_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/filtered_point_cloud", 1);
}

void DynamicHeightFilterNode::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    // Store the current pose of the UAV
    current_pose_ = odom->pose.pose;
}

void DynamicHeightFilterNode::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // Convert to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Create an OccupancyGrid message
    nav_msgs::OccupancyGrid occupancy_grid;
    occupancy_grid.header.stamp = ros::Time::now();
    occupancy_grid.header.frame_id = "world";
    occupancy_grid.info.resolution = resolution_;
    occupancy_grid.info.width = width_;
    occupancy_grid.info.height = height_;
    occupancy_grid.info.origin.position.x = current_pose_.position.x - grid_size_ / 2.0;
    occupancy_grid.info.origin.position.y = current_pose_.position.y - grid_size_ / 2.0;
    occupancy_grid.info.origin.position.z = 0.0;
    occupancy_grid.info.origin.orientation.w = 1.0;

    // Initialize the grid with unknown (-1)
    occupancy_grid.data.assign(width_ * height_, 100);

    // Create a filtered point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Dynamically set height filtering based on UAV's current position
    double uav_height = current_pose_.position.z;
    double height_offset = 2.0; // Allowable offset above/below UAV
    double min_height = uav_height - height_offset;
    double max_height = uav_height + height_offset;

    // Populate the occupancy grid
    for (const auto& point : cloud->points) {
        // Ignore NaN points
        if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
            continue;
        }

        // Filter points based on height relative to UAV
        if (point.z < min_height || point.z > max_height) {
            continue;
        }

        // Convert world coordinates to grid coordinates
        int grid_x = (point.x - current_pose_.position.x + grid_size_ / 2.0) / resolution_;
        int grid_y = (point.y - current_pose_.position.y + grid_size_ / 2.0) / resolution_;

        // Check bounds
        if (grid_x >= 0 && grid_x < width_ && grid_y >= 0 && grid_y < height_) {
            int index = grid_y * width_ + grid_x;
            occupancy_grid.data[index] = 0; // Mark as occupied
        }
    }

    // Publish the occupancy grid
    occupancy_grid_pub_.publish(occupancy_grid);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dynamic_height_filter_node");
    DynamicHeightFilterNode node;
    ros::spin();
    return 0;
}
