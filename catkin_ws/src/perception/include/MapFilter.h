#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

class DynamicHeightFilterNode {
public:
    DynamicHeightFilterNode();

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber cloud_sub_;
    ros::Publisher occupancy_grid_pub_;
    ros::Publisher filtered_cloud_pub_;

    double resolution_;
    double grid_size_;
    int width_;
    int height_;

    geometry_msgs::Pose current_pose_;

    void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom);

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

};