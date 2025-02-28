#pragma once

#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/OcTree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

class JunctionDetection {
    ros::NodeHandle nh_;

    // Subscriber
    ros::Subscriber octomap_sub_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_control_;

    // Publisher
    ros::Publisher clustered_junction_pub_;
    ros::Publisher cluster_centroids_pub_;
    ros::Publisher goal_point_pub_;

    octomap::OcTree* latest_octree_ = nullptr;  // saves the received PointCloud
    geometry_msgs::Pose current_pose_;          // Current odom
    bool running_;                              // node turned on or off
    double cave_entry_clearance_;               // Distance from cave entry to not detect it as a junction
    double update_threshold_;                   // Distance between old and new junction to be considered as the same
    int skipped_;                               // Skipped PointCloud msgs
    int map_skips_;                             // Num of PointCloud msgs, which should be skipped between iterations

    Eigen::Vector3d cave_entry_;                // Saved cave entry coordinates as soon as running_ == true

    float search_radius_;                       // Search radius for free cells, which also have an 'unknown' neighbor
    int min_neighbors_;                         // Min # of neighbors to consider as 'important' for later clustering

    std::vector<pcl::PointXYZ> detected_centroids_; // Store centroids in the order they were found

public:
    /**
     * Constructor of ROS Node JunctionDetection
     */
    JunctionDetection();

    /**
     * Destructor
     */
    ~JunctionDetection();

    /**
     * Callback for Subscription to turn node on and off
     *
     * @param msg Ptr to Bool
     */
    void controlCallback(const std_msgs::Bool::ConstPtr& msg);

    /**
     * Callback for Subscription to latest odom
     *
     * @param odom Ptr to Odometry
     */
    void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom);

    /**
     * Callback for Subscription to latest Octomap
     *
     * @param msg Ptr to Octomap
     */
    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg);

    /**
     * Checks if 'point' has at least 'min_neighbors' in 'radius'
     *
     * @param kdtree KdTreeFLANN
     * @param point PointXYZ
     * @param radius radius in m to be checked
     * @param min_neighbors min # of neighbors to consider 'point' as important
     * @return
     */
    static bool hasEnoughNeighbors(const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree,
                                   const pcl::PointXYZ& point,
                                   float radius,
                                   int min_neighbors);

    /**
     * Detect junctions in given 'tree'
     *
     * @param tree OcTree
     */
    void detectJunctionsAndPublish(octomap::OcTree& tree);

    /**
     * Cluster the detected junctions, provided in 'junctions'
     *
     * @param junctions PointCloud of relevant junctions
     */
    void clusterJunctionsAndPublish(const pcl::PointCloud<pcl::PointXYZ>::Ptr& junctions);

    /**
     * Update or remove old centroids based on new detections and keep order
     *
     * @param new_centroids PointCloud of detected junctions in latest Octomap PointCloud
     */
    void updateDetectedCentroids(const pcl::PointCloud<pcl::PointXYZ>::Ptr & new_centroids);

    /**
     * Publisher to send latest detected junction
     *
     * @param point PointXYZ
     */
    void publishGoalPoint(const pcl::PointXYZ& point) const;
};
