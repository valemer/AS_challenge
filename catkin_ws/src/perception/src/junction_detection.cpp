#include "junction_detection.h"

#include <eigen_conversions/eigen_msg.h>
#include <fla_msgs/GlobalPoint.h>
#include <octomap_msgs/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>

JunctionDetection::JunctionDetection() :
    running_(false),
    cave_entry_clearance_(100.0),
    update_threshold_(10.0),
    skipped_(0),
    map_skips_(2),
    cave_entry_(0,0,0),
    search_radius_(30.0),
    min_neighbors_(275)
{
    // Publisher
    clustered_junction_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/clustered_junction_points", 1);
    cluster_centroids_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cluster_centroids", 1);
    goal_point_pub_ = nh_.advertise<fla_msgs::GlobalPoint>("/goal_point", 1);

    // Subscriber
    octomap_sub_ = nh_.subscribe("/octomap_full", 1, &JunctionDetection::octomapCallback, this);
    sub_odom_ = nh_.subscribe("/current_state_est", 1, &JunctionDetection::uavOdomCallback, this);
    sub_control_ = nh_.subscribe("/control_planner", 1, &JunctionDetection::controlCallback, this);

    // Params
    nh_.getParam("junction_detection/cave_entry_clearance", cave_entry_clearance_);
    nh_.getParam("junction_detection/update_threshold", update_threshold_);
    nh_.getParam("junction_detection/skipped", skipped_);
    nh_.getParam("junction_detection/search_radius", search_radius_);
    nh_.getParam("junction_detection/min_neighbors", min_neighbors_);
}

JunctionDetection::~JunctionDetection() {
    if (latest_octree_) delete latest_octree_;
}

void JunctionDetection::controlCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (!running_ && msg->data) {
        ROS_INFO("Junction Detection started");
    } else if (running_ && !msg->data) {
        ROS_INFO("Junction Detection stopped");
    }
    running_ = msg->data;
}

void JunctionDetection::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    current_pose_ = odom->pose.pose;

    // Set cave enty point if not set and node running
    if (cave_entry_.x() == 0.0 && running_) {
        cave_entry_<<  current_pose_.position.x, current_pose_.position.y, current_pose_.position.z;
    }
}

void JunctionDetection::octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
    if (!running_) return;

    if (skipped_ < map_skips_) {
        skipped_++;
        return;
    }

    octomap::AbstractOcTree* tree = fullMsgToMap(*msg);
    if (latest_octree_) {
        // delete latest octree to avoid memory leak
        delete latest_octree_;
        latest_octree_ = nullptr;
    }
    latest_octree_ = dynamic_cast<octomap::OcTree*>(tree);

    if (latest_octree_) {
        detectJunctionsAndPublish(*latest_octree_);
    }

    // reset num of skipped maps
    skipped_ = 0;
}


bool JunctionDetection::hasEnoughNeighbors(const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree,
                        const pcl::PointXYZ& point,
                        const float radius,
                        const int min_neighbors)
{
    std::vector<int> point_indices;
    std::vector<float> point_distances;
    return (kdtree.radiusSearch(point, radius, point_indices, point_distances) >= min_neighbors);
}

void JunctionDetection::detectJunctionsAndPublish(octomap::OcTree& tree) {
    const pcl::PointCloud<pcl::PointXYZ>::Ptr all_junctions(new pcl::PointCloud<pcl::PointXYZ>());
    const pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_junctions(new pcl::PointCloud<pcl::PointXYZ>());

    const float resolution = tree.getResolution();

    // Collect frontier points ('free' with unknown 'neighbor')
    #pragma omp parallel for
    for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
        if (tree.isNodeOccupied(*it)) continue;

        octomap::point3d pt = it.getCoordinate();
        bool isFrontier = false;

        std::vector<octomap::point3d> neighbors = {
            {pt.x()+resolution, pt.y(),           pt.z()},
            {pt.x()-resolution, pt.y(),           pt.z()},
            {pt.x(),            pt.y()+resolution,pt.z()},
            {pt.x(),            pt.y()-resolution,pt.z()},
            {pt.x(),            pt.y(),           pt.z()+resolution},
            {pt.x(),            pt.y(),           pt.z()-resolution}
        };

        for (auto& nbr : neighbors) {
            if (tree.search(nbr) == nullptr) {
                isFrontier = true;
                break;
            }
        }

        if (isFrontier) {
            #pragma omp critical
            {
                all_junctions->push_back({pt.x(), pt.y(), pt.z()});
            }
        }
    }

    // Filter by num of neighbors in radius to avoid noise
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(all_junctions);

    #pragma omp parallel for
    for (size_t i = 0; i < all_junctions->size(); ++i) {
        if (hasEnoughNeighbors(kdtree, (*all_junctions)[i], search_radius_, min_neighbors_)) {
            #pragma omp critical
            {
                filtered_junctions->push_back((*all_junctions)[i]);
            }
        }
    }

    clusterJunctionsAndPublish(filtered_junctions);
}

void JunctionDetection::clusterJunctionsAndPublish(const pcl::PointCloud<pcl::PointXYZ>::Ptr& junctions) {
    // 1. Cluster
    pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZ>());
    search_tree->setInputCloud(junctions);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(10.0);
    ec.setMinClusterSize(50);
    ec.setMaxClusterSize(10000);
    ec.setSearchMethod(search_tree);
    ec.setInputCloud(junctions);

    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);

    // 2. Collect cluster clouds, centroids
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_frame_centroids(new pcl::PointCloud<pcl::PointXYZ>());

    int cluster_id = 0;
    for (const auto& cluster : cluster_indices) {
        float sum_x = 0, sum_y = 0, sum_z = 0;
        int count = 0;

        for (int idx : cluster.indices) {
            pcl::PointXYZRGB p;
            p.x = junctions->points[idx].x;
            p.y = junctions->points[idx].y;
            p.z = junctions->points[idx].z;

            clustered_cloud->push_back(p);

            sum_x += p.x;
            sum_y += p.y;
            sum_z += p.z;
            ++count;
        }
        if (count > 0) {
            pcl::PointXYZ centroid;
            centroid.x = sum_x / count;
            centroid.y = sum_y / count;
            centroid.z = sum_z / count;

            // filter for entries close to cave entry
            double dist = std::sqrt(std::pow(centroid.x - cave_entry_.x(), 2) +
                                    std::pow(centroid.y - cave_entry_.y(), 2) +
                                    std::pow(centroid.z - cave_entry_.z(), 2));
            if (dist > cave_entry_clearance_) {
                current_frame_centroids->push_back(centroid);
            }
        }
        cluster_id++;
    }

    // Publish all colored cluster points
    sensor_msgs::PointCloud2 clustered_output;
    toROSMsg(*clustered_cloud, clustered_output);
    clustered_output.header.frame_id = "world";
    clustered_output.header.stamp = ros::Time::now();
    clustered_junction_pub_.publish(clustered_output);

    // Publish centroids from this frame
    sensor_msgs::PointCloud2 centroid_output;
    toROSMsg(*current_frame_centroids, centroid_output);
    centroid_output.header.frame_id = "world";
    centroid_output.header.stamp = ros::Time::now();
    cluster_centroids_pub_.publish(centroid_output);

    // 3. Update global list of detected centroids
    updateDetectedCentroids(current_frame_centroids);

    // 4. Publish the newest (last) centroid if available
    if (!detected_centroids_.empty()) {
        publishGoalPoint(detected_centroids_.back());
    }
}

void JunctionDetection::updateDetectedCentroids(const pcl::PointCloud<pcl::PointXYZ>::Ptr & new_centroids)
{
    std::vector<pcl::PointXYZ> updated_list;
    updated_list.reserve(detected_centroids_.size());

    // Track which new centroids have already matched an old one
    std::vector<bool> matched_new(new_centroids->size(), false);

    // 1) Try to match old centroids to new centroids
    for (auto & old_c : detected_centroids_) {
        for (size_t i = 0; i < new_centroids->size(); ++i) {
            if (matched_new[i]) continue;  // already matched

            double dx = old_c.x - (*new_centroids)[i].x;
            double dy = old_c.y - (*new_centroids)[i].y;
            double dz = old_c.z - (*new_centroids)[i].z;
            double dist_sq = dx*dx + dy*dy + dz*dz;

            if (dist_sq < update_threshold_ * update_threshold_) {
                // Found a match within threshold
                matched_new[i] = true;

                // Overwrite with new centroid's positio
                old_c = (*new_centroids)[i];

                updated_list.push_back(old_c);
                break;  // Stop searching new centroids for this old centroid
            }
        }
    }

    // 2) Any new centroid that wasn't matched is truly "new"; add it
    for (size_t i = 0; i < new_centroids->size(); ++i) {
        if (!matched_new[i]) {
            updated_list.push_back((*new_centroids)[i]);
        }
    }

    // 3) Replace our old list
    detected_centroids_ = std::move(updated_list);
}

void JunctionDetection::publishGoalPoint(const pcl::PointXYZ& point) const {
    fla_msgs::GlobalPoint goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "world";
    goal.point.x = point.x;
    goal.point.y = point.y;
    goal.point.z = point.z;
    goal.orientation = 0.0;
    goal.velocity = 0.0;
    goal.acceleration = 0.0;
    goal_point_pub_.publish(goal);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "junction_detector");
    JunctionDetection detector;
    ros::spin();
    return 0;
}
