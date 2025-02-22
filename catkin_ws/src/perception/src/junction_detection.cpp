#include <eigen_conversions/eigen_msg.h>
#include <fla_msgs/GlobalPoint.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>

class StrongJunctionDetector {
private:
    ros::NodeHandle nh_;
    ros::Subscriber octomap_sub_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_control_;
    ros::Publisher clustered_junction_pub_;
    ros::Publisher cluster_centroids_pub_;
    ros::Publisher goal_point_pub_;

    octomap::OcTree* latest_octree_ = nullptr;
    geometry_msgs::Pose current_pose_;
    bool running = false;

    // Store centroids in the order they were found:
    std::vector<pcl::PointXYZ> detected_centroids_;

public:
    StrongJunctionDetector() {
        clustered_junction_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/clustered_junction_points", 1);
        cluster_centroids_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cluster_centroids", 1);
        goal_point_pub_ = nh_.advertise<fla_msgs::GlobalPoint>("/goal_point", 1);

        octomap_sub_ = nh_.subscribe("/octomap_full", 1, &StrongJunctionDetector::octomapCallback, this);
        sub_odom_   = nh_.subscribe("/current_state_est", 1, &StrongJunctionDetector::uavOdomCallback, this);
        sub_control_ = nh_.subscribe("/control_planner", 1, &StrongJunctionDetector::controlCallback, this);
    }

    ~StrongJunctionDetector() {
        if (latest_octree_) delete latest_octree_;
    }

    void controlCallback(const std_msgs::Bool::ConstPtr& msg) {
        running = msg->data;
        if (running) {
            ROS_INFO("Junction Detection started");
        } else {
            ROS_INFO("Junction Detection stopped");
        }
    }

    void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
        current_pose_ = odom->pose.pose;
    }

    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
        if (!running) return;
        octomap::AbstractOcTree* tree = octomap_msgs::fullMsgToMap(*msg);
        if (latest_octree_) {
            delete latest_octree_;
            latest_octree_ = nullptr;
        }
        latest_octree_ = dynamic_cast<octomap::OcTree*>(tree);
        processJunctions();
    }

    void processJunctions() {
        if (!latest_octree_) return;
        detectJunctionsAndPublish(*latest_octree_);
    }

    // Returns true if 'point' has at least 'min_neighbors' in 'radius'
    bool hasEnoughNeighbors(pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree,
                            const pcl::PointXYZ& point,
                            float radius,
                            int min_neighbors)
    {
        std::vector<int> point_indices;
        std::vector<float> point_distances;
        return (kdtree.radiusSearch(point, radius, point_indices, point_distances) >= min_neighbors);
    }

    void detectJunctionsAndPublish(octomap::OcTree& tree) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr all_junctions(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_junctions(new pcl::PointCloud<pcl::PointXYZ>());

        float resolution = tree.getResolution();

        // Collect frontier points
        #pragma omp parallel for
        for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
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

        // Filter by neighbors
        float search_radius = 30.0;
        int   min_neighbors = 275;
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(all_junctions);

        #pragma omp parallel for
        for (size_t i = 0; i < all_junctions->size(); ++i) {
            if (hasEnoughNeighbors(kdtree, (*all_junctions)[i], search_radius, min_neighbors)) {
                #pragma omp critical
                {
                    filtered_junctions->push_back((*all_junctions)[i]);
                }
            }
        }

        clusterJunctionsAndPublish(filtered_junctions);
    }

    void clusterJunctionsAndPublish(const pcl::PointCloud<pcl::PointXYZ>::Ptr& junctions) {
        // 1) Cluster
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

        // 2) Collect cluster clouds, centroids
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
                // Simple coloring
                p.r = (cluster_id * 30) % 255;
                p.g = (cluster_id * 50) % 255;
                p.b = (cluster_id * 70) % 255;

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

                // Example distance filter
                double dist = std::sqrt(std::pow(centroid.x + 322.0, 2) +
                                        std::pow(centroid.y +   7.0, 2) +
                                        std::pow(centroid.z -  20.0, 2));
                if (dist > 100.0) {
                    current_frame_centroids->push_back(centroid);
                }
            }
            cluster_id++;
        }

        // Publish all colored cluster points
        sensor_msgs::PointCloud2 clustered_output;
        pcl::toROSMsg(*clustered_cloud, clustered_output);
        clustered_output.header.frame_id = "world";
        clustered_output.header.stamp = ros::Time::now();
        clustered_junction_pub_.publish(clustered_output);

        // Publish centroids from this frame
        sensor_msgs::PointCloud2 centroid_output;
        pcl::toROSMsg(*current_frame_centroids, centroid_output);
        centroid_output.header.frame_id = "world";
        centroid_output.header.stamp = ros::Time::now();
        cluster_centroids_pub_.publish(centroid_output);

        // 3) Update our internal list of detected centroids
        updateDetectedCentroids(current_frame_centroids, 10.0 /* threshold in meters */);

        // 4) Publish the newest (last) centroid if available
        if (!detected_centroids_.empty()) {
            publishGoalPoint(detected_centroids_.back());
        }
    }

    // Update or remove old centroids based on new detections
    void updateDetectedCentroids(const pcl::PointCloud<pcl::PointXYZ>::Ptr & new_centroids,
                                 double threshold_distance)
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

                if (dist_sq < threshold_distance*threshold_distance) {
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

    // Publish a single centroid as fla_msgs::GlobalPoint
    void publishGoalPoint(const pcl::PointXYZ& point) {
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
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "strong_junction_detector");
    StrongJunctionDetector detector;
    ros::spin();
    return 0;
}
