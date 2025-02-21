#include <eigen_conversions/eigen_msg.h>
#include <fla_msgs/GlobalPath.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/builtin_bool.h>
#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>  // KD-Tree for fast nearest neighbor search
#include <pcl/segmentation/extract_clusters.h>  // Euclidean
#include <tf/transform_listener.h>

class StrongJunctionDetector {
private:
    ros::NodeHandle nh_;
    ros::Subscriber octomap_sub_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_control_;
    ros::Publisher clustered_junction_pub_;
    ros::Publisher cluster_centroids_pub_;
    ros::Publisher goal_point_pub_;
    ros::Timer timer_;
    octomap::OcTree* latest_octree_ = nullptr;
    bool new_octomap_received_ = false;
    geometry_msgs::Pose current_pose_;
    bool running = false;

public:
    StrongJunctionDetector() {
        clustered_junction_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/clustered_junction_points", 1);
        cluster_centroids_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cluster_centroids", 1);
        goal_point_pub_ = nh_.advertise<fla_msgs::GlobalPoint>("/goal_point", 1);
        octomap_sub_ = nh_.subscribe("/octomap_full", 1, &StrongJunctionDetector::octomapCallback, this);
        sub_odom_ = nh_.subscribe("/current_state_est", 1, &StrongJunctionDetector::uavOdomCallback, this);
        sub_control_ = nh_.subscribe("/control_planner", 1, &StrongJunctionDetector::control, this);
    }

    ~StrongJunctionDetector() {
        if (latest_octree_) delete latest_octree_;
    }

    void control(const  std_msgs::Bool::ConstPtr& msg) {
        if (running && !msg->data) {
            ROS_INFO("Junction Detection stopped");
        } else if (!running && msg->data) {
            ROS_INFO("Junction Detection started");
        }
        running = msg->data;
    }

    // Callback to get current Pose of UAV
    void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
        // Store current position in our planner
        current_pose_ = odom->pose.pose;
    }

    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
        if (!running) {
            return;
        }
        octomap::AbstractOcTree* tree = msgToMap(*msg);
        if (latest_octree_) {
            delete latest_octree_;
            latest_octree_ = nullptr;
        }
        latest_octree_ = dynamic_cast<octomap::OcTree*>(tree);
        processJunctions();
    }

    void processJunctions() {

        detectJunctionsAndPublish(*latest_octree_);
    }

    // Function to check if a point has at least 50 neighbors within 10 meters
    bool hasEnoughNeighbors(pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree, const pcl::PointXYZ& point, float radius, int min_neighbors) {
        std::vector<int> point_indices;
        std::vector<float> point_distances;
        return kdtree.radiusSearch(point, radius, point_indices, point_distances) >= min_neighbors;
    }

    void detectJunctionsAndPublish(octomap::OcTree& tree) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr all_junctions(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_junctions(new pcl::PointCloud<pcl::PointXYZ>());

        float resolution = tree.getResolution();

        #pragma omp parallel for
        for (octomap::OcTree::leaf_iterator it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
            if (tree.isNodeOccupied(*it)) continue;

            octomap::point3d point = it.getCoordinate();
            bool isFrontier = false;

            std::vector<octomap::point3d> neighbors = {
                octomap::point3d(point.x() + resolution, point.y(), point.z()),
                octomap::point3d(point.x() - resolution, point.y(), point.z()),
                octomap::point3d(point.x(), point.y() + resolution, point.z()),
                octomap::point3d(point.x(), point.y() - resolution, point.z()),
                octomap::point3d(point.x(), point.y(), point.z() + resolution),
                octomap::point3d(point.x(), point.y(), point.z() - resolution)};

            for (const auto& neighbor : neighbors) {
                if (tree.search(neighbor) == nullptr) {
                    isFrontier = true;
                    break;
                }
            }

            if (isFrontier) {
                #pragma omp critical
                all_junctions->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
            }
        }

        float search_radius = 30.0;
        int min_neighbors = 275;
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(all_junctions);

        #pragma omp parallel for
        for (size_t i = 0; i < all_junctions->size(); ++i) {
            if (hasEnoughNeighbors(kdtree, all_junctions->points[i], search_radius, min_neighbors)) {
                #pragma omp critical
                filtered_junctions->push_back(all_junctions->points[i]);
            }
        }

        clusterJunctionsAndPublish(filtered_junctions);
    }

    void clusterJunctionsAndPublish(pcl::PointCloud<pcl::PointXYZ>::Ptr& junctions) {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        std::vector<pcl::PointIndices> cluster_indices;

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(10.0);
        ec.setMinClusterSize(50);
        ec.setMaxClusterSize(10000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(junctions);
        ec.extract(cluster_indices);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr centroids_cloud(new pcl::PointCloud<pcl::PointXYZ>());

        int cluster_id = 0;
        for (const auto& cluster : cluster_indices) {
            float sum_x = 0, sum_y = 0, sum_z = 0;
            int count = 0;

            for (const auto& idx : cluster.indices) {
                pcl::PointXYZRGB p;
                p.x = junctions->points[idx].x;
                p.y = junctions->points[idx].y;
                p.z = junctions->points[idx].z;
                p.r = (cluster_id * 30) % 255;
                p.g = (cluster_id * 50) % 255;
                p.b = (cluster_id * 70) % 255;
                clustered_cloud->push_back(p);

                sum_x += p.x;
                sum_y += p.y;
                sum_z += p.z;
                count++;
            }

            if (count > 0) {
                pcl::PointXYZ centroid;
                centroid.x = sum_x / count;
                centroid.y = sum_y / count;
                centroid.z = sum_z / count;
                if (std::sqrt(std::pow(centroid.x - 322.0, 2) + std::pow(centroid.y - 7.0, 2) + std::pow(centroid.z - 20, 2)) > 50)
                {
                    centroids_cloud->push_back(centroid);
                }
            }
            cluster_id++;
        }

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*clustered_cloud, output);
        output.header.frame_id = "world";
        output.header.stamp = ros::Time::now();
        clustered_junction_pub_.publish(output);

        sensor_msgs::PointCloud2 centroid_output;
        pcl::toROSMsg(*centroids_cloud, centroid_output);
        centroid_output.header.frame_id = "world";
        centroid_output.header.stamp = ros::Time::now();
        cluster_centroids_pub_.publish(centroid_output);
        findAndPublishClosestCentroid(centroids_cloud);
    }


    void findAndPublishClosestCentroid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& centroids) {
        if (centroids->empty()) return;

        pcl::PointXYZ closest_centroid;
        double min_distance = std::numeric_limits<double>::max();

        for (const auto& centroid : centroids->points) {
            double distance = std::hypot(centroid.x - current_pose_.position.x, centroid.y - current_pose_.position.y);
            if (distance < min_distance) {
                min_distance = distance;
                closest_centroid = centroid;
            }
        }

        publishGoalPoint(closest_centroid);
    }

    void publishGoalPoint(const pcl::PointXYZ& closest_point) {
        fla_msgs::GlobalPoint goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "world";
        goal.point.x = closest_point.x;
        goal.point.y = closest_point.y;
        goal.point.z = closest_point.z;
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
