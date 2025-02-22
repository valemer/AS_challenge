#!/usr/bin/env python

import time
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import Point
from fla_msgs.msg import GlobalPath, GlobalPoint
import sensor_msgs.point_cloud2 as pc2
from tf.transformations import quaternion_matrix
from scipy.spatial import cKDTree


class CaveExplorerNode:
    def __init__(self):
        rospy.init_node('cave_explorer', anonymous=True)

        # Load parameters
        self.max_radius = rospy.get_param("~max_radius", 20.0)
        self.min_radius = rospy.get_param("~min_radius", 3.0)
        self.sampling_angle_deg = rospy.get_param("~sampling_angle_deg", 10)
        self.max_sampling_angle_deg = rospy.get_param("~max_sampling_angle_deg", 50)
        self.max_planning_distance = rospy.get_param("~max_planning_distance", 55.0)

        self.current_position = None
        self.current_velocity = np.array([0.0, 0.0, 0.0])
        self.current_orientation = np.eye(3)

        self.cloud = []
        self.filtered_cloud = []
        self.path_markers = MarkerArray()
        self.best_point = None
        self.running = False
        self.goal_point = None

        # KD-tree for nearest-neighbor queries
        self.kd_tree = None

        # Subscribers
        rospy.Subscriber("current_state_est", Odometry, self.odom_callback)
        rospy.Subscriber("/octomap_point_cloud_centers", PointCloud2, self.point_cloud_callback)
        rospy.Subscriber("control_planner", Bool, self.control)
        rospy.Subscriber("/goal_point", GlobalPoint, self.goal_point_callback)

        # Publishers
        self.path_pub = rospy.Publisher("path_marker_array", MarkerArray, queue_size=1)
        self.global_path_pub = rospy.Publisher("global_path", GlobalPath, queue_size=1)
        self.marker_pub = rospy.Publisher('sphere', Marker, queue_size=10)

        rospy.loginfo("CaveExplorerNode initialized. Waiting for position update...")
        self.explore()

    def goal_point_callback(self, msg: GlobalPoint):
        """Callback to receive a superior goalpoint."""
        if msg.point.x == 0 and msg.point.y == 0 and msg.point.z == 0:
            self.goal_point = None
        else:
            self.goal_point = msg

    def odom_callback(self, msg):
        """Updates current position, orientation, and velocity from odometry."""
        self.current_position = np.array([msg.pose.pose.position.x,
                                          msg.pose.pose.position.y,
                                          msg.pose.pose.position.z])
        orientation = msg.pose.pose.orientation
        self.current_orientation = quaternion_matrix([orientation.x,
                                                      orientation.y,
                                                      orientation.z,
                                                      orientation.w])[:3, :3]
        self.current_velocity = np.array([msg.twist.twist.linear.x,
                                          msg.twist.twist.linear.y,
                                          msg.twist.twist.linear.z])

        rospy.loginfo_once("Position updated for the first time. Exploration will begin.")

    def point_cloud_callback(self, msg):
        """Processes the incoming point cloud and filters it to a region in front of the drone."""
        # If you only want to load the cloud once per entire exploration cycle,
        # you can leave the "if len(self.cloud) > 0: return" check.
        if len(self.cloud) > 0:
            return

        full_cloud = np.array([[p[0], p[1], p[2]] for p in pc2.read_points(msg, skip_nans=True)])

        if self.current_position is None:
            self.cloud = full_cloud
        else:
            # Filter point cloud to only include points in a region around the drone
            forward_vector = np.dot(self.current_orientation, np.array([1.0, 0.0, 0.0]))
            self.filtered_cloud = []
            for point in full_cloud:
                relative_position = point - self.current_position
                if (np.dot(relative_position, forward_vector) > -50 and
                        abs(np.dot(relative_position, np.array([0.0, 1.0, 0.0]))) <= 100.0 and
                        abs(np.dot(relative_position, np.array([0.0, 0.0, 1.0]))) <= 100.0):
                    self.filtered_cloud.append(point)

            self.cloud = np.array(self.filtered_cloud)

        # Build a KD-Tree for fast nearest-neighbor lookups
        if len(self.cloud) > 0:
            self.kd_tree = cKDTree(self.cloud)
        else:
            self.kd_tree = None

    def control(self, msg):
        if msg.data and not self.running:
            rospy.loginfo("Cave Explorer started")
        elif not msg.data and self.running:
            rospy.loginfo("Cave Explorer stopped")
        self.running = msg.data

    def sample_sphere_directed(self, center, direction, radius, max_angle_deg, step_deg):
        """Samples points on a sphere around the center by rotating the direction vector."""
        direction = direction / np.linalg.norm(direction)

        max_angle_rad = np.radians(max_angle_deg)
        step_rad = np.radians(step_deg)

        sampled_points = []
        for yaw in np.arange(-max_angle_rad, max_angle_rad + step_rad, step_rad):
            for pitch in np.arange(-max_angle_rad, max_angle_rad + step_rad, step_rad):

                R_yaw = np.array([
                    [np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw),  np.cos(yaw),  0],
                    [0,            0,            1]
                ])
                R_pitch = np.array([
                    [np.cos(pitch), 0, np.sin(pitch)],
                    [0,             1, 0           ],
                    [-np.sin(pitch),0, np.cos(pitch)]
                ])

                rotated_direction = R_yaw @ (R_pitch @ direction)
                rotated_point = center + rotated_direction * radius

                # Compute clearance using KD-tree
                max_radius = self.calculate_max_radius(rotated_point)

                sampled_points.append((rotated_point, max_radius))
        return sampled_points

    def calculate_max_radius(self, point):
        """
        Calculates the maximum radius without obstacles for a given point.
        Uses a cKDTree query if available, otherwise returns self.max_radius.
        """
        if self.kd_tree is None or len(self.cloud) == 0:
            return self.max_radius

        # Single nearest-neighbor query
        distance, _ = self.kd_tree.query(point, k=1)
        return min(self.max_radius, distance - 0.1)

    def add_arrow_marker(self, point, father_point, mid):
        """Adds an arrow marker to represent the path, pointing from father to child."""
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "path"
        marker.id = mid
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        start = father_point
        end = point
        marker.points = [self.to_geometry_msg_point(start), self.to_geometry_msg_point(end)]

        marker.scale.x = 0.1
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 0.8
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.path_markers.markers.append(marker)

    def to_geometry_msg_point(self, point):
        p = Point()
        p.x, p.y, p.z = point
        return p

    def reconstruct_path(self, point_node, orientation=None):
        """Reconstructs the path from the start to the best point iteratively."""
        global_path = GlobalPath()
        global_path.header = Header(stamp=rospy.Time.now(), frame_id="world")
        global_path.points = []

        path_nodes = []
        current_node = point_node
        while current_node is not None:
            path_nodes.append(current_node)
            current_node = current_node['father']

        path_nodes.reverse()

        for i in range(1, len(path_nodes) - 1):
            node = path_nodes[i]
            next_node = path_nodes[i + 1]
            self.add_arrow_marker(next_node['position'], node['position'], i)

            global_point = GlobalPoint()
            global_point.point = self.to_geometry_msg_point(node['position'])
            global_point.orientation = np.arctan2(
                next_node['position'][1] - node['position'][1],
                next_node['position'][0] - node['position'][0]
            )
            global_point.velocity = -1.0
            global_point.acceleration = -1.0
            global_path.points.append(global_point)

        if orientation:
            global_path.points[-1].orientation = orientation
        return global_path

    def close_to_goal(self, node, goal):
        """Checks if we are within self.max_radius of the global goal."""
        if self.goal_point is None:
            return False
        else:
            return np.linalg.norm(goal - node['position']) < self.max_radius

    def explore(self):
        """
        Main exploration logic using nested loops.
        We'll time the "inner loop" here to see how long each planning cycle takes.
        """
        rate = rospy.Rate(10)
        rate.sleep()

        best_node = None

        while not rospy.is_shutdown():
            if self.current_position is None or len(self.cloud) < 1:
                rospy.loginfo_once("Waiting for initial position update...")
                rate.sleep()
                continue

            if not self.running:
                rate.sleep()
                continue

            if best_node is not None:
                dis = np.linalg.norm(self.current_position - best_node['father']['position'])
                if dis > 5.0:
                    rate.sleep()
                    continue

            start_position = self.current_position
            start_orientation = self.current_orientation

            base_dis_goal = 2 * self.max_planning_distance
            if self.goal_point is not None:
                goal_point = np.array([
                    self.goal_point.point.x,
                    self.goal_point.point.y,
                    self.goal_point.point.z
                ])
            else:
                goal_point = start_position + base_dis_goal * np.dot(
                    start_orientation,
                    np.array([1.0, 0.0, 0.0])
                )

            point_nodes = [{
                'position': start_position,
                'father': None,
                'radius': 0.0
            }]
            best_node = point_nodes[0]
            best_node['radius'] = self.calculate_max_radius(best_node["position"])
            total_distance = 0.0

            # ----------------------
            # TIMING THE INNER LOOP
            # ----------------------
            loop_start_time = time.time()

            # Inner loop
            while total_distance < self.max_planning_distance:
                if self.close_to_goal(best_node, goal_point):
                    point_nodes.append({
                        'position': goal_point,
                        'father': best_node,
                        'radius': 10.0
                    })
                    point_nodes.append({
                        'position': best_node['position'] + np.array([
                            10.0 * np.cos(self.goal_point.orientation),
                            10.0 * np.sin(self.goal_point.orientation),
                            self.goal_point.point.z
                        ]),
                        'father': goal_point,
                        'radius': 10.0
                    })
                    self.goal_point = None
                    break

                current_node = best_node
                if current_node['father'] is None:
                    forward_direction = np.dot(start_orientation, np.array([1.0, 0.0, 0.0]))
                else:
                    vec = current_node['position'] - current_node['father']['position']
                    forward_direction = vec / np.linalg.norm(vec)

                sampled_points = self.sample_sphere_directed(
                    current_node['position'],
                    forward_direction,
                    current_node['radius'],
                    self.max_sampling_angle_deg if self.goal_point is None else
                    self.max_sampling_angle_deg + 100,
                    self.sampling_angle_deg
                )

                best_value = -float('inf')
                best_candidate_node = None

                for sampled_point, max_radius in sampled_points:
                    if max_radius < self.min_radius:
                        continue
                    distance_to_goal = np.linalg.norm(sampled_point - goal_point)
                    distance_from_start = np.linalg.norm(sampled_point - start_position)
                    value = (3 * distance_from_start) - (5 * distance_to_goal) + (18 * max_radius)

                    if value > best_value:
                        best_value = value
                        best_candidate_node = {
                            'position': sampled_point,
                            'father': current_node,
                            'radius': max_radius
                        }

                if best_candidate_node is None:
                    rospy.logwarn("No valid next sampling point found. Breaking.")
                    break

                best_node = best_candidate_node
                point_nodes.append(best_node)
                total_distance += np.linalg.norm(best_node['position'] - start_position)

            # Measure how long the entire inner loop took
            loop_duration = time.time() - loop_start_time
            rospy.logdebug(f"Inner planning loop took {loop_duration:.3f} seconds.")

            # Reconstruct and publish the path
            rospy.logdebug("Maximum planning distance or goal reached. Reconstructing path.")
            global_path = self.reconstruct_path(best_node)
            if len(global_path.points) > 0:
                self.global_path_pub.publish(global_path)
                self.path_pub.publish(self.path_markers)
            else:
                rospy.logwarn("No path found.")

            # Clear markers and cloud for the next loop
            self.path_markers = MarkerArray()
            self.cloud = []
            self.kd_tree = None
            rate.sleep()

    def pub_sphere(self, point, radius):
        """(Optional) Publishes a sphere marker for visualization."""
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "sphere_namespace"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = point[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 2 * radius
        marker.scale.y = 2 * radius
        marker.scale.z = 2 * radius

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)


if __name__ == "__main__":
    try:
        explorer = CaveExplorerNode()
    except rospy.ROSInterruptException:
        pass
