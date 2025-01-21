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


class CaveExplorerNode:
    def __init__(self):
        rospy.init_node('cave_explorer', anonymous=True)

        # Load parameters from the parameter server
        self.max_radius = rospy.get_param("~max_radius", 20.0)
        self.min_radius = rospy.get_param("~min_radius", 3.0)
        self.sampling_angle_deg = rospy.get_param("~sampling_angle_deg", 10)
        self.max_sampling_angle_deg = rospy.get_param("~max_sampling_angle_deg", 50)
        self.max_planning_distance = rospy.get_param("~max_planning_distance", 55.0)

        self.current_position = None  # Position is None until updated
        self.current_velocity = np.array([0.0, 0.0, 0.0])
        self.current_orientation = np.eye(3)
        self.cloud = []
        self.path_markers = MarkerArray()
        self.best_point = None
        self.running = True

        self.goal_point = None

        # Subscribers
        rospy.Subscriber("current_state_est", Odometry, self.odom_callback)
        rospy.Subscriber("/octomap_point_cloud_centers", PointCloud2, self.point_cloud_callback)
        rospy.Subscriber("/control_planner", Bool, self.control)
        rospy.Subscriber("/goal_point", GlobalPoint, self.goal_point_callback)

        # Publishers
        self.path_pub = rospy.Publisher("path_marker_array", MarkerArray, queue_size=1)
        self.global_path_pub = rospy.Publisher("global_path", GlobalPath, queue_size=1)
        self.marker_pub = rospy.Publisher('sphere', Marker, queue_size=10)

        rospy.loginfo("CaveExplorerNode initialized. Waiting for position update...")
        self.explore()
       # callback to receive a superior goalpoint
    def goal_point_callback(self, msg: GlobalPoint):
        """Callback to receive a superior goalpoint."""
        if msg.point.x == 0 and msg.point.y == 0 and msg.point.z == 0:
            self.goal_point = None
            rospy.loginfo("Goal point removed")
        else:
            self.goal_point = msg
            rospy.loginfo("Goal point received")


    def odom_callback(self, msg):
        """Updates current position, orientation, and velocity from odometry."""
        self.current_position = np.array([msg.pose.pose.position.x,
                                          msg.pose.pose.position.y,
                                          msg.pose.pose.position.z])
        orientation = msg.pose.pose.orientation
        self.current_orientation = quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])[:3, :3]
        self.current_velocity = np.array([msg.twist.twist.linear.x,
                                          msg.twist.twist.linear.y,
                                          msg.twist.twist.linear.z])

        rospy.loginfo_once("Position updated for the first time. Exploration will begin.")

    def point_cloud_callback(self, msg):
        """Processes the incoming point cloud and filters it to a region in front of the drone."""
        if len(self.cloud) > 0:
            return

        full_cloud = np.array([[p[0], p[1], p[2]] for p in pc2.read_points(msg, skip_nans=True)])
        if self.current_position is None:
            self.cloud = full_cloud
            return

        # Filter point cloud to only include points in a 30x30m region in front of the drone
        forward_vector = np.dot(self.current_orientation, np.array([1.0, 0.0, 0.0]))
        self.filtered_cloud = []
        for point in full_cloud:
            relative_position = point - self.current_position
            if (np.dot(relative_position, forward_vector) > -50 and
                    abs(np.dot(relative_position, np.array([0.0, 1.0, 0.0]))) <= 100.0 and
                    abs(np.dot(relative_position, np.array([0.0, 0.0, 1.0]))) <= 100.0):
                self.filtered_cloud.append(point)

        self.cloud = np.array(self.filtered_cloud)

    def control(self, msg):
        self.running = msg

    def sample_sphere_directed(self, center, direction, radius, max_angle_deg, step_deg):
        """Samples points on the sphere around the middle point by rotating in y and z directions."""
        # Normalize the direction vector
        direction = direction / np.linalg.norm(direction)

        # Convert max angle and step to radians
        max_angle_rad = np.radians(max_angle_deg)
        step_rad = np.radians(step_deg)

        # Points storage
        sampled_points = []

        # Sampling by rotating around y and z axes
        for yaw in np.arange(-max_angle_rad, max_angle_rad + step_rad, step_rad):  # Rotation in y-direction
            for pitch in np.arange(-max_angle_rad, max_angle_rad + step_rad, step_rad):  # Rotation in z-direction
                # Rotation matrix for yaw (rotation around z-axis)
                R_yaw = np.array([
                    [np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]
                ])

                # Rotation matrix for pitch (rotation around y-axis)
                R_pitch = np.array([
                    [np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]
                ])

                # Apply rotation to the normalized direction vector
                rotated_direction = np.dot(R_yaw, np.dot(R_pitch, direction))

                # Compute the rotated point on the sphere surface
                rotated_point = center + rotated_direction * radius

                # Optionally calculate the maximum radius (if required)
                max_radius = self.calculate_max_radius(rotated_point)

                # Append the sampled point and its properties
                sampled_points.append((rotated_point, max_radius))

                self.pub_sphere(rotated_point, max_radius)

        return sampled_points

    def calculate_max_radius(self, point):
        """Calculates the maximum radius without obstacles for a given point."""
        if len(self.cloud) == 0:
            return self.max_radius

        distances = np.linalg.norm(self.cloud - point, axis=1)
        nearest_distance = np.min(distances)
        return min(self.max_radius, nearest_distance - 0.1)

    def add_arrow_marker(self, point, father_point, id):
        """Adds an arrow marker to represent the path, pointing from father to child."""
        marker = Marker()
        marker.header.frame_id = "world"  # Use the `world` frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "path"
        marker.id = id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Arrow start and end points
        start = father_point
        end = point

        marker.points = []
        marker.points.append(self.to_geometry_msg_point(start))
        marker.points.append(self.to_geometry_msg_point(end))

        # Arrow scale and color
        marker.scale.x = 0.1  # Shaft diameter
        marker.scale.y = 0.2  # Head diameter
        marker.scale.z = 0.2  # Head length
        marker.color.a = 0.8  # Slightly transparent
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.path_markers.markers.append(marker)

    def to_geometry_msg_point(self, point):
        """Converts a NumPy point to a ROS geometry_msgs/Point."""
        p = Point()
        p.x, p.y, p.z = point
        return p

    def reconstruct_path(self, point_node,orientation = None):
        """Reconstructs the path from the start to the best point iteratively."""
        global_path = GlobalPath()
        global_path.header = Header(stamp=rospy.Time.now(), frame_id="world")
        global_path.points = []

        path_nodes = []
        current_node = point_node

        # Collect nodes in order from start to end
        while current_node is not None:
            path_nodes.append(current_node)
            current_node = current_node['father']

        # Reverse the collected path to get the correct order
        path_nodes.reverse()

        # Create path markers and global path message
        for i in range(1, len(path_nodes) - 1):  # Start from the second node, skip the first
            node = path_nodes[i]
            next_node = path_nodes[i + 1]

            self.add_arrow_marker(next_node['position'], node['position'], i)

            # Create a new GlobalPoint message
            global_point = GlobalPoint()
            global_point.point = self.to_geometry_msg_point(node['position'])
            global_point.orientation = np.arctan2(
                next_node['position'][1] - node['position'][1],
                next_node['position'][0] - node['position'][0]
            )
            global_point.velocity = -1.0  # Placeholder value
            global_point.acceleration = -1.0  # Placeholder value

            global_path.points.append(global_point)
        if orientation:
            global_path.points[-1].orientation = orientation
        return global_path


    def close_to_goal(self, node, goal):
        if self.goal_point is None:
            return False
        else:
            return np.linalg.norm(goal - node['position']) < self.max_radius
        

    def explore(self):
        """Main exploration logic using nested loops."""
        rate = rospy.Rate(1)
        rate.sleep()
        best_node = None
        while not rospy.is_shutdown():

            # Wait for the first position update
            if self.current_position is None or len(self.cloud) < 1:
                rospy.loginfo_once("Waiting for initial position update...")
                rate.sleep()
                continue

            if not self.running:
                continue


            if best_node is not None:
                dis = np.linalg.norm(self.current_position - best_node['father']['position'])
                if dis > 0.5:
                    continue

            start_position = self.current_position
            start_orientation = self.current_orientation

            base_dis_goal = 2 * self.max_planning_distance
            if self.goal_point is not None:
                goal_point = np.array(
                    [self.goal_point.point.x, self.goal_point.point.y, self.goal_point.point.z])
                total_dis_to_goal = np.linalg.norm(goal_point - self.current_position)
            else:
                goal_point = start_position + base_dis_goal * np.dot(start_orientation,
                                                                     np.array([1.0, 0.0, 0.0]))


            point_nodes = [{'position': start_position, 'father': None, 'radius': 0.0}]
            best_node = point_nodes[0]

            best_node['radius'] = self.calculate_max_radius(best_node["position"])

            total_distance = 0.0

            while total_distance < self.max_planning_distance:

                if self.close_to_goal(best_node, goal_point):
                    point_nodes.append({'position': goal_point, 'father': best_node, 'radius': 10.0})
                    point_nodes.append({'position': best_node['position'] + np.array(
                                                [10.0 * np.cos(self.goal_point.orientation),
                                                 10.0 * np.sin(self.goal_point.orientation),
                                                 self.goal_point.point.z]),
                                        'father': goal_point,
                                        'radius': 10.0})
                    break

                # Use the last node for exploration
                current_node = best_node
                forward_direction = np.dot(start_orientation, np.array([1.0, 0.0, 0.0])) if current_node['father'] is None else (current_node['position'] - current_node['father']['position']) / np.linalg.norm(current_node['position'] - current_node['father']['position'])

                # Sample points
                sampled_points = self.sample_sphere_directed(current_node['position'], forward_direction, current_node['radius'], self.max_sampling_angle_deg, self.sampling_angle_deg)

                #self.pub_sphere(current_node['position'], current_node['radius'])

                # Evaluate sampled points and find the best one
                best_value = -float('inf')
                for sampled_point, max_radius in sampled_points:
                    if max_radius < self.min_radius:
                        continue
                    distance_to_goal = np.linalg.norm(sampled_point - goal_point)
                    if self.goal_point is not None:
                        distance_to_goal *= (base_dis_goal / total_dis_to_goal)
                    distance_from_start = np.linalg.norm(sampled_point - start_position)
                    value = 3 * distance_from_start - 5 * distance_to_goal + 15 * max_radius
                    if value > best_value:
                        best_value = value
                        best_node = {'position': sampled_point, 'father': current_node, 'radius': max_radius}

                if np.linalg.norm(start_position - best_node['position']) < 0.1:
                    best_node['radius'] = best_node['radius'] - 1
                    rospy.logwarn('Start radius needs to be smallse!')

                # Add the new best node to the list
                point_nodes.append(best_node)
                total_distance += np.linalg.norm(best_node['position'] - start_position)

            # Reconstruct and publish the path
            rospy.logdebug("Maximum planning distance reached. Reconstructing and publishing path.")
            global_path = self.reconstruct_path(best_node)
            self.global_path_pub.publish(global_path)
            self.path_pub.publish(self.path_markers)
            self.path_markers = MarkerArray()
            self.cloud = []
            rate.sleep()

    def pub_sphere(self, point, radius):
        marker = Marker()
        marker.header.frame_id = "world"  # Frame of reference
        marker.header.stamp = rospy.Time.now()

        marker.ns = "sphere_namespace"  # Namespace for the marker
        marker.id = 0  # Unique ID for the marker
        marker.type = Marker.SPHERE  # Type of marker (sphere)
        marker.action = Marker.ADD  # Add or modify the marker

        # Set the pose of the sphere
        marker.pose.position.x = point[0]  # X position
        marker.pose.position.y = point[1]  # Y position
        marker.pose.position.z = point[2]  # Z position
        marker.pose.orientation.x = 0.0  # Orientation (quaternion)
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Set the scale of the sphere
        marker.scale.x = 2 * radius  # Scale in X dimension
        marker.scale.y = 2 * radius  # Scale in Y dimension
        marker.scale.z = 2 * radius  # Scale in Z dimension

        # Set the color of the sphere
        marker.color.r = 0.0  # Red
        marker.color.g = 1.0  # Green
        marker.color.b = 0.0  # Blue
        marker.color.a = 1.0  # Alpha (transparency)

        self.marker_pub.publish(marker)


if __name__ == "__main__":
    try:
        explorer = CaveExplorerNode()
    except rospy.ROSInterruptException:
        pass