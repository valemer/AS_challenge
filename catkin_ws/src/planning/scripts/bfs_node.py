import rospy
import numpy as np
from collections import deque
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from fla_msgs.msg import GlobalPath, GlobalPoint
import math
import tf.transformations as tf_trans

class BFS:
    def __init__(self):
        self.adj = {}  # Adjacency list for the graph
        self.visited_locations = []  # Stores visited locations as 3D points
        self.start_point = None  # Start point for BFS
        self.goal_point = None  # Goal point for BFS
        self.current_position = None  # Current position of the drone
        self.planning = False

        self.min_distance_between_nodes = 20.0  # Minimum distance between nodes
        self.max_distance_between_nodes = 35.0  # Maximum distance between nodes

        rospy.init_node('bfs_node', anonymous=True)

        rospy.Subscriber('/current_state_est', Odometry, self.current_position_callback)
        rospy.Subscriber('/fly_back_start_points', Point, self.start_point_callback)
        rospy.Subscriber('/fly_back_goal_points', Point, self.goal_point_callback)

        self.planned_path_pub = rospy.Publisher('/global_path', GlobalPath, queue_size=10)
        self.graph_visualization_pub = rospy.Publisher('/graph_visualization', MarkerArray, queue_size=10)

    def start_point_callback(self, msg):
        self.start_point = np.array([msg.x, msg.y, msg.z])
        rospy.logdebug("received start: {}".format(self.start_point))
        self.check_and_run_bfs()

    def goal_point_callback(self, msg):
        self.goal_point = np.array([msg.x, msg.y, msg.z])
        rospy.logdebug("received goal: {}".format(self.goal_point))
        self.check_and_run_bfs()

    def check_and_run_bfs(self):
        if self.goal_point is None:
            rospy.logdebug("goal missing for bfs")
            return
        if self.start_point is None:
            rospy.logdebug("start missing for bfs")
            return

        if self.planning:
            return
        else:
            self.planning = True

        rospy.loginfo("Fly-Back PLanning started")

        start_index = self.find_closest_node(self.start_point)
        goal_index = self.find_closest_node(self.goal_point)

        if start_index is None:
            rospy.loginfo("start index missing for bfs")
            return
        if goal_index is None:
            rospy.loginfo("goal index missing for bfs")
            return

        # 1) Use BFS to get path from start to the node that is closest to the goal
        path_indices = self.find_path(start_index, goal_index)

        # 2) Convert indices to actual 3D points (raw path, no smoothing)
        raw_path = [self.visited_locations[idx] for idx in path_indices]

        # 3) Append the actual goal as a final point
        #    This ensures we actually reach the goal coordinate, not just the closest node.
        raw_path.append(self.goal_point)

        # 4) Compute orientations for each waypoint
        poses_with_orientation = self.compute_orientations(raw_path)

        # 5) Finally, publish the path
        self.publish_path(poses_with_orientation)

    def find_closest_node(self, point):
        min_dist = float('inf')
        closest_index = None
        for i, location in enumerate(self.visited_locations):
            dist = np.linalg.norm(np.array(location) - point)
            if dist < min_dist:
                min_dist = dist
                closest_index = i
        return closest_index

    def current_position_callback(self, msg):
        self.current_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        # Only add a new node if we are sufficiently far from existing nodes
        if (not self.visited_locations or
                not any(np.linalg.norm(self.current_position - np.array(loc)) < self.min_distance_between_nodes
                        for loc in self.visited_locations)):
            self.visited_locations.append(self.current_position)
            self.update_graph()
            self.publish_graph_visualization()

    def update_graph(self):
        current_index = len(self.visited_locations) - 1
        self.adj.setdefault(current_index, [])

        # Connect to the previous node if any
        if current_index > 0:
            self.add_edge(current_index, current_index - 1)

        # Connect to any node within the specified distance
        for i in range(current_index):
            if np.linalg.norm(self.visited_locations[current_index] - self.visited_locations[i]) <= self.max_distance_between_nodes:
                self.add_edge(current_index, i)

    def add_edge(self, u, v):
        self.adj[u].append(v)
        self.adj[v].append(u)

    def find_path(self, start, goal):
        rospy.logdebug("Finding path from {},{},{}, to {},{},{}".
                      format(self.start_point[0], self.start_point[1], self.start_point[2],
                             self.goal_point[0], self.goal_point[1], self.goal_point[2]))

        timer = rospy.Time.now()

        q = deque()
        visited = {k: False for k in self.adj.keys()}
        parent = {k: None for k in self.adj.keys()}

        q.append(start)
        visited[start] = True

        while q:
            curr = q.popleft()
            if curr == goal:
                break

            for neighbor in self.adj.get(curr, []):
                if not visited[neighbor]:
                    visited[neighbor] = True
                    parent[neighbor] = curr
                    q.append(neighbor)

        # Reconstruct path
        path = []
        current = goal
        while current is not None:
            path.append(current)
            current = parent[current]
        path.reverse()

        rospy.loginfo("Path found in {} seconds".format((rospy.Time.now() - timer).to_sec()))
        return path

    def compute_orientations(self, path_points):
        """
        Given a list of 3D points (as Nx3 numpy arrays), compute a PoseStamped
        for each point with orientation facing the next point in the list.
        For the last point, reuse the previous orientation.
        """
        poses = []
        for i in range(len(path_points)):
            # Current pose
            pose = GlobalPoint()
            pose.header.frame_id = "world"
            pose.point.x = path_points[i][0]
            pose.point.y = path_points[i][1]
            pose.point.z = path_points[i][2]

            if i < len(path_points) - 1:
                pose.acceleration = -1.0
                pose.velocity = -1.0
                # Calculate orientation to face the next point
                dx = path_points[i+1][0] - path_points[i][0]
                dy = path_points[i+1][1] - path_points[i][1]

                # Simple approach: use yaw only for heading.
                pose.orientation = math.atan2(dy, dx)
            else:
                pose.acceleration = 0.0
                pose.velocity = 0.0
                # Last point => same orientation as the previous one
                if len(poses) > 0:
                    pose.orientation = poses[-1].orientation
                else:
                    pose.orientation = 0.0  # fallback if there's only one point

            poses.append(pose)

        return poses

    def publish_path(self, poses):
        """
        1) Visualize the entire path as a blue LINE_STRIP.
        2) Publish PoseStamped waypoints in batches.
        """
        if not poses:
            rospy.logwarn("No path poses to publish.")
            return

        # 1) Publish as a BLUE line strip for visualization
        path_marker = Marker()
        path_marker.header.frame_id = "world"
        path_marker.header.stamp = rospy.Time.now()
        path_marker.id = 9999  # Unique ID for the path marker
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.2  # Thickness of the line
        path_marker.color.r = 0.0
        path_marker.color.g = 0.0
        path_marker.color.b = 1.0
        path_marker.color.a = 1.0

        for pose in poses:
            point = Point()
            point.x = pose.point.x
            point.y = pose.point.y
            point.z = pose.point.z
            path_marker.points.append(point)

        self.graph_visualization_pub.publish(MarkerArray(markers=[path_marker]))

        # 2) Publish in batches so the drone follows gradually
        max_waypoints = 1
        total_waypoints = len(poses)

        for i in range(0, total_waypoints, max_waypoints):
            if i + max_waypoints < total_waypoints:
                batch_poses = poses[i:i + max_waypoints]
            else:
                batch_poses = poses[i:]

            # Publish the current batch as a Path message
            path_msg = GlobalPath()
            path_msg.header.frame_id = "world"
            path_msg.header.stamp = rospy.Time.now()
            path_msg.points = batch_poses

            self.planned_path_pub.publish(path_msg)
            rospy.logdebug("BFS node: Published waypoints from {} to {}".
                          format(i, i + len(batch_poses) - 1))

            # Wait until the drone is near the last waypoint in the current batch
            last_pose = batch_poses[-1]
            last_waypoint = np.array([
                last_pose.point.x,
                last_pose.point.y,
                last_pose.point.z
            ])

            while not rospy.is_shutdown():
                if self.current_position is None:
                    rospy.sleep(0.1)
                    continue

                if np.linalg.norm(self.current_position - last_waypoint) < 10.0:
                    rospy.logdebug("BFS node: Reached waypoint index = {}".format(i + len(batch_poses) - 1))
                    break
                rospy.sleep(0.1)  # Check every 100 ms

    def publish_graph_visualization(self):
        marker_array = MarkerArray()

        # Add nodes
        for i, location in enumerate(self.visited_locations):
            node_marker = Marker()
            node_marker.header.frame_id = "world"
            node_marker.header.stamp = rospy.Time.now()
            node_marker.id = i
            node_marker.type = Marker.SPHERE
            node_marker.action = Marker.ADD
            node_marker.pose.position.x = location[0]
            node_marker.pose.position.y = location[1]
            node_marker.pose.position.z = location[2]
            node_marker.scale.x = 0.5
            node_marker.scale.y = 0.5
            node_marker.scale.z = 0.5
            node_marker.color.r = 0.0
            node_marker.color.g = 1.0
            node_marker.color.b = 0.0
            node_marker.color.a = 1.0
            marker_array.markers.append(node_marker)

        # Add edges
        edge_id = len(self.visited_locations)
        for u, neighbors in self.adj.items():
            for v in neighbors:
                # Only add each edge once
                if u < v:
                    edge_marker = Marker()
                    edge_marker.header.frame_id = "world"
                    edge_marker.header.stamp = rospy.Time.now()
                    edge_marker.id = edge_id
                    edge_marker.type = Marker.LINE_STRIP
                    edge_marker.action = Marker.ADD
                    edge_marker.scale.x = 0.15
                    edge_marker.color.r = 1.0
                    edge_marker.color.g = 0.0
                    edge_marker.color.b = 0.0
                    edge_marker.color.a = 1.0

                    edge_marker.points.append(Point(*self.visited_locations[u]))
                    edge_marker.points.append(Point(*self.visited_locations[v]))

                    marker_array.markers.append(edge_marker)
                    edge_id += 1

        self.graph_visualization_pub.publish(marker_array)


if __name__ == '__main__':
    try:
        listener = BFS()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
