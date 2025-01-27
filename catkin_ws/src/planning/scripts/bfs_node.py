import rospy
import numpy as np
from collections import deque
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray


class BFS:
    def __init__(self):
        self.adj = {}  # Adjacency list for the graph
        self.visited_locations = []  # Stores visited locations as 3D points
        self.start_point = None  # Start point for BFS
        self.goal_point = None  # Goal point for BFS
        self.current_position = None  # Current position of the drone

        rospy.init_node('bfs_node', anonymous=True)

        rospy.Subscriber('/current_state_est', Odometry, self.current_position_callback)
        rospy.Subscriber('/fly_back_start_points', Point, self.start_point_callback)
        rospy.Subscriber('/fly_back_goal_points', Point, self.goal_point_callback)

        self.planned_path_pub = rospy.Publisher('/planned_path', Path, queue_size=10)
        self.graph_visualization_pub = rospy.Publisher('/graph_visualization', MarkerArray, queue_size=10)

    def start_point_callback(self, msg):
        self.start_point = np.array([msg.x, msg.y, msg.z])
        self.check_and_run_bfs()

    def goal_point_callback(self, msg):
        self.goal_point = np.array([msg.x, msg.y, msg.z])
        self.check_and_run_bfs()

    def check_and_run_bfs(self):
        if self.start_point is not None and self.goal_point is not None:
            start_index = self.find_closest_node(self.start_point)
            goal_index = self.find_closest_node(self.goal_point)
            if start_index is not None and goal_index is not None:
                path = self.find_path(start_index, goal_index)
                self.publish_path(path)

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
        self.current_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        if not self.visited_locations or np.linalg.norm(self.current_position - np.array(self.visited_locations[-1])) > 5.0:
            self.visited_locations.append(self.current_position)
            self.update_graph()
            self.publish_graph_visualization()

    def update_graph(self):
        current_index = len(self.visited_locations) - 1
        self.adj.setdefault(current_index, [])

        # Connect to the previous node
        if current_index > 0:
            self.add_edge(current_index, current_index - 1)

        # Connect to any node within 5m
        for i in range(current_index):
            if np.linalg.norm(self.visited_locations[current_index] - self.visited_locations[i]) <= 5.0:
                self.add_edge(current_index, i)

    def add_edge(self, u, v):
        self.adj[u].append(v)
        self.adj[v].append(u)

    def find_path(self, start, goal):
        rospy.loginfo("Finding path from {},{},{}, to {},{},{}".format(self.start_point[0], self.start_point[1], self.start_point[2], self.goal_point[0], self.goal_point[1], self.goal_point[2]))
        # rospy.loginfo("Start coordinates: x={}, y={}, z={}".format(self.start_point[0], self.start_point[1], self.start_point[2]))
        # rospy.loginfo("Goal coordinates: x={}, y={}, z={}".format(self.goal_point[0], self.goal_point[1], self.goal_point[2]))
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
        # Append the goal point to the end of the path
        path.append(self.goal_point)

        rospy.loginfo("Path found in {} seconds".format((rospy.Time.now() - timer).to_sec()))

        return path

    # def publish_path(self, path):
    #     # Publish the path as a Path message
    #     path_msg = Path()
    #     path_msg.header.frame_id = "world"
    #     path_msg.header.stamp = rospy.Time.now()

    #     for index in path:
    #         pose = PoseStamped()
    #         pose.header.frame_id = "world"
    #         pose.pose.position.x = self.visited_locations[index][0]
    #         pose.pose.position.y = self.visited_locations[index][1]
    #         pose.pose.position.z = self.visited_locations[index][2]
    #         path_msg.poses.append(pose)

    #     self.planned_path_pub.publish(path_msg)
    #     rospy.loginfo("Path: {}".format(path))

    #     # Publish the path as a red line (LINE_STRIP)
    #     path_marker = Marker()
    #     path_marker.header.frame_id = "world"
    #     path_marker.header.stamp = rospy.Time.now()
    #     path_marker.id = 9999  # Unique ID for the path marker
    #     path_marker.type = Marker.LINE_STRIP
    #     path_marker.action = Marker.ADD
    #     path_marker.scale.x = 0.2  # Thickness of the line
    #     path_marker.color.r = 1.0
    #     path_marker.color.g = 0.0
    #     path_marker.color.b = 0.0
    #     path_marker.color.a = 1.0

    #     for index in path:
    #         point = Point()
    #         point.x = self.visited_locations[index][0]
    #         point.y = self.visited_locations[index][1]
    #         point.z = self.visited_locations[index][2]
    #         path_marker.points.append(point)

    #     self.graph_visualization_pub.publish(MarkerArray(markers=[path_marker]))

    def publish_path(self, path):

        # Visualize the entire path as a red line (LINE_STRIP)
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

        for index in path:
            point = Point()
            point.x = self.visited_locations[index][0]
            point.y = self.visited_locations[index][1]
            point.z = self.visited_locations[index][2]
            path_marker.points.append(point)

        self.graph_visualization_pub.publish(MarkerArray(markers=[path_marker]))

        max_waypoints = 5
        total_waypoints = len(path)

        for i in range(0, total_waypoints, max_waypoints):
            if i + max_waypoints < total_waypoints:
                # Get the next batch of waypoints (up to max_waypoints)
                batch = path[i:i + max_waypoints]
            else:
                # Get the remaining waypoints
                batch = path[i:]

            # Publish the current batch as a Path message
            path_msg = Path()
            path_msg.header.frame_id = "world"
            path_msg.header.stamp = rospy.Time.now()

            for index in batch:
                pose = PoseStamped()
                pose.header.frame_id = "world"
                pose.pose.position.x = self.visited_locations[index][0]
                pose.pose.position.y = self.visited_locations[index][1]
                pose.pose.position.z = self.visited_locations[index][2]
                path_msg.poses.append(pose)

            self.planned_path_pub.publish(path_msg)
            rospy.loginfo("BFS node: Published waypoints: {}".format(batch))

            # Wait until the drone is near the last waypoint in the current batch
            last_waypoint = np.array(self.visited_locations[batch[-1]])
            while not rospy.is_shutdown():
                # current_position = np.array([self.current_position[0], self.current_position[1], self.current_position[2]])
                if np.linalg.norm(self.current_position - last_waypoint) < 1.0:  # Threshold of 1 meter
                    rospy.loginfo("BFS node: Reached waypoint: {}".format(batch[-1]))
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
                if u < v:  # Avoid duplicating edges
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
