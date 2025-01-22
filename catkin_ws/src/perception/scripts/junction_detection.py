import rospy
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid
from skimage.morphology import skeletonize
from geometry_msgs.msg import PointStamped,Point
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry
from fla_msgs.msg import Junction, JunctionArray
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import math

class JunctionDetectionNode:
    def __init__(self):
        rospy.init_node("junction_detection_node")

        # Parameters
        self.current_uav_height = 0.0
        self.detected_junctions = []
        self.min_dis_new_junction = 30.0
        self.min_detections_in_area = 20
        self.square_size = 15

        # Subscribers
        rospy.Subscriber("/current_state_est", Odometry, self.uav_odom_callback)
        rospy.Subscriber("/occupancy_grid", OccupancyGrid, self.occupancy_grid_callback)

        # Publisher
        self.marker_pub = rospy.Publisher("/junction_arrows_array", MarkerArray, queue_size=10)
        self.image_pub = rospy.Publisher("/junction_visualization", Image, queue_size=10)

        # we will publish the Junctions as Junctions Array
        self.junction_pub = rospy.Publisher("/junctions_array",JunctionArray, queue_size = 10)


    def uav_odom_callback(self, msg):
        self.current_uav_height = msg.pose.pose.position.z

    def close_angle(self, angle1, angle2, old_size):
        diff = abs(angle1 - angle2)
        diff = min(diff, 2 * math.pi - diff)
        if diff <= math.pi / 4:
            x = old_size * math.cos(angle1) + math.cos(angle2)
            y = old_size * math.sin(angle1) + math.sin(angle2)
            angle = math.atan2(y, x) % (2 * math.pi)
            # Normalize to -pi to pi
            if angle > math.pi:
                angle -= 2 * math.pi
            return angle
        return None

    def add_to_detected_junctions(self, new_detected_junctions):
        for new_det_jun in new_detected_junctions:
            for junction in self.detected_junctions:
                old_counter = junction['counter']
                if np.linalg.norm(junction['position'] - new_det_jun[0]) < self.min_dis_new_junction:
                    new_orientations = []
                    for orientation in new_det_jun[1]:
                        for old_orientation in junction['orientations']:
                            angle = self.close_angle(old_orientation, orientation, old_counter)
                            if angle is not None:
                                new_orientations.append(angle)
                    if len(new_orientations) < len(junction['orientations']):
                        continue
                    junction['position'] = (new_det_jun[0] + junction['position'] * old_counter) / (old_counter + 1)
                    junction['orientations'] = new_orientations
                    junction['counter'] = old_counter + 1
                    return

            self.detected_junctions.append({'position': new_det_jun[0], 'orientations': new_det_jun[1], 'counter': 1})

    def tf_to_world_coords(self, junctions, resolution, origin, width):
        world_junctions = []
        for (y_index, x_index), orientations in junctions:
            x_world = origin.position.x + ((width - 1 - x_index) * resolution)
            y_world = origin.position.y + (y_index * resolution)
            z_world = self.current_uav_height
            world_junctions.append((np.array([x_world, y_world, z_world]), orientations))
        return world_junctions

    def occupancy_grid_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin

        grid_data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        grid_map = (grid_data == 0).astype(np.uint8)
        grid_map = np.fliplr(grid_map)

        kernel = np.ones((5, 5), np.uint8)
        grid_map = cv2.dilate(grid_map, kernel, iterations=1)

        junction_orientations, skeleton = self.detect_junctions_with_orientations(grid_map, width, height)

        junction_orientations = self.filter_oriented_junctions_in_square(junction_orientations, width, height, self.square_size)

        filtered_junction_orientations = [entry for entry in junction_orientations if len(entry[1]) >= 3]

        junction_orientations_in_world_frame = self.tf_to_world_coords(filtered_junction_orientations, resolution, origin, width)
        self.add_to_detected_junctions(junction_orientations_in_world_frame)

        self.publish_final_junctions()
        self.publish_junction_arrows()
        self.visualize_junctions(grid_map, filtered_junction_orientations, skeleton, width, height, self.square_size)

    def mean_of_angles(self, radians):
        avg_sin = sum(math.sin(r) for r in radians) / len(radians)
        avg_cos = sum(math.cos(r) for r in radians) / len(radians)
        return math.atan2(avg_sin, avg_cos)

    def detect_junctions_with_orientations(self, grid_map, width, height, neighbor_threshold=14, distance_threshold=10):
        free_space = (grid_map == 0).astype(np.uint8)
        skeleton = skeletonize(free_space).astype(np.uint8)

        kernel = np.array([[1, 1, 1],
                           [1, 10, 1],
                           [1, 1, 1]], dtype=np.uint8)
        neighbor_count = cv2.filter2D(skeleton, -1, kernel)
        junction_map = ((neighbor_count >= neighbor_threshold) & (skeleton > 0)).astype(np.uint8)
        junction_points = np.argwhere(junction_map > 0)

        junction_orientations = []
        for y, x in junction_points:
            orientations = []
            current_group = []
            groups = []
            for index, angle in enumerate(range(0, 360, 15)):
                radians = math.radians(angle)
                dx = math.cos(radians)
                dy = math.sin(radians)
                free_cells = 0
                for step in range(1, distance_threshold + 1):
                    nx = int(x + step * dx)
                    ny = int(y + step * dy)
                    if 0 <= ny < height and 0 <= nx < width and grid_map[ny, nx] == 0:
                        free_cells += 1
                    else:
                        break

                if free_cells >= distance_threshold:
                    current_group.append({'idx': index, 'angle': math.atan2(dy, -dx)})
                else:
                    if current_group:
                        groups.append(current_group)
                        current_group = []

            if current_group:
                if len(groups) > 0 and groups[0][0]['idx'] == 0:
                    groups[0].extend(current_group)
                else:
                    groups.append(current_group)

            for group in groups:
                orientations.append(self.mean_of_angles([item["angle"] for item in group]))

            junction_orientations.append(((y, x), orientations))

        return junction_orientations, skeleton

    def filter_oriented_junctions_in_square(self, junction_orientations, width, height, square_size):
        margin_x = (width - square_size) // 2
        margin_y = (height - square_size) // 2
        return [((y, x), orientations) for (y, x), orientations in junction_orientations
                if margin_y <= y < margin_y + square_size and margin_x <= x < margin_x + square_size]

    from visualization_msgs.msg import MarkerArray

    def publish_junction_arrows(self):
        marker_array = MarkerArray()  # Create a MarkerArray message
        marker_id = 0

        for junction in self.detected_junctions:
            if junction['counter'] > self.min_detections_in_area:
                x_world, y_world, z_world = junction['position']
                for angle in junction['orientations']:
                    dx = 15.0 * math.cos(angle)
                    dy = 15.0 * math.sin(angle)

                    marker = Marker()
                    marker.header.frame_id = "world"
                    marker.header.stamp = rospy.Time.now()
                    marker.ns = "junction_arrows"
                    marker.id = marker_id
                    marker.type = Marker.ARROW
                    marker.action = Marker.ADD

                    # Arrow start and end points
                    start_point = PointStamped().point
                    start_point.x = x_world
                    start_point.y = y_world
                    start_point.z = z_world

                    end_point = PointStamped().point
                    end_point.x = x_world + dx
                    end_point.y = y_world + dy
                    end_point.z = z_world

                    marker.points = [start_point, end_point]

                    # Arrow properties
                    marker.scale.x = 1.5  # Shaft diameter
                    marker.scale.y = 3.0  # Head diameter
                    marker.scale.z = 3.0  # Head length
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    marker.color.a = 1.0  # Fully opaque

                    marker_array.markers.append(marker)  # Add the marker to the MarkerArray
                    marker_id += 1

        # Publish the MarkerArray
        self.marker_pub.publish(marker_array)

    def publish_final_junctions(self):
        junction_array = JunctionArray()
        for junction in self.detected_junctions:
            if junction['counter'] > self.min_detections_in_area:
                j = Junction()         
                position = junction['position']
                orientations = junction['orientations']
                j.position = Point(*position)
                j.angles = orientations
                detections = junction['counter']
                junction_array.junctions.append(j)
                #rospy.loginfo(f'Junction detected at: {position} with {orientations} and {detections}')
        self.junction_pub.publish(junction_array)



    def visualize_junctions(self, grid_map, junction_orientations, skeleton, width, height, square_size):
        # Convert skeleton to displayable format
        skeleton_display = cv2.resize((skeleton * 255).astype(np.uint8), (500, 500), interpolation=cv2.INTER_NEAREST)

        # Highlight junctions on the grid map
        colored_map = cv2.cvtColor((grid_map == 0).astype(np.uint8) * 255, cv2.COLOR_GRAY2BGR)
        red_color = [0, 0, 255]
        for (y, x), _ in junction_orientations:
            cv2.circle(colored_map, (x, y), radius=2, color=red_color, thickness=-1)

        # Draw a rectangle for the central filtering square
        margin_x = (width - square_size) // 2
        margin_y = (height - square_size) // 2
        cv2.rectangle(colored_map, (margin_x, margin_y), (margin_x + square_size - 1, margin_y + square_size - 1),
                      (0, 255, 0), 1)

        # Resize the map for consistent visualization
        highlighted_map = cv2.resize(colored_map, (500, 500), interpolation=cv2.INTER_NEAREST)

        # Convert the highlighted map to a ROS image
        bridge = CvBridge()
        try:
            ros_image = bridge.cv2_to_imgmsg(highlighted_map, encoding="bgr8")  # Convert to ROS Image
            self.image_pub.publish(ros_image)  # Publish the image
        except Exception as e:
            rospy.logerr(f"Failed to convert and publish image: {e}")


if __name__ == "__main__":
    try:
        node = JunctionDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
