import rospy
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid
from skimage.morphology import skeletonize
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import math

class DeadEndDetectionNode:
    def __init__(self):
        rospy.init_node("dead_end_detection_node")

        # ROS Publishers
        self.marker_pub = rospy.Publisher("/debug_marked_grids", MarkerArray, queue_size=10)
        self.dead_end_pub = rospy.Publisher("/dead_end_point", Marker, queue_size=10)

        self.radius = 25
        self.angle_range = np.deg2rad([-90, 90])  # -90° to 90°

        # ROS Subscriber
        rospy.Subscriber("/occupancy_grid", OccupancyGrid, self.occupancy_grid_callback)

    def occupancy_grid_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin

        # Transform occupancy data to a 2D grid
        grid_data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        grid_map = (grid_data == 0).astype(np.uint8)
        grid_map = np.fliplr(grid_map)

        # Define parameters
        grid_center_x = width // 2
        grid_center_y = height // 2

        # Create a mask for the semicircle
        mask = np.zeros_like(grid_map, dtype=np.uint8)
        for x in range(width):
            for y in range(height):
                dx = x - grid_center_x
                dy = grid_center_y - y  # Flip y to align with grid_map
                distance = np.hypot(dx, dy)
                angle = np.arctan2(dy, dx)
                if distance <= self.radius and self.angle_range[0] <= angle <= self.angle_range[1]:
                    mask[y, x] = 1

        # Mask the grid
        masked_grid = cv2.bitwise_and(grid_map, grid_map, mask=mask)

        # Debugging: Visualize the masked grid
        marker_array = MarkerArray()
        all_occupied = True
        for y in range(height):
            for x in range(width):
                if mask[y, x] == 1:
                    # Determine the grid cell's color (red for occupied, green for free)
                    is_not_occupied = masked_grid[y, x] == 0
                    # if not is_occupied:
                    #     all_occupied = False

                    # Add grid cell to the debug marker array
                    marker = Marker()
                    marker.header.frame_id = "world"
                    marker.header.stamp = rospy.Time.now()
                    marker.ns = "debug_marked_grids"
                    marker.id = len(marker_array.markers)
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    marker.pose.position.x = (x - grid_center_x) * resolution + origin.position.x
                    marker.pose.position.y = (y - grid_center_y) * resolution + origin.position.y
                    marker.pose.position.z = origin.position.z
                    marker.pose.orientation.w = 1.0
                    marker.scale.x = resolution
                    marker.scale.y = resolution
                    marker.scale.z = resolution
                    marker.color.a = 0.8
                    if is_not_occupied:
                        marker.color.r = 0.0
                        marker.color.g = 1.0
                        marker.color.b = 0.0
                    else:
                        marker.color.r = 1.0
                        marker.color.g = 0.0
                        marker.color.b = 0.0
                    marker_array.markers.append(marker)

        # Publish the debug markers
        self.marker_pub.publish(marker_array)
        
        # Detect dead end
        is_dead_end = self.detect_dead_end(grid_map, width, height, grid_center_x, grid_center_y, self.radius, resolution)

        # Publish the dead-end marker if detected
        if is_dead_end:
            dead_end_marker = Marker()
            dead_end_marker.header.frame_id = "world"
            dead_end_marker.header.stamp = rospy.Time.now()
            dead_end_marker.ns = "dead_end_point"
            dead_end_marker.id = 0
            dead_end_marker.type = Marker.SPHERE
            dead_end_marker.action = Marker.ADD
            dead_end_marker.pose.position.x = origin.position.x + radius * resolution
            dead_end_marker.pose.position.y = origin.position.y
            dead_end_marker.pose.position.z = origin.position.z
            dead_end_marker.pose.orientation.w = 1.0
            dead_end_marker.scale.x = resolution * 2
            dead_end_marker.scale.y = resolution * 2
            dead_end_marker.scale.z = resolution * 2
            dead_end_marker.color.a = 1.0
            dead_end_marker.color.r = 0.5
            dead_end_marker.color.g = 0.0
            dead_end_marker.color.b = 0.5

            self.dead_end_pub.publish(dead_end_marker)

    def detect_dead_end(self, grid_map, width, height, grid_center_x, grid_center_y, radius, resolution):
        """
        Detects if there is a dead end in the semicircular area in front of the grid center.

        Args:
            grid_map (numpy.ndarray): Occupancy grid map.
            width (int): Width of the grid map.
            height (int): Height of the grid map.
            grid_center_x (int): X-coordinate of the grid center.
            grid_center_y (int): Y-coordinate of the grid center.
            radius (int): Radius of the semicircle.
            resolution (float): Grid resolution (meters per cell).

        Returns:
            bool: True if it is a dead end, False otherwise.
        """
        all_occupied = True  # Assume the semicircle is fully occupied unless proven otherwise
        for angle in range(-90, 91, 5):  # Check in 5-degree increments
            radians = np.deg2rad(angle)
            dx = np.cos(radians)
            dy = np.sin(radians)
            for step in range(1, int(radius / resolution) + 1):
                nx = int(grid_center_x + step * dx)
                ny = int(grid_center_y - step * dy)  # Flip y to match grid map coordinates
                if 0 <= nx < width and 0 <= ny < height:
                    if grid_map[ny, nx] == 0:  # Free space
                        all_occupied = False
                        break
                else:
                    break  # Out of bounds
            if not all_occupied:
                break

        return all_occupied


if __name__ == "__main__":
    try:
        node = DeadEndDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
