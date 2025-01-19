import rospy
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid
from skimage.morphology import skeletonize
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
import math

# Global variable to store UAV height
current_uav_height = 0.0

detected_junctions = []

min_dis_new_junction = 30.0



def uav_odom_callback(msg):
    """
    Callback to update the UAV's current height from odometry.
    """
    global current_uav_height
    current_uav_height = msg.pose.pose.position.z


def close_angle(angle1, angle2, old_size):
    # Calculate the absolute difference
    diff = abs(angle1 - angle2)

    # Account for circular difference (e.g., 10 and 340 degrees)
    diff = min(diff, 2 * math.pi - diff)

    # Check if the difference is within Pi/4
    if diff <= math.pi / 4:
        x = old_size * math.cos(angle1) + math.cos(angle2)
        y = old_size * math.sin(angle1) + math.sin(angle2)
        weighted_mean_angle = math.atan2(y, x) % (2 * math.pi)
        return weighted_mean_angle
    else:
        return None


def add_to_detected_junctions(new_detected_junctions):
    for new_det_jun in new_detected_junctions:
        for junction in detected_junctions:
            old_counter = junction['counter']
            if np.linalg.norm(junction['position'] - new_det_jun[0]) < min_dis_new_junction:
                junction['position'] = (new_det_jun[0] + junction['position'] * old_counter) / (old_counter + 1)
                new_orientations = []
                for orientation in new_det_jun[1]:
                    for old_orientation in junction['orientations']:
                        angle = close_angle(old_orientation, orientation, old_counter)
                        if angle is not None:
                            new_orientations.append(angle)
                junction['orientations'] = new_orientations
                junction['counter'] = old_counter + 1
                return

        detected_junctions.append({'position': new_det_jun[0], 'orientations': new_det_jun[1], 'counter': 1})

def tf_to_world_coords(junctions, resolution, origin, width):

    world_junctions = []

    for (y_index, x_index), orientations in junctions:
        # Calculate world position of the junction
        x_world = origin.position.x + ((width - 1 - x_index) * resolution)
        y_world = origin.position.y + (y_index * resolution)
        z_world = current_uav_height

        world_junctions.append((np.array([x_world, y_world, z_world]), orientations))

    return world_junctions


def occupancy_grid_callback(msg):
    """
    Callback to process the occupancy grid and detect junctions.
    """
    width = msg.info.width
    height = msg.info.height
    resolution = msg.info.resolution
    origin = msg.info.origin  # This is a geometry_msgs/Pose

    # Convert OccupancyGrid data to a 2D NumPy array
    grid_data = np.array(msg.data, dtype=np.int8).reshape((height, width))
    grid_map = (grid_data == 0).astype(np.uint8)  # Free space is 0, obstacles are 1

    # Flip the grid map horizontally
    grid_map = np.fliplr(grid_map)

    # Strengthen borders by dilating the obstacles
    kernel = np.ones((5, 5), np.uint8)  # Increased kernel size to thicken occupied areas
    grid_map = cv2.dilate(grid_map, kernel, iterations=1)  # Increased iterations for thicker obstacles

    # Detect junctions and their orientations
    junction_orientations, skeleton = detect_junctions_with_orientations(grid_map, width, height)

    # Filter junctions outside the inner 25x25 square
    junction_orientations = filter_oriented_junctions_in_square(junction_orientations, width, height, 25)

    # Filter junctions with less than three outgoings
    filtered_junction_orientations = [entry for entry in junction_orientations if len(entry[1]) >= 3]

    junction_orientations_in_world_frame = tf_to_world_coords(filtered_junction_orientations, resolution, origin, width)

    add_to_detected_junctions(junction_orientations_in_world_frame)

    publish_final_junctions()

    # Publish junction arrows as RViz markers
    publish_junction_arrows()

    # Visualize the results
    visualize_junctions(grid_map, filtered_junction_orientations, skeleton, width, height, 25)


def mean_of_angles(radians):
    # Calculate the average sine and cosine
    avg_sin = sum(math.sin(r) for r in radians) / len(radians)
    avg_cos = sum(math.cos(r) for r in radians) / len(radians)

    # Compute the mean angle in radians
    return math.atan2(avg_sin, avg_cos)


def detect_junctions_with_orientations(grid_map, width, height, neighbor_threshold=14, distance_threshold=10):
    """
    Detect junctions and their orientations by checking in 15-degree increments around each junction point.

    Parameters:
        grid_map: 2D NumPy array (0 = free, 1 = occupied).
        width: Width of the grid map.
        height: Height of the grid map.
        neighbor_threshold: Minimum neighbors for a pixel to qualify as a junction.
        distance_threshold: Number of free cells required in a direction to confirm an exit.

    Returns:
        junction_orientations: List of tuples (junction position, list of grouped exit orientations in radians).
        skeleton: Skeletonized version of the free space.
    """
    free_space = (grid_map == 0).astype(np.uint8)
    skeleton = skeletonize(free_space).astype(np.uint8)

    # Detect branch points in the skeleton
    kernel = np.array([[1, 1, 1],
                       [1, 10, 1],
                       [1, 1, 1]], dtype=np.uint8)
    neighbor_count = cv2.filter2D(skeleton, -1, kernel)
    junction_map = ((neighbor_count >= neighbor_threshold) & (skeleton > 0)).astype(np.uint8)

    # Find junction points
    junction_points = np.argwhere(junction_map > 0)

    # Detect orientations for each junction
    junction_orientations = []
    for y, x in junction_points:
        orientations = []
        current_group = []
        groups = []
        for index, angle in enumerate(range(0, 360, 15)):  # 15-degree increments
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
                # Add angle to the current group
                current_group.append({'idx': index, 'angle': math.atan2(dy, -dx)})
            else:
                # If the current group has entries, calculate the mean and start a new group
                if current_group:
                    groups.append(current_group)
                    current_group = []

        # Finalize the last group
        if current_group:
            if len(groups) > 0 and groups[0][0]['idx'] == 0:
                groups[0].extend(current_group)
            else:
                groups.append(current_group)

        for group in groups:
            orientations.append(mean_of_angles([item["angle"] for item in group]))


        junction_orientations.append(((y, x), orientations))

    return junction_orientations, skeleton


def filter_oriented_junctions_in_square(junction_orientations, width, height, square_size):
    """
    Filter out junctions and their orientations outside a central square of the given size.

    Parameters:
        junction_orientations: List of tuples (junction position, list of exit orientations in radians).
        width: Width of the map.
        height: Height of the map.
        square_size: Size of the central square.

    Returns:
        Filtered list of junctions with orientations.
    """
    margin_x = (width - square_size) // 2
    margin_y = (height - square_size) // 2
    filtered_orientations = []
    for (y, x), orientations in junction_orientations:
        if margin_y <= y < margin_y + square_size and margin_x <= x < margin_x + square_size:
            filtered_orientations.append(((y, x), orientations))
    return filtered_orientations


def publish_junction_arrows():
    """
    Publish arrows representing junction exits for RViz.

    Parameters:
        junction_orientations: List of tuples (junction position, list of exit orientations in radians).
        resolution: Resolution of the grid map (meters per cell).
        origin: Origin of the map in the world frame (geometry_msgs/Pose).
        height: Current height of the UAV (z-coordinate).
        width: Width of the grid map.
    """
    marker_pub = rospy.Publisher("/junction_arrows", Marker, queue_size=10)
    marker_id = 0

    for pos, orientations in detected_junctions:
        # Calculate world position of the junction
        x_world = pos[0]
        y_world = pos[1]
        z_world = pos[2]

        for angle in orientations:
            # Calculate the arrow's end point (5m length in the direction of the angle)
            dx = 5.0 * math.cos(angle)
            dy = 5.0 * math.sin(angle)

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
            marker.scale.x = 0.1  # Shaft diameter
            marker.scale.y = 0.2  # Head diameter
            marker.scale.z = 0.2  # Head length
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0  # Fully opaque

            marker_pub.publish(marker)
            marker_id += 1


def publish_final_junctions():
    for junction in detected_junctions:
        if junction['counter'] > 10:
            position = junction['position']
            orientations = junction['orientations']
            print(f'Junction detected ad: {position} with {orientations}')


def visualize_junctions(grid_map, junction_orientations, skeleton, width, height, square_size):
    """
    Visualize the grid map, skeleton, and junctions with the filtering square.
    """
    # Resize and display skeleton
    skeleton_display = cv2.resize((skeleton * 255).astype(np.uint8), (500, 500), interpolation=cv2.INTER_NEAREST)

    # Overlay junctions on the grid map
    colored_map = cv2.cvtColor((grid_map == 0).astype(np.uint8) * 255, cv2.COLOR_GRAY2BGR)
    red_color = [0, 0, 255]
    for (y, x), _ in junction_orientations:
        cv2.circle(colored_map, (x, y), radius=2, color=red_color, thickness=-1)

    # Draw the filtering square outline (1-pixel thick)
    margin_x = (width - square_size) // 2
    margin_y = (height - square_size) // 2
    cv2.rectangle(colored_map, (margin_x, margin_y), (margin_x + square_size - 1, margin_y + square_size - 1),
                  (0, 255, 0), 1)

    # Resize and display the map with junctions
    highlighted_map = cv2.resize(colored_map, (500, 500), interpolation=cv2.INTER_NEAREST)

    # Show images
    cv2.imshow("Skeleton (Resized)", skeleton_display)
    cv2.imshow("Junctions Highlighted (Resized)", highlighted_map)
    cv2.waitKey(1)


if __name__ == "__main__":
    rospy.init_node("junction_detection_node")

    # Subscribe to the UAV odometry topic
    rospy.Subscriber("/current_state_est", Odometry, uav_odom_callback)

    # Subscribe to the occupancy grid topic
    rospy.Subscriber("/occupancy_grid", OccupancyGrid, occupancy_grid_callback)

    rospy.spin()
