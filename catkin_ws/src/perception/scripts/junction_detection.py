#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid, Odometry  # <-- Odometry is from nav_msgs
from skimage.morphology import skeletonize
from scipy.spatial.distance import cdist
from geometry_msgs.msg import PointStamped
# Removed Marker import since we are returning to PointStamped

# -----------------------------
# Global variables and params
# -----------------------------
current_uav_height = 0.0

# Rolling storage of last N frames of detected junctions in **world** coords
FRAMES_REQUIRED = 15
recent_junction_points_world = []
MAX_STORED_FRAMES = FRAMES_REQUIRED

# Keep track of stable junctions in world coordinates (once confirmed)
stable_junctions = []  # Will store tuples (x_world, y_world, z_world)

# Main real-world distance threshold for stable junctions
REAL_WORLD_THRESHOLD = 10.0

# threshold for cross-frame consistency checks
cross_frame_threshold = REAL_WORLD_THRESHOLD / 2.0

# Publisher handle
confirmed_junction_pub = None


# -----------------------------
# Helper functions
# -----------------------------
def within_threshold(p1, p2, threshold):
    """
    Returns True if p1 and p2 (each a tuple (x, y, z)) are within 'threshold' distance.
    """
    dist = np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 + (p1[2] - p2[2])**2)
    return dist < threshold


def uav_odom_callback(msg):
    """
    Callback to update the UAV's current height from odometry.
    """
    global current_uav_height
    current_uav_height = msg.pose.pose.position.z


def occupancy_grid_callback(msg):
    """
    Callback to process the occupancy grid and detect junctions.
    """
    width = msg.info.width
    height = msg.info.height
    resolution = msg.info.resolution
    origin = msg.info.origin  # geometry_msgs/Pose

    # Convert OccupancyGrid data to a 2D NumPy array
    grid_data = np.array(msg.data, dtype=np.int8).reshape((height, width))
    # Free cells=0 => passable, mark them as 0 in a binary map; obstacle or unknown => 1
    grid_map = (grid_data == 0).astype(np.uint8)

    # Flip horizontally (depending on your map conventions)
    grid_map = np.fliplr(grid_map)

    # Strengthen borders by dilating obstacles
    kernel = np.ones((8, 8), np.uint8)
    grid_map = cv2.dilate(grid_map, kernel, iterations=1)

    # Detect junctions
    junction_map, skeleton = detect_junctions_with_curves(grid_map, width, height)

    # Filter to the central 25x25 region
    junction_map = filter_junctions_in_square(junction_map, width, height, 25)

    # Convert to pixel coords
    new_junction_points_pixels = np.argwhere(junction_map > 0)

    # Confirm and store junctions in real-world coords
    confirm_and_store_junctions(
        new_junction_points_pixels, resolution, origin, current_uav_height, width
    )

    # Publish all stable junctions (old + new) as separate PointStamped messages
    publish_all_stable_junctions()

    # Visualization
    visualize_junctions(grid_map, junction_map, skeleton, width, height, 25)


def confirm_and_store_junctions(junction_points_pixels, resolution, origin, height, width):
    """
    Convert the newly detected junctions from pixel to world coords, 
    keep them in a rolling buffer, and check if any remain stable 
    (present in all frames) => add to stable_junctions.
    """
    global recent_junction_points_world, stable_junctions

    # Convert pixel coords to world coords for this frame
    current_frame_world_points = []
    for (jy, jx) in junction_points_pixels:
        x_world = origin.position.x + (width - 1 - jx) * resolution
        y_world = origin.position.y + (jy * resolution)
        z_world = height
        current_frame_world_points.append((x_world, y_world, z_world))

    # Add current frame's detections to rolling list
    recent_junction_points_world.append(current_frame_world_points)
    if len(recent_junction_points_world) > MAX_STORED_FRAMES:
        recent_junction_points_world.pop(0)

    # Only proceed if we have N=FRAMES_REQUIRED frames stored
    if len(recent_junction_points_world) < FRAMES_REQUIRED:
        return

    # The most recent set of junctions in world coords
    latest_junctions_world = recent_junction_points_world[-1]

    # For each junction in the latest frame, check if it appears in all previous frames
    # within cross_frame_threshold (REAL_WORLD_THRESHOLD/2).
    for junction_w in latest_junctions_world:
        # If already stable, skip
        if any(within_threshold(junction_w, sj, REAL_WORLD_THRESHOLD) for sj in stable_junctions):
            continue

        # Check the preceding frames to see if there's a matching junction
        # in each older frame
        found_in_all_previous = True
        for older_frame_idx in range(len(recent_junction_points_world) - 1):
            older_junctions_world = recent_junction_points_world[older_frame_idx]
            # Are we within cross_frame_threshold for at least one junction in that older frame?
            if not any(within_threshold(junction_w, oj, cross_frame_threshold) for oj in older_junctions_world):
                found_in_all_previous = False
                break

        # If found in all frames => stable
        if found_in_all_previous:
            # Check if it's at least REAL_WORLD_THRESHOLD away from all stable junctions
            # (so we don't duplicate an already stable junction)
            if not any(within_threshold(junction_w, sj, REAL_WORLD_THRESHOLD) for sj in stable_junctions):
                stable_junctions.append(junction_w)
                rospy.loginfo(
                    f"[junction_detection] New stable junction found: x={junction_w[0]:.2f}, "
                    f"y={junction_w[1]:.2f}, z={junction_w[2]:.2f}"
                )


def publish_all_stable_junctions():
    """
    Publish each stable junction as a separate PointStamped message.
    
    """
    global confirmed_junction_pub, stable_junctions

    # Create publisher if not yet created
    if confirmed_junction_pub is None:
        # Adjust 'queue_size' if you like, 
        # but the main setting is in RViz (History length / Decay Time).
        confirmed_junction_pub = rospy.Publisher("/junction_points", PointStamped, queue_size=10)

    # Publish each stable junction individually
    for (x, y, z) in stable_junctions:
        msg = PointStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = rospy.Time.now()
        msg.point.x = x
        msg.point.y = y
        msg.point.z = z
        confirmed_junction_pub.publish(msg)


def detect_junctions_with_curves(grid_map, width, height, neighbor_threshold=14, min_distance=5):
    """
    Skeletonize the free space, then detect junctions as branch points.
    """
    free_space = (grid_map == 0).astype(np.uint8)
    skeleton = skeletonize(free_space).astype(np.uint8)

    # Detect branch points in the skeleton
    kernel = np.array([[1, 1, 1],
                       [1, 10, 1],
                       [1, 1, 1]], dtype=np.uint8)
    neighbor_count = cv2.filter2D(skeleton, -1, kernel)
    junction_map = ((neighbor_count >= neighbor_threshold) & (skeleton > 0)).astype(np.uint8)

    # Optionally cluster close junctions
    junction_points = np.argwhere(junction_map > 0)
    if len(junction_points) > 1:
        distances = cdist(junction_points, junction_points)
        clusters = cluster_junctions(junction_points, distances, min_distance)
        pruned_junction_map = np.zeros_like(junction_map)
        for point in clusters:
            pruned_junction_map[tuple(point)] = 1
        return pruned_junction_map, skeleton

    return junction_map, skeleton


def cluster_junctions(points, distances, min_distance):
    """
    Cluster nearby junction points into single representative points.
    """
    clustered_points = []
    visited = set()
    for i, point in enumerate(points):
        if i in visited:
            continue
        cluster = [point]
        visited.add(i)
        for j, dist in enumerate(distances[i]):
            if dist < min_distance and j not in visited:
                cluster.append(points[j])
                visited.add(j)
        cluster_center = np.mean(cluster, axis=0).astype(int)
        clustered_points.append(cluster_center)
    return np.array(clustered_points)


def filter_junctions_in_square(junction_map, width, height, square_size):
    """
    Filter out junctions outside a central square of side 'square_size'.
    """
    margin_x = (width - square_size) // 2
    margin_y = (height - square_size) // 2
    filtered_junction_map = np.zeros_like(junction_map)

    coords = np.argwhere(junction_map > 0)
    for y, x in coords:
        if margin_y <= y < margin_y + square_size and margin_x <= x < margin_x + square_size:
            filtered_junction_map[y, x] = 1
    return filtered_junction_map


def visualize_junctions(grid_map, junction_map, skeleton, width, height, square_size):
    """
    Visualize in a CV window the grid map, skeleton, and the detected junctions.
    """
    # Resize and display skeleton
    skeleton_display = cv2.resize((skeleton * 255).astype(np.uint8),
                                  (500, 500),
                                  interpolation=cv2.INTER_NEAREST)

    # Overlay junctions on the free-space map
    colored_map = cv2.cvtColor((grid_map == 0).astype(np.uint8) * 255, cv2.COLOR_GRAY2BGR)
    red_color = [0, 0, 255]
    coords = np.argwhere(junction_map > 0)
    for (y, x) in coords:
        cv2.circle(colored_map, (x, y), radius=2, color=red_color, thickness=-1)

    # Draw filtering square
    margin_x = (width - square_size) // 2
    margin_y = (height - square_size) // 2
    cv2.rectangle(colored_map,
                  (margin_x, margin_y),
                  (margin_x + square_size - 1, margin_y + square_size - 1),
                  (0, 255, 0), 1)

    highlighted_map = cv2.resize(colored_map, (500, 500), interpolation=cv2.INTER_NEAREST)

    cv2.imshow("Skeleton (Resized)", skeleton_display)
    cv2.imshow("Junctions Highlighted (Resized)", highlighted_map)
    cv2.waitKey(1)


if __name__ == "__main__":
    rospy.init_node("junction_detection_node")

    # Subscribe to UAV odometry
    rospy.Subscriber("/current_state_est", Odometry, uav_odom_callback)

    # Subscribe to OccupancyGrid
    rospy.Subscriber("/occupancy_grid", OccupancyGrid, occupancy_grid_callback)

    rospy.spin()
