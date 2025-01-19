#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
import math
from nav_msgs.msg import OccupancyGrid, Odometry
from skimage.morphology import skeletonize
from scipy.spatial.distance import cdist
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray  # <-- Added for arrow visualization in RViz

# -----------------------------
# Global variables and params
# -----------------------------
current_uav_height = 0.0

# Rolling storage of last N frames of detected junctions in (pos + orientation)
# Each element is a list of items: [ { 'pos': (x,y,z), 'angles': [float, float, ...] }, ... ]
#Changed from 15 to 8 because sometimes the data is too unreliable and we need to set it before it dissaperas
FRAMES_REQUIRED = 8
recent_junctions_world = []
MAX_STORED_FRAMES = FRAMES_REQUIRED

# Keep track of stable junctions (position + final orientations)
# stable_junctions will be a list of dicts, e.g.:
# [ { 'pos': (x, y, z), 'angles': [angle1, angle2, ...] }, ... ]
stable_junctions = []

# Main real-world distance threshold for stable junctions
REAL_WORLD_THRESHOLD = 10.0

# Use half this threshold for cross-frame consistency checks
cross_frame_threshold = REAL_WORLD_THRESHOLD / 2.0

# Publisher handles
confirmed_junction_pub = None
junction_marker_pub = None  # publisher for the MarkerArray (arrows)




# ----------------------------------------------------------------
# Orientation-related helpers
# ----------------------------------------------------------------

def mean_of_angles(radians_list):
    """
    Compute the mean angle (in radians) for the given list of angles.
    """
    s = sum(np.sin(a) for a in radians_list)
    c = sum(np.cos(a) for a in radians_list)
    return math.atan2(s, c)

def group_and_average_angles(angle_list, angle_gap_degs=20, max_groups=3):
    """
    Group angles that are 'close' (within angle_gap_degs), then average each group.
    Returns a final list of representative angles (in radians).
    """
    if not angle_list:
        return []

    # Convert angles to [-pi, pi], and sort
    angles = [math.atan2(math.sin(a), math.cos(a)) for a in angle_list]
    angles.sort()

    grouped = []
    current_group = [angles[0]]
    gap_radians = math.radians(angle_gap_degs)

    for a in angles[1:]:
        if abs(a - current_group[-1]) <= gap_radians:
            current_group.append(a)
        else:
            grouped.append(mean_of_angles(current_group))
            current_group = [a]

    grouped.append(mean_of_angles(current_group))

    # Limit the number of groups to max_groups
    if len(grouped) > max_groups:
        grouped = grouped[:max_groups]

    return grouped

# ----------------------------------------------------------------
# Position-related helpers
# ----------------------------------------------------------------

def within_threshold(p1, p2, threshold):
    """
    Returns True if p1 and p2 (each a tuple (x, y, z)) are within 'threshold' distance.
    """
    dist = math.dist(p1, p2)  # Python 3.8+: math.dist
    return dist < threshold

# ----------------------------------------------------------------
# ROS Callbacks
# ----------------------------------------------------------------

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
    # Free cells=0 => passable, obstacles or unknown => 1
    grid_map = (grid_data == 0).astype(np.uint8)

    # Flip horizontally (depending on your map conventions)
    grid_map = np.fliplr(grid_map)

    # Strengthen borders by dilating obstacles, smaller the borders, less risk of false positive
    kernel = np.ones((8, 8), np.uint8)
    grid_map = cv2.dilate(grid_map, kernel, iterations=1)

    # Detect junctions (position in pixel + orientation angles)
    junction_info_pixels, skeleton = detect_junctions_with_orientations(grid_map, width, height)

    # Filter to the central 25×25 region
    junction_info_pixels = filter_oriented_junctions_in_square(junction_info_pixels, width, height, 25)

    # Convert pixel coords => world coords, store in rolling buffer, check stability
    confirm_and_store_junctions(junction_info_pixels, resolution, origin, current_uav_height, width)

    # Publish stable junctions as points
    publish_all_stable_junctions()

    # Publish stable junctions' orientations as RViz arrows
    publish_junction_arrows(stable_junctions, resolution, origin, width)

    # OpenCV visualization
    visualize_junctions(grid_map, junction_info_pixels, skeleton, width, height, 25)

# ----------------------------------------------------------------
# Main logic for storing / confirming junctions
# ----------------------------------------------------------------

def confirm_and_store_junctions(junction_info_pixels, resolution, origin, height, width):
    """
    Convert newly detected junctions (pixel + orientation) => world coords + orientation,
    store them in a rolling buffer, and check if any remain stable across FRAMES_REQUIRED frames.
    Once stable, compute a final orientation set and add to stable_junctions.
    """
    global recent_junctions_world, stable_junctions

    # Convert pixel coords + angles => world coords + angles
    current_frame_junctions = []
    for (pix_y, pix_x, angle_list) in junction_info_pixels:
        # Convert pixel => world
        x_world = origin.position.x + (width - 1 - pix_x) * resolution
        y_world = origin.position.y + (pix_y * resolution)
        z_world = height
        current_frame_junctions.append({
            'pos': (x_world, y_world, z_world),
            'angles': angle_list[:]  # copy
        })

    # Add current frame's detections to rolling buffer
    recent_junctions_world.append(current_frame_junctions)
    if len(recent_junctions_world) > MAX_STORED_FRAMES:
        recent_junctions_world.pop(0)

    # Only proceed if we have FRAMES_REQUIRED frames
    if len(recent_junctions_world) < FRAMES_REQUIRED:
        return

    # The most recent set of junctions
    latest_junctions = recent_junctions_world[-1]

    # For each junction in the latest frame, check if it appears in ALL previous frames
    for junc in latest_junctions:
        pos_latest = junc['pos']
        angles_latest = junc['angles']

        # If already stable, skip
        if any(within_threshold(pos_latest, st['pos'], REAL_WORLD_THRESHOLD) for st in stable_junctions):
            continue

        found_in_all = True
        matched_angles_across_frames = [angles_latest]

        # Check older frames
        for older_frame_idx in range(len(recent_junctions_world) - 1):
            older_junctions = recent_junctions_world[older_frame_idx]
            matched_junc = None
            for oj in older_junctions:
                if within_threshold(pos_latest, oj['pos'], cross_frame_threshold):
                    matched_junc = oj
                    break
            if matched_junc is None:
                found_in_all = False
                break
            else:
                matched_angles_across_frames.append(matched_junc['angles'])

        # If found in all frames => stable
        if found_in_all:
            # Also ensure it's not too close to an existing stable junction
            if not any(within_threshold(pos_latest, st['pos'], REAL_WORLD_THRESHOLD) for st in stable_junctions):
                combined_angles = []
                for alist in matched_angles_across_frames:
                    combined_angles.extend(alist)

                # Group them so multiple outgoings are recognized, then average each group
                final_angles = group_and_average_angles(combined_angles, angle_gap_degs=20, max_groups=3)

                stable_junctions.append({
                    'pos': pos_latest,
                    'angles': final_angles
                })

                # Print the stable junction with orientations in degrees
                final_degs = [math.degrees(a) for a in final_angles]
                rospy.loginfo(
                    f"[junction_detection] New stable junction found at "
                    f"x={pos_latest[0]:.2f}, y={pos_latest[1]:.2f}, z={pos_latest[2]:.2f} "
                    f"with outgoings: {len(final_angles)} => {final_degs} degrees"
                )

# ----------------------------------------------------------------
# Publishing to RViz
# ----------------------------------------------------------------

def publish_all_stable_junctions():
    """
    Publish each stable junction as a PointStamped message.
    In RViz, to see multiple points, set the "Queue Size" or "Decay Time."
    """
    global confirmed_junction_pub
    if confirmed_junction_pub is None:
        confirmed_junction_pub = rospy.Publisher("/junction_points", PointStamped, queue_size=10)

    for st_junc in stable_junctions:
        (x, y, z) = st_junc['pos']
        msg = PointStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = rospy.Time.now()
        msg.point.x = x
        msg.point.y = y
        msg.point.z = z
        confirmed_junction_pub.publish(msg)


def publish_junction_arrows(junctions, resolution, origin, width):
    """
    Publish arrows representing junction exits for RViz as a MarkerArray.
    One arrow per outgoing direction.
    """
    global junction_marker_pub
    if junction_marker_pub is None:
        junction_marker_pub = rospy.Publisher("/junction_arrows", MarkerArray, queue_size=10)

    marker_array = MarkerArray()
    marker_id = 0

    for junction in junctions:
        (x, y, z) = junction['pos']
        for angle in junction['angles']:
            dx = 10.0 * math.cos(angle)
            dy = 10.0 * math.sin(angle)

            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "junction_arrows"
            marker.id = marker_id
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration(0)  # persistent in RViz

            # Arrow start (junction) and end (some meters away)
            start_x, start_y, start_z = x, y, z

            #Because the map is reversed, we should be careful
            end_x, end_y, end_z = x + dx, y + dy, z

            marker.points = []
            start_pt = PointStamped().point
            start_pt.x, start_pt.y, start_pt.z = start_x, start_y, start_z
            end_pt = PointStamped().point
            end_pt.x, end_pt.y, end_pt.z = end_x, end_y, end_z

            marker.points.append(start_pt)
            marker.points.append(end_pt)

            # Arrow dimensions
            marker.scale.x = 0.1  # shaft diameter
            marker.scale.y = 0.2  # head diameter
            marker.scale.z = 0.2  # head length

            # Arrow color (red)
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)
            marker_id += 1

    junction_marker_pub.publish(marker_array)

# ----------------------------------------------------------------
# Detect junctions + orientation (pixel domain)
# ----------------------------------------------------------------

def detect_junctions_with_orientations(grid_map, width, height, neighbor_threshold=14, min_distance=5):
    """
    1) Skeletonize the free space.
    2) Detect branch points as "junction" pixels.
    3) For each junction pixel, detect outgoings by scanning 0..360 in increments (e.g. 15 deg).
    4) Group & average angles => final directions for that pixel.

    Returns: list of (pix_y, pix_x, [angles]) + skeleton
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
        junction_map = pruned_junction_map

    final_junction_pixels = np.argwhere(junction_map > 0)

    junction_info_list = []
    for (py, px) in final_junction_pixels:
        # For each junction pixel, detect outgoings
        angles = detect_outgoings_for_junction(px, py, grid_map, width, height,
                                               distance_threshold=8,
                                               angle_step_degs=15,
                                               angle_gap_degs=20)
        junction_info_list.append((py, px, angles))

    return junction_info_list, skeleton

def detect_outgoings_for_junction(px, py, grid_map, width, height,
                                  distance_threshold=8,
                                  angle_step_degs=15,
                                  angle_gap_degs=20):
    """
    For a single junction pixel in the grid_map:
      1) Scan in 0..360 degrees (angle_step_degs increments).
      2) If we find distance_threshold consecutive free cells, record that angle.
      3) Group & average => final outgoings.
    """
    angles_found = []

    for deg in range(0, 360, angle_step_degs):
        rad = math.radians(deg)
        # NOTE: I added this - in front of dx because of the inversion in occupancy grid
        dx = -math.cos(rad)
        dy = math.sin(rad)

        free_count = 0
        for step in range(1, distance_threshold + 1):
            test_x = int(px + dx * step)
            test_y = int(py + dy * step)
            if 0 <= test_x < width and 0 <= test_y < height:
                if grid_map[test_y, test_x] == 0:
                    free_count += 1
                else:
                    break
            else:
                break
        if free_count >= distance_threshold:
            angles_found.append(rad)

    # Group similar angles
    grouped_angles = group_and_average_angles(angles_found, angle_gap_degs=angle_gap_degs)
    return grouped_angles

def cluster_junctions(points, distances, min_distance):
    """
    Cluster nearby junction pixels into single representative points.
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

def filter_oriented_junctions_in_square(junction_info, width, height, square_size):
    """
    Filter out junctions (py, px, angles) outside a central square of side 'square_size'.
    """
    margin_x = (width - square_size) // 2
    margin_y = (height - square_size) // 2
    filtered_list = []
    for (py, px, angles) in junction_info:
        if margin_y <= py < margin_y + square_size and margin_x <= px < margin_x + square_size:
            filtered_list.append((py, px, angles))
    return filtered_list

# ----------------------------------------------------------------
# Visualization
# ----------------------------------------------------------------

def visualize_junctions(grid_map, junction_info_pixels, skeleton, width, height, square_size):
    """
    Visualize in an OpenCV window the grid map, skeleton, and the detected junctions.
    """
    # Skeleton
    skeleton_display = cv2.resize((skeleton * 255).astype(np.uint8),
                                  (500, 500),
                                  interpolation=cv2.INTER_NEAREST)

    # Overlay junctions on the map
    colored_map = cv2.cvtColor((grid_map == 0).astype(np.uint8) * 255, cv2.COLOR_GRAY2BGR)
    red_color = (0, 0, 255)
    for (py, px, angles) in junction_info_pixels:
        cv2.circle(colored_map, (px, py), radius=2, color=red_color, thickness=-1)


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

# ----------------------------------------------------------------
# Main
# ----------------------------------------------------------------

if __name__ == "__main__":
    rospy.init_node("junction_detection_node")

    # Subscribe to UAV odometry
    rospy.Subscriber("/current_state_est", Odometry, uav_odom_callback)

    # Subscribe to OccupancyGrid
    rospy.Subscriber("/occupancy_grid", OccupancyGrid, occupancy_grid_callback)

    rospy.spin()
