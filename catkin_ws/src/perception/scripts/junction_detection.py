import rospy
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid
from skimage.morphology import skeletonize
from scipy.spatial.distance import cdist

def occupancy_grid_callback(msg):
    """
    Callback to process the occupancy grid and detect junctions.
    """
    width = msg.info.width
    height = msg.info.height

    # Convert OccupancyGrid data to a 2D NumPy array
    grid_data = np.array(msg.data, dtype=np.int8).reshape((height, width))
    grid_map = (grid_data == 0).astype(np.uint8)  # Free space is 0, obstacles are 1

    # Flip the grid map horizontally
    grid_map = np.fliplr(grid_map)

    # Strengthen borders by dilating the obstacles
    kernel = np.ones((5, 5), np.uint8)  # Increased kernel size to thicken occupied areas
    grid_map = cv2.dilate(grid_map, kernel, iterations=1)  # Increased iterations for thicker obstacles

    # Detect junctions
    junction_map, skeleton = detect_junctions_with_curves(grid_map)

    # Visualize the results
    visualize_junctions(grid_map, junction_map, skeleton)

def detect_junctions_with_curves(grid_map, neighbor_threshold=14, min_distance=5):
    """
    Detect junctions on maps using skeletonization.

    Parameters:
        grid_map: 2D NumPy array (0 = free, 1 = occupied).
        neighbor_threshold: Minimum neighbors for a pixel to qualify as a junction.
        min_distance: Minimum distance to separate nearby junctions.

    Returns:
        junction_map: Binary map marking detected junctions.
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

    # Cluster close junctions
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

def visualize_junctions(grid_map, junction_map, skeleton):
    """
    Visualize the grid map, skeleton, and junctions.
    """
    # Resize and display skeleton
    skeleton_display = cv2.resize((skeleton * 255).astype(np.uint8), (500, 500), interpolation=cv2.INTER_NEAREST)

    # Overlay junctions on the grid map
    colored_map = cv2.cvtColor((grid_map == 0).astype(np.uint8) * 255, cv2.COLOR_GRAY2BGR)
    red_color = [0, 0, 255]
    junction_points = np.argwhere(junction_map > 0)
    for y, x in junction_points:
        cv2.circle(colored_map, (x, y), radius=2, color=red_color, thickness=-1)

    # Resize and display the map with junctions
    highlighted_map = cv2.resize(colored_map, (500, 500), interpolation=cv2.INTER_NEAREST)

    # Show images
    cv2.imshow("Skeleton (Resized)", skeleton_display)
    cv2.imshow("Junctions Highlighted (Resized)", highlighted_map)
    cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node("junction_detection_node")

    # Subscribe to the occupancy grid topic
    rospy.Subscriber("/occupancy_grid", OccupancyGrid, occupancy_grid_callback)

    rospy.spin()
