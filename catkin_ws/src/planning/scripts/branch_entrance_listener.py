import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray
from fla_msgs.msg import JunctionArray
import numpy as np
import math

class BranchEntranceListener:
    def __init__(self):
        self.junctions = []

        # Blue dot positions
        self.branch_entrances = []
        

        # Blue dot correspondence to junctions
        self.branch_sources = []
        self.branch_sources_old = []

        # Path we have moved along
        self.visited_locations = []

        # Min distance to say we visited a branch
        self.min_distance = 10

        self.list_len = 0

        rospy.init_node('branch_entrance_listener', anonymous=True)
        rospy.Subscriber('/junctions_array', JunctionArray, self.junctions_callback)
        rospy.Subscriber('/visited_locations', MarkerArray, self.visited_locations_callback)
        self.emergency_pub = rospy.Publisher('/emergency_superior_waypoint_target', Point, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(10.0), self.timer_callback)

    def junctions_callback(self, msg):
        new_junctions = msg.junctions
        new_len = len(new_junctions)
        if new_len != self.list_len:
            diff = new_len - self.list_len
            for i in range(diff):
                junction = new_junctions[-(i+1)]
                for angle in junction.angles:
                    entrance_point = Point()
                    entrance_point.x = junction.position.x + 10.0 * math.cos(angle)
                    entrance_point.y = junction.position.y + 10.0 * math.sin(angle)
                    entrance_point.z = junction.position.z
                    self.branch_entrances.append(entrance_point)
                    rospy.loginfo(f"New branch entrance added: {entrance_point}")
                    self.branch_sources.append(new_len - i)
            self.list_len = new_len

    def visited_locations_callback(self, msg):
        self.visited_locations = [marker.pose.position for marker in msg.markers]

    def timer_callback(self, event):
        if not self.visited_locations or not self.branch_entrances:
            rospy.logwarn("No visited locations or branch entrances received yet.")
            return

        # Convert branch entrances and visited locations to NumPy arrays
        branch_entrances_np = np.array([[entrance.x, entrance.y, entrance.z] for entrance in self.branch_entrances])
        visited_locations_np = np.array([[loc.x, loc.y, loc.z] for loc in self.visited_locations])

        # Calculate distances between each branch entrance and each visited location
        distances = np.linalg.norm(branch_entrances_np[:, np.newaxis, :] - visited_locations_np[np.newaxis, :, :], axis=2)

        # Find the minimum distance for each branch entrance
        min_distances = np.min(distances, axis=1)

        # Filter branch entrances and sources based on the minimum distance
        mask = min_distances > self.min_distance
        new_branch_entrances = branch_entrances_np[mask]
        new_branch_sources = np.array(self.branch_sources)[mask]

        if set(self.branch_sources_old) != set(new_branch_sources):
            rospy.loginfo(f"Old Set {set(self.branch_sources_old)},New Set {set(new_branch_sources)}")
            for source in set(self.branch_sources_old) - set(new_branch_sources):
                rospy.loginfo(f"Used all branches for junction {source}")
            if len(new_branch_entrances) > 0:
                last_entrance = new_branch_entrances[-1]
                emergency_point = Point(x=last_entrance[0], y=last_entrance[1], z=last_entrance[2])
                rospy.loginfo(f"Emergency waypoint {emergency_point}")
                self.emergency_pub.publish(emergency_point)

        self.branch_entrances = [Point(x=entrance[0], y=entrance[1], z=entrance[2]) for entrance in new_branch_entrances]
        self.branch_sources_old = self.branch_sources
        self.branch_sources = new_branch_sources.tolist()

if __name__ == '__main__':
    try:
        listener = BranchEntranceListener()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
