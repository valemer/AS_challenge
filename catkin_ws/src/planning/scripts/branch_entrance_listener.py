import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray
from fla_msgs.msg import JunctionArray
import math

class BranchEntranceListener:
    def __init__(self):
        self.junctions = []

        # Blue dot positions
        self.branch_entrances = []

        # Blue dot correspondence to junctions
        self.branch_sources = []

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
                    self.branch_sources.append(new_len - i)
            self.list_len = new_len

    def visited_locations_callback(self, msg):
        self.visited_locations = [marker.pose.position for marker in msg.markers]

    def timer_callback(self, event):
        if not self.visited_locations or not self.branch_entrances:
            rospy.logwarn("No visited locations or branch entrances received yet.")
            return

        new_branch_entrances = []
        new_branch_sources = []

        # For each blue dot
        for i, entrance in enumerate(self.branch_entrances):
            min_distance = float('inf')

            # Compare with path points
            for point in self.visited_locations:
                distance = math.sqrt(
                    (entrance.x - point.x) ** 2 +
                    (entrance.y - point.y) ** 2 +
                    (entrance.z - point.z) ** 2
                )
                if distance < min_distance:
                    min_distance = distance

            if min_distance >= self.min_distance:  # Threshold distance to drop the blue dot
                new_branch_entrances.append(entrance)
                new_branch_sources.append(self.branch_sources[i])

        if set(new_branch_sources) != set(self.branch_sources):
            for source in set(self.branch_sources) - set(new_branch_sources):
                rospy.loginfo(f"Used all branches for junction {source}")
            if new_branch_entrances:
                # Maybe there would be no branches left and we need to return. But because of the task logic
                # when we go to all branches we will already obtain the lanterns and a new order will be given.
                rospy.loginfo(f"Emergency waypoint {new_branch_entrances[-i]}")
                self.emergency_pub.publish(new_branch_entrances[-1])
            

        self.branch_entrances = new_branch_entrances
        self.branch_sources = new_branch_sources

if __name__ == '__main__':
    try:
        listener = BranchEntranceListener()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
