from copy import copy
from enum import Enum

import rospy
import tf.transformations
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from fla_msgs.msg import JunctionArray, GlobalPoint
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Bool
import numpy as np
import math

class State(Enum):
    OBSERVING = 1,
    CHANGING_PLACE = 2
    STOP = 3

class ExplorationManager:
    def __init__(self):
        self.entrances = []

        self.visited_locations = []
        self.goal = None
        self.current_position = None


        # Min distance to say we visited a branch
        self.min_distance = 30
        self.min_distance_visited = 10.0
        self.skip_for_loop_detection = 10

        rospy.init_node('exploration_manager', anonymous=True)

        rospy.Subscriber('/junctions_array', JunctionArray, self.junctions_callback)
        rospy.Subscriber('/current_state_est',Odometry,self.current_position_callback)
        rospy.Subscriber("control_planner", Bool, self.control)
        self.target_branch_entrance_pub = rospy.Publisher('/goal_point', GlobalPoint, queue_size=10)
        self.marker_pub = rospy.Publisher('/branch_entrances_markers', MarkerArray, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1), self.timer_callback)

        self.state = State.OBSERVING
    
    def control(self, msg):
        # if received False, stop the exploration
        if msg.data == False:
            self.state = State.STOP

    def current_position_callback(self, msg):
        self.current_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        if not self.visited_locations or np.linalg.norm(self.current_position - np.array(self.visited_locations[-1])) > 5.0:
            self.visited_locations.append(self.current_position)

    def pub_marker(self):
        markers = MarkerArray()
        id = 0
        for entrance_point in self.entrances:
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "branch_entrances"
            marker.id = id
            id += 1
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = entrance_point[0]
            marker.pose.position.y = entrance_point[1]
            marker.pose.position.z = entrance_point[2]
            quaternion_array = tf.transformations.quaternion_from_euler(0, 0, entrance_point[3])
            marker.pose.orientation.x = quaternion_array[0]
            marker.pose.orientation.y = quaternion_array[1]
            marker.pose.orientation.z = quaternion_array[2]
            marker.pose.orientation.w = quaternion_array[3]
            marker.scale.x = 16
            marker.scale.y = 2
            marker.scale.z = 2
            marker.color.r = entrance_point[4]
            marker.color.g = 1 - entrance_point[4]
            marker.color.b = 0.0
            marker.color.a = 1.0
            markers.markers.append(marker)
        self.marker_pub.publish(markers)
    
    
    def junctions_callback(self, msg):
        for jun in msg.junctions:
            for angle in jun.angles:
                new_entrance = np.array([jun.position.x + self.min_distance * math.cos(angle),
                                         jun.position.y + self.min_distance * math.sin(angle),
                                         jun.position.z,
                                         angle,
                                         False])
                if all(np.linalg.norm(entrance[:3] - new_entrance[:3]) > 5.0 for entrance in self.entrances):
                    self.entrances.append(new_entrance)

    def fly_to_not_explored_entrance(self):
        last_unvisited_entrance = None
        for entrance in self.entrances:
            if not entrance[4]:
                last_unvisited_entrance = entrance
        gp = GlobalPoint()
        gp.point.x = last_unvisited_entrance[0]
        gp.point.y = last_unvisited_entrance[1]
        gp.point.z = last_unvisited_entrance[2]
        gp.orientation = last_unvisited_entrance[3]

        self.target_branch_entrance_pub.publish(gp)

        rospy.loginfo(f"Sended GP: {gp.point.x, gp.point.y, gp.point.z}")

        self.goal = last_unvisited_entrance[:3]

    def timer_callback(self, event):
        if not self.entrances:
            return
        
        if self.state == State.STOP:
            return

        # update entrances whether they are visited (Marker: red = visited, green = need to be explored)
        for entrance in self.entrances:
            if any(np.linalg.norm(entrance[:3] - visited) < self.min_distance_visited for visited in self.visited_locations):
                entrance[4] = True

        self.pub_marker()

        if self.state == State.OBSERVING and len(self.visited_locations) > self.skip_for_loop_detection:
            if any(np.linalg.norm(old_location - self.visited_locations[-1]) < self.min_distance_visited for old_location in self.visited_locations[:-self.skip_for_loop_detection]):
                self.fly_to_not_explored_entrance()
                self.state = State.CHANGING_PLACE

        if self.state == State.CHANGING_PLACE and np.linalg.norm(self.visited_locations[-1] - self.goal) < self.min_distance_visited:
            self.state = State.OBSERVING


if __name__ == '__main__':
    try:
        listener = ExplorationManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
