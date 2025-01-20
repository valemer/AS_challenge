import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from fla_msgs.msg import JunctionArray,GlobalPoint
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
        self.visited_locations = MarkerArray()


        # Min distance to say we visited a branch
        self.min_distance = 30
        self.postn_of_min_dist_pts = 30

        #Save the ids so we can delete from the rviz once passed.
        self.published_markers = {}

        self.list_len = 0


        rospy.init_node('branch_entrance_listener', anonymous=True)
        rospy.Subscriber('/junctions_array', JunctionArray, self.junctions_callback)
        #rospy.Subscriber('/visited_locations', MarkerArray, self.visited_locations_callback)
        rospy.Subscriber('/current_position',Point,self.current_position_callback)
        self.target_branch_entrance_pub = rospy.Publisher('/branch_entrance_superior_waypoint_target', GlobalPoint, queue_size=10)
        self.marker_pub = rospy.Publisher('/branch_entrances_markers', MarkerArray, queue_size=10)
        self.visited_locations_pub = rospy.Publisher('/visited_locations', MarkerArray, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(10.0), self.timer_callback)

    def current_position_callback(self, msg):
        current_position = np.array([msg.x, msg.y, msg.z])
        if not self.visited_locations.markers or np.linalg.norm(current_position - np.array([self.visited_locations.markers[-1].pose.position.x, self.visited_locations.markers[-1].pose.position.y, self.visited_locations.markers[-1].pose.position.z])) > 5.0:
            #self.visited_locations.append(msg)
            #rospy.loginfo(f"Added a visited point at{(msg.x,msg.y,msg.z)}")


            # Create a marker for the new visited point
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "visited_locations"
            marker.id = len(self.visited_locations.markers) - 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = msg
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            # Publish the updated visited locations
            
            self.visited_locations.markers.append(marker)
            self.visited_locations_pub.publish(self.visited_locations)

    
    
    
    def junctions_callback(self, msg):
        new_junctions = msg.junctions
        new_len = len(new_junctions)
        if new_len != self.list_len:
            diff = new_len - self.list_len
            markers = MarkerArray()
            for i in range(diff):
                junction = new_junctions[-(i+1)]
                for angle in junction.angles:
                    entrance_point = GlobalPoint()
                    # entrance_point.header=Header(
                    #         stamp=rospy.Time.now(),
                    #         frame_id="world"  # I dont know if we need it
                    #     )
                    entrance_point.point.x = junction.position.x + self.postn_of_min_dist_pts* math.cos(angle)
                    entrance_point.point.y = junction.position.y + self.postn_of_min_dist_pts* math.sin(angle)
                    entrance_point.point.z = junction.position.z
                    entrance_point.orientation = angle
                    entrance_point.velocity = -1 #Placeholder value
                    entrance_point.acceleration = -1#Placeholder value

                    self.branch_entrances.append(entrance_point)
                    rospy.loginfo(f"New branch entrance added: {entrance_point.point.x,entrance_point.point.y,entrance_point.point.z}")
                    self.branch_sources.append(new_len - i)
                                        # Create a marker for the new branch entrance
                    marker = Marker()
                    marker.header.frame_id = "world"
                    marker.header.stamp = rospy.Time.now()
                    marker.ns = "branch_entrances"
                    marker.id = len(self.branch_entrances) - 1
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                    marker.pose.position.x = entrance_point.point.x
                    marker.pose.position.y = entrance_point.point.y
                    marker.pose.position.z = entrance_point.point.z
                    marker.scale.x = self.min_distance
                    marker.scale.y = self.min_distance
                    marker.scale.z = self.min_distance
                    marker.color.r = 0.0
                    marker.color.g = 0.0
                    marker.color.b = 1.0
                    marker.color.a = 1.0
                    markers.markers.append(marker)

                    self.published_markers[marker.id] = marker


            self.marker_pub.publish(markers)
            self.list_len = new_len

    def visited_locations_callback(self, msg):
        self.visited_locations = [marker.pose.position for marker in msg.markers]

    def timer_callback(self, event):
        if not self.visited_locations or not self.branch_entrances:
            #rospy.logwarn("No visited locations or branch entrances received yet.")
            return

        # Convert branch entrances and visited locations to NumPy arrays
        branch_entrances_np = np.array([[entrance.point.x, entrance.point.y, entrance.point.z,entrance.orientation] for entrance in self.branch_entrances])
        visited_locations_np = np.array([[marker.pose.position.x, marker.pose.position.y, marker.pose.position.z] for marker in self.visited_locations.markers])
        #visited_locations_np = np.array([[loc.x, loc.y, loc.z] for loc in self.visited_locations])

        # Calculate distances between each branch entrance and each visited location
        distances = np.linalg.norm(
        branch_entrances_np[:, np.newaxis, :3] - visited_locations_np[np.newaxis, :, :], axis=2)

        # Find the minimum distance for each branch entrance
        min_distances = np.min(distances, axis=1)

        #For debuging, currently commented
        # smallest_distance_yet = np.min(min_distances)
        # rospy.loginfo(f'Smallest distance yet {smallest_distance_yet}')

        
        # Filter branch entrances and sources based on the minimum distance
        mask = min_distances > self.min_distance
        new_branch_entrances = branch_entrances_np[mask]
        new_branch_sources = np.array(self.branch_sources)[mask]



        # Delete markers for branch entrances that no longer exist
        markers_to_delete = MarkerArray()
        for marker_id in list(self.published_markers.keys()):
            if marker_id not in new_branch_sources:
                marker = self.published_markers.pop(marker_id)
                marker.action = Marker.DELETE
                markers_to_delete.markers.append(marker)

        if markers_to_delete.markers:
            self.marker_pub.publish(markers_to_delete)

        if set(self.branch_sources_old) > set(new_branch_sources):
            rospy.loginfo(f"Old Set {set(self.branch_sources_old)},New Set {set(new_branch_sources)}")
            for source in set(self.branch_sources_old) - set(new_branch_sources):
                rospy.loginfo(f"Used all branches for junction {source}")
            if len(new_branch_entrances) > 0:
                last_entrance = new_branch_entrances[-1]
                target_branch_entrance_point = GlobalPoint()
                target_branch_entrance_point.point.x = last_entrance[0]
                target_branch_entrance_point.point.y = last_entrance[1]
                target_branch_entrance_point.point.z = last_entrance[2]

                # Set orientation
                target_branch_entrance_point.orientation = last_entrance[3]
                rospy.loginfo(f"Target branch entrance waypoint {(target_branch_entrance_point.point.x,target_branch_entrance_point.point.y,target_branch_entrance_point.point.z)} with orientation {target_branch_entrance_point.orientation}")
                self.target_branch_entrance_pub.publish(target_branch_entrance_point)

        self.branch_entrances = [
                    GlobalPoint(
                        point=Point(
                            x=ent[0],
                            y=ent[1],
                            z=ent[2]
                        ),
                        orientation=ent[3],
                        velocity=-1,       # Dummy
                        acceleration=-1    # Dummy
                    )
                    for ent in new_branch_entrances
                ]
        

        #rospy.loginfo(f'Unvisited branch entrances: {self.branch_entrances}')
        self.branch_sources_old = self.branch_sources
        self.branch_sources = new_branch_sources.tolist()

if __name__ == '__main__':
    try:
        listener = BranchEntranceListener()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
