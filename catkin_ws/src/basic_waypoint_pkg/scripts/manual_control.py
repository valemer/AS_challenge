#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Transform, Quaternion, Vector3
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from pynput import keyboard
import tf
import threading

# Global variables for current drone state
current_position = [0.0, 0.0, 1.0]  # Default position (x, y, z)
current_orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
stop_thread = False  # Flag to stop the keyboard listener

def state_callback(msg):
    """Callback to update the drone's current position and orientation."""
    global current_position, current_orientation
    current_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    current_orientation = msg.pose.orientation

def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles to a Quaternion."""
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    return Quaternion(*quaternion)

from geometry_msgs.msg import Twist

def publish_trajectory(x_offset, y_offset, z_offset, yaw_offset):
    global current_position, current_orientation

    # Convert current orientation to a rotation matrix
    quaternion = [
        current_orientation.x,
        current_orientation.y,
        current_orientation.z,
        current_orientation.w
    ]
    rotation_matrix = tf.transformations.quaternion_matrix(quaternion)

    # Local movement vector (in the drone's frame)
    local_movement = [x_offset, y_offset, z_offset, 1.0]

    # Transform local movement to global coordinates
    global_movement = rotation_matrix.dot(local_movement)
    new_position = [
        current_position[0] + global_movement[0],
        current_position[1] + global_movement[1],
        current_position[2] + global_movement[2]
    ]

    # Update yaw (rotation around Z-axis)
    current_euler = tf.transformations.euler_from_quaternion(quaternion)
    new_yaw = current_euler[2] + yaw_offset
    new_orientation = euler_to_quaternion(0, 0, new_yaw)

    # Create the trajectory message
    trajectory = MultiDOFJointTrajectory()
    trajectory.header.stamp = rospy.Time.now()
    trajectory.header.frame_id = "world"

    # Create the trajectory point
    transform = Transform()
    transform.translation.x = new_position[0]
    transform.translation.y = new_position[1]
    transform.translation.z = new_position[2]
    transform.rotation = new_orientation

    velocities = Twist()  # Zero velocities
    accelerations = Twist()  # Zero accelerations

    point = MultiDOFJointTrajectoryPoint(
        transforms=[transform],
        velocities=[velocities],
        accelerations=[accelerations],
        time_from_start=rospy.Duration(1.0)
    )

    # Add the point to the trajectory
    trajectory.points.append(point)

    # Publish the trajectory
    trajectory_pub.publish(trajectory)
    rospy.loginfo(f"Published trajectory: position={new_position}, yaw={new_yaw}")

def on_press(key):
    """Handle keyboard input to publish new trajectories."""
    try:
        if hasattr(key, 'char') and key.char:
            mul = 5
            if key.char == 'w':  # Move forward
                publish_trajectory(mul, 0.0, 0.0, 0.0)
            elif key.char == 's':  # Move backward
                publish_trajectory(-mul, 0.0, 0.0, 0.0)
            elif key.char == 'a':  # Move left
                publish_trajectory(0.0, mul, 0.0, 0.0)
            elif key.char == 'd':  # Move right
                publish_trajectory(0.0, -mul, 0.0, 0.0)
            elif key.char == 'q':  # Turn left (yaw +1 radian)
                publish_trajectory(0.0, 0.0, 0.0, 1.0)
            elif key.char == 'e':  # Turn right (yaw -1 radian)
                publish_trajectory(0.0, 0.0, 0.0, -1.0)
            elif key.char == 'r':  # Move up
                publish_trajectory(0.0, 0.0, mul, 0.0)
            elif key.char == 'f':  # Move down
                publish_trajectory(0.0, 0.0, -mul, 0.0)
    except Exception as e:
        rospy.logerr(f"Error processing key input: {e}")

def keyboard_listener():
    """Run the keyboard listener in a separate thread."""
    global stop_thread
    with keyboard.Listener(on_press=on_press) as listener:
        while not stop_thread and not rospy.is_shutdown():
            rospy.sleep(0.1)
        listener.stop()

def main():
    global stop_thread
    rospy.init_node('keyboard_trajectory_node')

    # Subscriber to current state
    rospy.Subscriber('/true_pose', PoseStamped, state_callback)

    # Publisher for trajectory commands
    global trajectory_pub
    trajectory_pub = rospy.Publisher('/command/trajectory', MultiDOFJointTrajectory, queue_size=10)

    rospy.loginfo("Node started. Use W/S/A/D for direction, Q/E for yaw, R/F for height control.")

    # Start the keyboard listener in a separate thread
    listener_thread = threading.Thread(target=keyboard_listener)
    listener_thread.start()

    try:
        rospy.spin()  # Keep the node running
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down node...")
        stop_thread = True  # Signal the keyboard listener to stop
        listener_thread.join()  # Wait for the listener to finish

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
