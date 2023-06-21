import rospy
from mavros_msgs.msg import WaypointList, Waypoint
from mavros_msgs.srv import WaypointPush, WaypointClear

def main():
    # Initialize ROS node
    rospy.init_node('waypoint_simulation')
    rospy.loginfo("Waypoint simulation node created")

    # Create a publisher to send the waypoint list
    waypoint_pub = rospy.Publisher('/mavros/mission/waypoints', WaypointList, queue_size=10)

    # Create a service proxy to push waypoints to the autopilot
    waypoint_push = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)

    # Create a service proxy to clear waypoints from the autopilot
    waypoint_clear = rospy.ServiceProxy('/mavros/mission/clear', WaypointClear)

    # Wait for the services to become available
    rospy.wait_for_service('/mavros/mission/push')
    rospy.wait_for_service('/mavros/mission/clear')

    # Clear existing waypoints from the autopilot
    waypoint_clear()

    # Create a waypoint list
    waypoints = WaypointList()

    # Set the waypoints' frame of reference (0: global, 3: mission)
    waypoints.waypoints_frame = 0

    # Create the starting waypoint
    start_waypoint = Waypoint()
    start_waypoint.frame = 0  # Global frame
    start_waypoint.command = 16  # Takeoff command
    start_waypoint.is_current = True
    start_waypoint.autocontinue = True
    start_waypoint.param1 = 5.0  # Takeoff altitude
    waypoints.waypoints.append(start_waypoint)

    # Create the destination waypoint
    dest_waypoint = Waypoint()
    dest_waypoint.frame = 0  # Global frame
    dest_waypoint.command = 16  # Waypoint command
    dest_waypoint.is_current = False
    dest_waypoint.autocontinue = True
    dest_waypoint.x_lat = 28.382129  # Destination latitude
    dest_waypoint.y_long = 36.482940  # Destination longitude
    dest_waypoint.z_alt = 10.0  # Destination altitude
    waypoints.waypoints.append(dest_waypoint)

    # Push the waypoints to the autopilot
    waypoint_push(waypoints)

    # Publish the waypoint list to start the mission
    waypoint_pub.publish(waypoints)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
