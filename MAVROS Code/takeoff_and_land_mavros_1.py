#!/usr/bin/env python

import rospy
from mavros_msgs.msg import CommandBool, SetMode
from mavros_msgs.srv import CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Header

# Global variables
current_pose = PoseStamped()
current_velocity = TwistStamped()

# Callback function for the current position
def pose_callback(msg):
    global current_pose
    current_pose = msg

# Callback function for the current velocity
def velocity_callback(msg):
    global current_velocity
    current_velocity = msg

# Function to arm the quadcopter
def arm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        arming_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        arming_service(True)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

