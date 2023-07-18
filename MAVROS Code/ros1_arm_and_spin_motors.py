#!/usr/bin/env python3

import rospy
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import ActuatorControl

def arm_and_spin_motors():
    rospy.init_node('arm_and_spin_motors', anonymous=True)

    arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    mode = "GUIDED"

    arming_client(True)

    while not rospy.is_shutdown():
        if arming_client().success:
            rospy.loginfo("Copter armed successfully!")
            break
        rospy.loginfo("Arming failed. Retrying...")
        rospy.sleep(1)

    set_mode_client(base_mode=0, custom_mode=mode)

    actuator_pub = rospy.Publisher('/mavros/setpoint_raw/actuator_control', ActuatorControl, queue_size=10)
    rospy.sleep(1)  

    actuator_msg = ActuatorControl()
    actuator_msg.header.stamp = rospy.Time.now()
    actuator_msg.group_mix = 0
    actuator_msg.controls = [0, 0, 0, 0.1, 0.1, 0, 0, 0]  # Adjust motor speeds here

