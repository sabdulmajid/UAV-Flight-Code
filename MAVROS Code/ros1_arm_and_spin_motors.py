#!/usr/bin/env python3

import rospy
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import ActuatorControl

def arm_and_spin_motors():
    rospy.init_node('arm_and_spin_motors', anonymous=True)

    arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    mode = "STABILIZE"

    rospy.set_param('/mavros/battery/failsafe_action', 1)  # Set battery failsafe to LAND
    rospy.set_param('/mavros/setpoint_velocity/throttle_enable', False)  # Disable throttle failsafe

    # Arm the vehicle
    arming_result = arming_client(True)

    while not rospy.is_shutdown():
        if arming_result.success:
            rospy.loginfo("Copter armed successfully!")
            break
        rospy.loginfo("Arming failed. Retrying...")
        rospy.sleep(1)

    # Set the mode to STABILIZE
    set_mode_client(base_mode=0, custom_mode=mode)

    rospy.sleep(1)

    # Spin the motors
    actuator_pub = rospy.Publisher('/mavros/setpoint_raw/actuator_control', ActuatorControl, queue_size=10)
    actuator_msg = ActuatorControl()
    actuator_msg.header.stamp = rospy.Time.now()
    actuator_msg.group_mix = 0
    actuator_msg.controls = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]  # Maximum throttle to spin motors

    rate = rospy.Rate(10)  # Publish at 10Hz

    for _ in range(50):  # Publish for 5 seconds
        actuator_pub.publish(actuator_msg)
        rate.sleep()

    rospy.loginfo("Motors spinning!")

    rospy.spin()

if __name__ == '__main__':
    try:
        arm_and_spin_motors()
    except rospy.ROSInterruptException:
        pass
