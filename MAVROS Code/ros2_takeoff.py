#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Header

class QuadcopterControl(Node):
    def __init__(self):
        super().__init__('quadcopter_control')

        # Global variables
        self.current_pose = PoseStamped()
        self.current_velocity = TwistStamped()

        # Subscribe to pose and velocity topics
        self.pose_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, 10)
        self.velocity_sub = self.create_subscription(TwistStamped, '/mavros/local_position/velocity', self.velocity_callback, 10)

        # Advertise to the necessary topics
        self.arm_pub = self.create_publisher(CommandBool, '/mavros/cmd/arming', 10)
        self.mode_pub = self.create_publisher(SetMode, '/mavros/set_mode', 10)
        self.takeoff_pub = self.create_publisher(CommandTOL, '/mavros/cmd/takeoff', 10)
        self.land_pub = self.create_publisher(CommandTOL, '/mavros/cmd/land', 10)

        # Set the rate (in Hz) at which to publish messages
        self.rate = self.create_rate(20)

    def pose_callback(self, msg):
        self.current_pose = msg

    def velocity_callback(self, msg):
        self.current_velocity = msg

    def arm(self):
        arm_msg = CommandBool()
        arm_msg.value = True
        self.arm_pub.publish(arm_msg)

    def set_mode(self, mode):
        mode_msg = SetMode()
        mode_msg.custom_mode = mode
        self.mode_pub.publish(mode_msg)

    def takeoff(self, altitude):
        takeoff_msg = CommandTOL()
        takeoff_msg.altitude = altitude
        self.takeoff_pub.publish(takeoff_msg)

    def land(self):
        land_msg = CommandTOL()
        land_msg.altitude = 0
        self.land_pub.publish(land_msg)

    def run(self):
        # Wait for the connection to be established
        while self.current_pose.header.seq == 0 or self.current_velocity.header.seq == 0:
            self.rate.sleep()

        # Arm the quadcopter
        self.arm()

        # Set the mode to GUIDED
        self.set_mode('GUIDED')

        # Takeoff to a specified altitude
        self.takeoff(2.0)  # Replace with your desired takeoff altitude (in meters)

        # Wait for the quadcopter to reach the desired altitude
        while self.current_pose.pose.position.z < 1.9:  # Replace with your desired altitude minus a tolerance value
            self.rate.sleep()

        # Descend back down
        self.set_mode('LAND')

        # Wait for the quadcopter to land
        while self.current_pose.pose.position.z > 0.1:  # Replace with your desired ground clearance (in meters)
            self.rate.sleep()

        # Disarm the quadcopter
        self.set_mode('STABILIZE')
        self.arm()

def main(args=None):
    rclpy.init(args=args)
    quadcopter_control = QuadcopterControl()
    quadcopter_control.run()
    rclpy.spin(quadcopter_control)
    quadcopter_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
