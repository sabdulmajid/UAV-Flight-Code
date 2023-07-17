#!/usr/bin/env python

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


