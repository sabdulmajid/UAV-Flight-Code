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
