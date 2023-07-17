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
