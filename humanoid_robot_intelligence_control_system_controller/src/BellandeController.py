#!/usr/bin/env python3

# Copyright (C) 2024 Bellande Robotics Sensors Research Innovation Center, Ronaldson Bellande
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import os
import time
from typing import Tuple

ros_version = os.getenv("ROS_VERSION")

if ros_version == "1":
    import rospy
    from std_msgs.msg import Float64
elif ros_version == "2":
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64
else:
    raise ValueError("Invalid ROS version. Set ROS_VERSION environment variable to '1' or '2'.")

class BellandeController:
    def __init__(self, gains: Tuple[float, float, float], name: str, output_limits: Tuple[float, float] = (-float('inf'), float('inf'))):
        self.kp, self.ki, self.kd = gains
        self.name = name
        self.output_limits = output_limits
        self.reset()

        if ros_version == "1":
            rospy.init_node('bellande_controller', anonymous=True)
            self.subscriber = rospy.Subscriber('control_output', Float64, self.callback)
        elif ros_version == "2":
            rclpy.init()
            self.node = Node('bellande_controller')
            self.subscriber = self.node.create_subscription(Float64, 'control_output', self.callback, 10)

    def reset(self):
        self.last_error = 0
        self.integral = 0
        self.last_time = time.time()

    def update_config(self, gains: Tuple[float, float, float]):
        self.kp, self.ki, self.kd = gains

    def callback(self, msg):
        print(f"Received control output: {msg.data}")

    def run(self):
        if ros_version == "1":
            rospy.spin()
        elif ros_version == "2":
            rclpy.spin(self.node)

    def shutdown(self):
        if ros_version == "2":
            self.node.destroy_node()
            rclpy.shutdown()
