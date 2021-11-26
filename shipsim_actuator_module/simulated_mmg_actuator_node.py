#! /usr/bin/python3
# -*- coding: utf-8 -*-

import numpy as np
import pandas as pd

import rclpy
from rclpy.node import Node

from shipsim_msgs_module.msg import MMGControl


class MmgActuatorNode(Node):
    """ActuatorNode."""

    rpm = 0.0
    current_rpm = 0.0
    d_current_rpm = 0.0
    add_current_rpm = 0.0

    rudder_angle_degree = 0.0
    current_angle = 0.0
    d_current_angle = 0.0
    add_current_angle = 0.0

    dfrpm = pd.DataFrame(index=range(1),columns=["rpm"])#no delay
    dfrpm.fillna(0, inplace=True) 
    dfr = pd.DataFrame(index=range(1),columns=["r"])#no delay
    dfr.fillna(0, inplace=True) 

    def __init__(self, publish_address="/ship1/actuator", timer_period=0.1):
        """init."""
        super().__init__("simulated_actuator")
        self.delta_time = timer_period
        self.pub_actuator = self.create_publisher(MMGControl, publish_address, 1)
        self.subscription = self.create_subscription(MMGControl, "/ship1/control", self.listener_callback, 1)
        self.timer = self.create_timer(timer_period, self.sender_callback)

    def sender_callback(self):
        """sender_callback."""
        self.pub_actuator_msg = MMGControl()

        listrpm = []
        current_rpm = self.dfrpm.iloc[-1]['rpm']
        d_current_rpm = self.rpm - current_rpm
        if abs(d_current_rpm) <= 0.5:
            self.dfrpm = self.dfrpm.append({"rpm":self.rpm},ignore_index=True)
            listrpm = self.dfrpm["rpm"].to_list()
            self.pub_actuator_msg.rpm = listrpm[len(listrpm)-1]
        elif d_current_rpm > 0.5:
            add_current_rpm = current_rpm + 0.5
            self.dfrpm = self.dfrpm.append({"rpm":add_current_rpm},ignore_index=True)
            listrpm = self.dfrpm["rpm"].to_list()
            self.pub_actuator_msg.rpm = listrpm[len(listrpm)-1]
        elif d_current_rpm < -0.5:
            add_current_rpm = current_rpm - 0.5
            self.dfrpm = self.dfrpm.append({"rpm":add_current_rpm},ignore_index=True)
            listrpm = self.dfrpm["rpm"].to_list()
            self.pub_actuator_msg.rpm = listrpm[len(listrpm)-1]

        listr=[]
        current_angle = self.dfr.iloc[-1]['r']
        d_current_angle = self.rudder_angle_degree - current_angle
        if abs(d_current_angle) <= 0.5:
            self.dfr = self.dfr.append({"r":self.rudder_angle_degree},ignore_index=True)
            listr = self.dfr["r"].to_list()
            self.pub_actuator_msg.rudder_angle_degree = listr[len(listr)-1]
        elif d_current_angle > 0.5:
            add_current_angle = current_angle + 0.5
            self.dfr = self.dfr.append({"r":add_current_angle},ignore_index=True)
            listr = self.dfr["r"].to_list()
            self.pub_actuator_msg.rudder_angle_degree = listr[len(listr)-1]
        elif d_current_angle < 0.5:
            add_current_angle = current_angle - 0.5
            self.dfr = self.dfr.append({"r":add_current_angle},ignore_index=True)
            listr = self.dfr["r"].to_list()
            self.pub_actuator_msg.rudder_angle_degree = listr[len(listr)-1]

        self.pub_actuator.publish(self.pub_actuator_msg)
        self.get_logger().info('MMG ActuatorNode Publishing: rpm="%s", rudder_angle="%s"'% (self.pub_actuator_msg.rpm, self.pub_actuator_msg.rudder_angle_degree))

    def listener_callback(self, msg):
        """listener_callback."""
        self.get_logger().info('MMG ActuatorNode heard: rpm="%s", rudder_angle="%s"'% (msg.rpm, msg.rudder_angle_degree))
        self.rpm = msg.rpm
        self.rudder_angle_degree = msg.rudder_angle_degree


def main(args=None):
    """Run main."""
    rclpy.init(args=args)
    node = MmgActuatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()