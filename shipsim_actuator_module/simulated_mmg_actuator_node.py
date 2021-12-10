#! /usr/bin/python3
# -*- coding: utf-8 -*-

import numpy as np
import pandas as pd

import rclpy
from rclpy.node import Node

from shipsim_msgs_module.msg import MMGControl


class MmgActuatorNode(Node):
    """ActuatorNode."""

    rps = 0.0
    current_rps = 0.0
    d_current_rps = 0.0
    add_current_rps = 0.0

    rudder_angle_degree = 0.0
    current_angle = 0.0
    d_current_angle = 0.0
    add_current_angle = 0.0

    dfrps = pd.DataFrame(index=range(1),columns=["rps"])#no delay
    dfrps.fillna(0, inplace=True) 
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

        listrps = []
        current_rps = self.dfrps.iloc[-1]['rps']
        d_current_rps = self.rps - current_rps
        if abs(d_current_rps) <= 0.5:
            self.dfrps = self.dfrps.append({"rps":self.rps},ignore_index=True)
            listrps = self.dfrps["rps"].to_list()
            self.pub_actuator_msg.rps = listrps[len(listrps)-1]
        elif d_current_rps > 0.5:
            add_current_rps = current_rps + 0.5
            self.dfrps = self.dfrps.append({"rps":add_current_rps},ignore_index=True)
            listrps = self.dfrps["rps"].to_list()
            self.pub_actuator_msg.rps = listrps[len(listrps)-1]
        elif d_current_rps < -0.5:
            add_current_rps = current_rps - 0.5
            self.dfrps = self.dfrps.append({"rps":add_current_rps},ignore_index=True)
            listrps = self.dfrps["rps"].to_list()
            self.pub_actuator_msg.rps = listrps[len(listrps)-1]

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
        self.get_logger().info('MMG ActuatorNode Publishing: rps="%s", rudder_angle="%s"'% (self.pub_actuator_msg.rps, self.pub_actuator_msg.rudder_angle_degree))

    def listener_callback(self, msg):
        """listener_callback."""
        self.get_logger().info('MMG ActuatorNode heard: rps="%s", rudder_angle="%s"'% (msg.rps, msg.rudder_angle_degree))
        self.rps = msg.rps
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