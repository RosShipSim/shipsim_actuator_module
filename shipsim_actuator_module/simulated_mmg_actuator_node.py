#! /usr/bin/python3
# -*- coding: utf-8 -*-

import pandas as pd

import rclpy
from rclpy.node import Node

from shipsim_msgs_module.msg import MMGControl


class MmgActuatorNode(Node):
    """ActuatorNode."""

    n_p = 0.0
    current_n_p = 0.0
    d_current_n_p = 0.0
    add_current_n_p = 0.0

    rudder_angle_degree = 0.0
    current_angle = 0.0
    d_current_angle = 0.0
    add_current_angle = 0.0

    dfn_p = pd.DataFrame(index=range(1),columns=["n_p"])#no delay
    dfn_p.fillna(0, inplace=True) 
    dfr = pd.DataFrame(index=range(1),columns=["r"])#no delay
    dfr.fillna(0, inplace=True) 

    def __init__(self):
        """init."""
        super().__init__("actuator", namespace="ship1")
        self.declare_parameter("publish_address", "/ship1/control_input")
        self.declare_parameter("subscribe_address", "/ship1/cmd_control")
        self.declare_parameter("delta_time", 0.1)

        publish_address = (
            self.get_parameter("publish_address").get_parameter_value().string_value
        )
        self.pub_actuator = self.create_publisher(MMGControl, publish_address, 1)
        subscribe_address = (
            self.get_parameter("subscribe_address").get_parameter_value().string_value
        )
        self.subscription = self.create_subscription(MMGControl, subscribe_address, self.listener_callback, 1)
        delta_time = self.get_parameter("delta_time").value
        self.timer = self.create_timer(delta_time, self.sender_callback)

    def sender_callback(self):
        """sender_callback."""
        self.pub_actuator_msg = MMGControl()

        listn_p = []
        current_n_p = self.dfn_p.iloc[-1]['n_p']
        d_current_n_p = self.n_p - current_n_p
        if abs(d_current_n_p) <= 0.5:
            self.dfn_p = self.dfn_p.append({"n_p":self.n_p},ignore_index=True)
            listn_p = self.dfn_p["n_p"].to_list()
            self.pub_actuator_msg.n_p = listn_p[len(listn_p)-1]
        elif d_current_n_p > 0.5:
            add_current_n_p = current_n_p + 0.5
            self.dfn_p = self.dfn_p.append({"n_p":add_current_n_p},ignore_index=True)
            listn_p = self.dfn_p["n_p"].to_list()
            self.pub_actuator_msg.n_p = listn_p[len(listn_p)-1]
        elif d_current_n_p < -0.5:
            add_current_n_p = current_n_p - 0.5
            self.dfn_p = self.dfn_p.append({"n_p":add_current_n_p},ignore_index=True)
            listn_p = self.dfn_p["n_p"].to_list()
            self.pub_actuator_msg.n_p = listn_p[len(listn_p)-1]

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
        self.get_logger().info('MMG ActuatorNode Publishing: n_p="%s", rudder_angle="%s"'% (self.pub_actuator_msg.n_p, self.pub_actuator_msg.rudder_angle_degree))

    def listener_callback(self, msg):
        """listener_callback."""
        self.get_logger().info('MMG ActuatorNode heard: n_p="%s", rudder_angle="%s"'% (msg.n_p, msg.rudder_angle_degree))
        self.n_p = msg.n_p
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
