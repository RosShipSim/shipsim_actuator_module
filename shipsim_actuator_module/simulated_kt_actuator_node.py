#! /usr/bin/python3
# -*- coding: utf-8 -*-

import numpy as np
import pandas as pd

import rclpy
from rclpy.node import Node

from shipsim_msgs_module.msg import KTControl


class KtActuatorNode(Node):
    """ActuatorNode."""

    u = 0.0
    rudder_angle_degree = 0.0
    current_angle = 0.0
    d_current_angle = 0.0
    add_current_angle = 0.0

    dfu = pd.DataFrame(index=range(30), columns=["u"])  # 3sec delay
    dfu.fillna(0, inplace=True)
    dfr = pd.DataFrame(index=range(30), columns=["r"])  # 3sec delay
    dfr.fillna(0, inplace=True)

    def __init__(self):
        """init."""
        super().__init__("actuator", namespace="ship1")
        self.declare_parameter("subscribe_address", "/ship1/control_input")
        self.declare_parameter("publish_address", "/ship1/cmd_control")
        self.declare_parameter("delta_time", 0.1)

        publish_address = (
            self.get_parameter("publish_address").get_parameter_value().string_value
        )
        self.publisher = self.create_publisher(KTControl, publish_address, 1)
        subscribe_address = (
            self.get_parameter("subscribe_address").get_parameter_value().string_value
        )
        self.subscription = self.create_subscription(
            KTControl, subscribe_address, self.listener_callback, 1
        )
        delta_time = self.get_parameter("delta_time").value
        self.timer = self.create_timer(delta_time, self.sender_callback)

    def sender_callback(self):
        """sender_callback."""
        self.pub_actuator_msg = KTControl()
        # self.pub_actuator_msg.ua = self.u #no delay
        # self.pub_actuator_msg.rudder_angle_degreea = self.rudder_angle_degree #no delay
        listu = []
        self.dfu = self.dfu.append({"u": self.u}, ignore_index=True)
        listu = self.dfu["u"].to_list()
        self.pub_actuator_msg.u = listu[len(listu) - 30]  # 3sec delay

        listr = []
        current_angle = self.dfr.iloc[-1]["r"]
        d_current_angle = self.rudder_angle_degree - current_angle
        if abs(d_current_angle) <= 0.5:
            self.dfr = self.dfr.append(
                {"r": self.rudder_angle_degree}, ignore_index=True
            )
            listr = self.dfr["r"].to_list()
            self.pub_actuator_msg.rudder_angle_degree = listr[
                len(listr) - 30
            ]  # 3sec delay
        elif d_current_angle > 0.5:
            add_current_angle = current_angle + 0.5
            self.dfr = self.dfr.append({"r": add_current_angle}, ignore_index=True)
            listr = self.dfr["r"].to_list()
            self.pub_actuator_msg.rudder_angle_degree = listr[
                len(listr) - 30
            ]  # 3sec delay
        elif d_current_angle < -0.5:
            add_current_angle = current_angle - 0.5
            self.dfr = self.dfr.append({"r": add_current_angle}, ignore_index=True)
            listr = self.dfr["r"].to_list()
            self.pub_actuator_msg.rudder_angle_degree = listr[
                len(listr) - 30
            ]  # 3sec delay

        self.publisher.publish(self.pub_actuator_msg)
        self.get_logger().info(
            'KT ActuatorNode Publishing: u="%s", rudder_angle="%s"'
            % (self.pub_actuator_msg.u, self.pub_actuator_msg.rudder_angle_degree)
        )

    def listener_callback(self, msg):
        """listener_callback."""
        self.get_logger().info(
            'KT ActuatorNode heard: u="%s", rudder_angle="%s"'
            % (msg.u, msg.rudder_angle_degree)
        )
        self.u = msg.u
        self.rudder_angle_degree = msg.rudder_angle_degree


def main(args=None):
    """Run main."""
    rclpy.init(args=args)
    node = KtActuatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
