#! /usr/bin/env python
from math import sin, cos, pi
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
import csv
import math
import os
import random
import yaml

def read_from_yaml(filename):
    with open(filename, 'r') as file:
        reader = yaml.load(file, Loader=yaml.FullLoader)
    return reader

class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} posed".format(self.nodeName))

        self.degree = pi / 180.0
        self.loop_rate = self.create_rate(30)

        # self.names = ["base_to_link1", "link1_to_link2", "link2_to_link3"]
        # self.states = []
        # for i in range(len(self.names)):
        #     self.states.append(float(0))

        # yml = read_from_yaml('kobylecki_palczuk/lab2/urdf/rpy.yaml')
        # joints = {"link1": "base", "link2": "link1", "link3": "link2"}

        # self.declare_parameter('a1', 0.)
        # self.declare_parameter('a2', 0.)
        # self.declare_parameter('a3', 0.)

        # self.a1 = self.get_parameter('a1').get_parameter_value().double_value
        # self.a2 = self.get_parameter('a2').get_parameter_value().double_value
        # self.a3 = self.get_parameter('a3').get_parameter_value().double_value

        # for element in yml:
        #     if yml[element]['j'] == "revolute":
        #         names.append(joints[element] + "_to_" + element)
        #         self.states.append(float(0))

        # message declarations
        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = 'odom'
        self.odom_trans.child_frame_id = 'base'
        self.joint_state = JointState()
        # self.joint_state.names = names
        self.timer = self.create_timer(0.1, self.update_state)

    def update_state(self):
        try:

            # update joint_state
            now = self.get_clock().now()
            self.joint_state.header.stamp = now.to_msg()

            # update transform
            self.odom_trans.header.stamp = now.to_msg()

            # send the joint state and transform
            self.joint_pub.publish(self.joint_state)
            self.broadcaster.sendTransform(self.odom_trans)

        except KeyboardInterrupt:
            pass

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    node = StatePublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
