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

        self.declare_parameter('pos1', 0.)
        self.declare_parameter('pos2', 0.)
        self.declare_parameter('pos3', 0.)

        self.pos1 = (self.get_parameter('pos1').get_parameter_value().double_value)
        self.pos2 = (self.get_parameter('pos2').get_parameter_value().double_value)
        self.pos3 = (self.get_parameter('pos3').get_parameter_value().double_value)

        # self.names = ["base_to_link1", "link1_to_link2", "link2_to_link3"]
        # self.states = []
        # for i in range(len(self.names)):
        #     self.states.append(float(0))

        # yml = read_from_yaml('kobylecki_palczuk/lab2/urdf/rpy.yaml')
        # joints = {"link1": "base", "link2": "link1", "link3": "link2"}

        # for element in yml:
        #     if yml[element]['j'] == "revolute":
        #         names.append(joints[element] + "_to_" + element)
        #         self.states.append(float(0))

        # message declarations
        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = 'odom'
        self.odom_trans.child_frame_id = 'base'
        self.joint_state = JointState()
        self.timer = self.create_timer(0.1, self.update_state)

    def update_state(self):
        try:
            # for i in range(len(self.states)):
            #     if self.states[i] > pi/2:
            #         self.going_back = True
            #     if self.states[i] < -pi/2:
            #         self.going_back = False
            #     if self.going_back:
            #         self.states[i] -= self.degree
            #     else:
            #         self.states[i] += self.degree

            # update joint_state
            now = self.get_clock().now()
            self.joint_state.header.stamp = now.to_msg()
            # self.joint_state.name = self.names
            # self.joint_state.position = self.states

            # update transform
            # (moving in a circle with radius=2) DO ZMIANY
            self.odom_trans.header.stamp = now.to_msg()

            # send the joint state and transform
            self.joint_pub.publish(self.joint_state)
            self.broadcaster.sendTransform(self.odom_trans)


            # This will adjust as needed per iteration
            # self.loop_rate.sleep()

            # if self.pos1 < (self.get_parameter("pos1").get_parameter_value().double_value):
            #     self.pos1 += 0.1
            # if self.pos2 < (self.get_parameter("pos2").get_parameter_value().double_value):
            #     self.pos2 += 0.1
            # if self.pos3 < (self.get_parameter("pos3").get_parameter_value().double_value):
            #     self.pos3 += 0.1

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
