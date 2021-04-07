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

def read_from_csv(filename):
    dh=[]
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            dh.append(row)
    return dh

class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        degree = pi / 180.0
        loop_rate = self.create_rate(30)

        dh = read_from_csv('kobylecki_palczuk/lab2/config/DH.csv')
        dh = dh[1:]
        matr = [[1,0,0,0,"base"]]
        for line in dh:
            matr.append(line)
        dh = matr
        # theta_varcount = 0
        # for line in dh:
        #     if line[3] == "var":
        #         theta_varcount += 1

        # robot state
        # tilt = 0.
        # tinc = degree
        # swivel = 0.
        # angle = 0.
        # height = 0.
        # hinc = 0.
        states = []
        names = []
        prev_line = dh[0]
        for line in dh:
            if line[3] == 'var':
                joint_name = prev_line[4] + "_to_" + line[4]
                names.append(joint_name)
                states.append(float(0))
            prev_line = line
        # basedr=0.
        # drtr=0.
        # trczw=0.
        angle=0.
        going_back = False

        # message declarations
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base'
        joint_state = JointState()

        try:
            while rclpy.ok():
                # for i in states:
                #     i += degree
                for i in range(len(states)):
                    if states[i] > pi/2:
                        going_back = True
                    if states[i] < -pi/2:
                        going_back = False
                    if going_back:
                        states[i] -= degree
                    else:
                        states[i] += degree
                # self.get_logger().info('degree = ' + str(degree))
                rclpy.spin_once(self)

                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = names # DO ZMIANY, może już nie?
                joint_state.position = states # DO ZMIANY, to nie wiem co to jest xd
                
                # platynowy debugger
                # for i in names:
                #     self.get_logger().info(str(i))
                # for j in states:
                #     self.get_logger().info(str(j))
                # self.get_logger().info('')

                # update transform
                # (moving in a circle with radius=2) DO ZMIANY
                odom_trans.header.stamp = now.to_msg()
                odom_trans.transform.translation.x = 0. #cos(angle)*2
                odom_trans.transform.translation.y = 0. # sin(angle)*2
                odom_trans.transform.translation.z = 0.
                odom_trans.transform.rotation = euler_to_quaternion(0, 0, 0) # roll,pitch,yaw   0, 0, angle + pi/2

                # send the joint state and transform
                self.joint_pub.publish(joint_state)
                self.broadcaster.sendTransform(odom_trans)

                # Create new robot state
                # tilt += tinc
                # if tilt < -0.5 or tilt > 0.0:
                #     tinc *= -1
                # height += hinc
                # if height > 0.2 or height < 0.0:
                #     hinc *= -1
                # swivel += degree
                # angle += degree/4

                # This will adjust as needed per iteration
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    # create_urdf("src/kobylecki_palczuk/lab2/urdf/bogson.urdf.xml", \
    #     "test_bogson", "src/kobylecki_palczuk/lab2/config/DH.csv")
    node = StatePublisher()

if __name__ == '__main__':
    main()