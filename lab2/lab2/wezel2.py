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

# def read_from_csv(filename):
#     dh=[]
#     with open(filename, 'r') as file:
#         reader = csv.reader(file)
#         for row in reader:
#             dh.append(row)
#     return dh

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

        self.declare_parameter(start1, 0.)
        self.declare_parameter(start2, 0.)
        self.declare_parameter(start3, 0.)

        # dh = read_from_csv('kobylecki_palczuk/lab2/config/DH.csv')
        # dh = dh[1:]
        # matr = [[1,0,0,0,"base"]]
        # for line in dh:
        #     matr.append(line)
        # dh = matr
        # theta_varcount = 0
        # for line in dh:
        #     if line[3] == "var":
        #         theta_varcount += 1

        # states = []
        # names = []
        # types = []
        # limits = []
        # prev_line = dh[0]
        # for line in dh:

        #     joint_type = None
        #     if line[3] == 'var':
        #         joint_type = "revolute"
        #     for index in range(len(str(line[0]))):
        #         if str(line[0])[index] == ':':
        #             joint_type = "prismatic"
        #             prismatic_lower_limit = float(line[0][:index])
        #             prismatic_upper_limit = float(line[0][index+1:])

        #     if joint_type != None:
        #         joint_name = prev_line[4] + "_to_" + line[4]
        #         names.append(joint_name)
        #         states.append(float(0))
        #         types.append(joint_type)
        #         if joint_type == "prismatic":
        #             limits.append((prismatic_lower_limit, prismatic_upper_limit))

        #     prev_line = line
        # angle=0.
        # going_back = False
        # prismatic_going_back = False

        # message declarations
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base'
        joint_state = JointState()

        try:
            while rclpy.ok():
                # for i in states:
                #     i += degree
                # for i in range(len(states)):
                    # if types[i] == "revolute":
                    #     if states[i] > pi/2:
                    #         going_back = True
                    #     if states[i] < -pi/2:
                    #         going_back = False
                    #     if going_back:
                    #         states[i] -= degree
                    #     else:
                    #         states[i] += degree
                    # elif types[i] == "prismatic":
                    #     if states[i] > limits[i][1] - limits[i][0]:
                    #         prismatic_going_back = True
                    #     if states[i] < 0:
                    #         prismatic_going_back = False
                    #     if prismatic_going_back:
                    #         states[i] -= 0.01
                    #     else:
                    #         states[i] += 0.01
                # self.get_logger().info('degree = ' + str(degree))
                rclpy.spin_once(self)

                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = ["base_to_link1", "link1_to_link2", "link2_to_link3"]
                joint_state.position = [self.start1, self.start2, self.start3]
                
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

                if self.start1 < double(self.get_parameter("start1").get_parameter_value()):
                    self.start1 += 0.1
                if self.start2 < double(self.get_parameter("start2").get_parameter_value()):
                    self.start2 += 0.1
                if self.start3 < double(self.get_parameter("start3").get_parameter_value()):
                    self.start3 += 0.1

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



# class StatePublisher(Node):

#     def __init__(self):
#         rclpy.init()
#         super().__init__('state_publisher')

#         qos_profile = QoSProfile(depth=10)
#         self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
#         self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
#         self.nodeName = self.get_name()
#         self.get_logger().info("{0} started".format(self.nodeName))

#         degree = pi / 180.0
#         self.loop_rate = self.create_rate(30)

#         # robot state parameters
#         self.declare_parameter('poz1', 0.0)
#         self.declare_parameter('poz2', 0.0)
#         self.declare_parameter('poz3', 0.0)

#         self.poz1 = self.get_parameter('poz1').get_parameter_value().double_value
#         self.poz2 = self.get_parameter('poz2').get_parameter_value().double_value
#         self.poz3 = self.get_parameter('poz3').get_parameter_value().double_value

#         # message declarations
#         self.odom_trans = TransformStamped()
#         self.odom_trans.header.frame_id = 'odom'
#         self.odom_trans.child_frame_id = 'baza'
#         self.joint_state = JointState()

#         self.timer = self.create_timer(0.1, self.update_state)

#     def update_state(self):

#         try:
#             # update joint_state
#             now = self.get_clock().now()
#             self.joint_state.header.stamp = now.to_msg()
#             self.joint_state.name = ["baza_do_ramie1", "ramie1_do_ramie2", "ramie2_do_ramie3"]
#             self.joint_state.position = [self.poz1, self.poz2, self.poz3]

#             self.odom_trans.header.stamp = now.to_msg()

#             # send the joint state and transform
#             self.joint_pub.publish(self.joint_state)
#             self.broadcaster.sendTransform(self.odom_trans)

#             # This will adjust as needed per iteration
#             self.loop_rate.sleep()

#             # change params
#             if self.poz1 < self.get_parameter('poz1').get_parameter_value().double_value:
#                 self.poz1 += 0.05
#             if self.poz1 >= self.get_parameter('poz1').get_parameter_value().double_value:
#                 self.poz1 -= 0.05

#             if self.poz2 < self.get_parameter('poz2').get_parameter_value().double_value:
#                 self.poz2 += 0.05
#             if self.poz2 >= self.get_parameter('poz2').get_parameter_value().double_value:
#                 self.poz2 -= 0.05

#             if self.poz3 < self.get_parameter('poz3').get_parameter_value().double_value:
#                 self.poz3 += 0.01
#             if self.poz3 >= self.get_parameter('poz3').get_parameter_value().double_value:
#                 self.poz3 -= 0.01

#         except KeyboardInterrupt:
#             pass


# def euler_to_quaternion(roll, pitch, yaw):
#     qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
#     qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2)
#     qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2)
#     qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
#     return Quaternion(x=qx, y=qy, z=qz, w=qw)


# def main():
#     node = StatePublisher()
#     rclpy.spin(node)


# if __name__ == '__main__':
#     main()