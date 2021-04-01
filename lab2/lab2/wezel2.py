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

    # robot state
    tilt = 0.
    tinc = degree
    swivel = 0.
    angle = 0.
    height = 0.
    hinc = 0.005

    # message declarations
    odom_trans = TransformStamped()
    odom_trans.header.frame_id = 'odom'
    odom_trans.child_frame_id = 'axis'
    joint_state = JointState()

    try:
        while rclpy.ok():
            rclpy.spin_once(self)

            # update joint_state
            now = self.get_clock().now()
            joint_state.header.stamp = now.to_msg()
            joint_state.name = ['swivel', 'tilt', 'periscope']
            joint_state.position = [swivel, tilt, height]

            # update transform
            # (moving in a circle with radius=2)
            odom_trans.header.stamp = now.to_msg()
            odom_trans.transform.translation.x = cos(angle)*2
            odom_trans.transform.translation.y = sin(angle)*2
            odom_trans.transform.translation.z = 0.7
            odom_trans.transform.rotation = \
                euler_to_quaternion(0, 0, angle + pi/2) # roll,pitch,yaw

            # send the joint state and transform
            self.joint_pub.publish(joint_state)
            self.broadcaster.sendTransform(odom_trans)

            # Create new robot state
            tilt += tinc
            if tilt < -0.5 or tilt > 0.0:
                tinc *= -1
            height += hinc
            if height > 0.2 or height < 0.0:
                hinc *= -1
            swivel += degree
            angle += degree/4

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

def normalize_angle(angle):
    while angle > pi * 2 or angle < -pi * 2:
        if angle > pi * 2:
            angle -= pi * 2
        elif angle < -pi * 2:
            angle += pi * 2
    return angle

def denavit_hartenberg_row_to_euler(previous_roll, previous_pitch, previous_yaw, matrix_row):
    alpha = matrix_row[1]
    theta = matrix_row[3]

    roll = previous_roll + alpha
    yaw = previous_yaw + theta

    roll = normalize_angle(roll)
    yaw = normalize_angle(yaw)
    # magic happens
    return (roll, pitch, yaw)

def denavit_hartenberg_to_euler(matrix):
    rpys = []
    alpha = matrix[0][1]
    theta = matrix[0][3]
    rpys[0] = (alpha, 0, theta)
    first_exec = True
    for row in matrix:
        if first_exec:
            first_exec = False
        else:
            rpys.append(denavit_hartenberg_row_to_euler(row)) # trzeba dodać żeby funckja wiersza macierzy dostawała wszystko czego potrzebuje
    return rpys

# takes filename and path of csv file and outputs DH matrix as table
def read_from_csv(filename):
    dh=[]
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            dh.append(row)
    return dh

# a, d, alpha, theta

# writes to a given location a urdf.xml file with robot data
def create_urdf(filename, name, dh):
    with open(filename, 'w') as file:
        file.write(f'<robot name="{name}">\n\n')
        size=len(dh)
        for i in range(size):

            # link

            if i==4:
                file.write(f'link name="Link{i} ubezpieczenia"\n')
            else:
                file.write(f'link name="{i}"\n')
            
            # inertia

            # file.write(f'  <inertial>\n')
            # file.write(f'    <mass value="1"/>\n')
            # file.write(f'    <inertia ixx="100" ixy="0" ixz="0" iyy="100" iyz="0" izz="100" />\n')
            # file.write(f'    <origin/>\n')
            # file.write(f'  </inertial>\n\n')

            # visual

            length = math.pow(dh[i][0], 2) + math.pow(dh[i][1], 2)
            length = math.sqrt(length)

            file.write(f'  <visual>\n')
            file.write(f'    <origin xyz="{dh[i][0]/2} 0 {dh[i][1]/2}" rpy="{1.57} 0 {0}" />\n')
            file.write(f'    <geometry>\n')
            file.write(f'      <cylinder radius="0.01" length="{length}" />\n')
            file.write(f'    </geometry>\n')
            file.write(f'    <material name="magenta">\n')
            file.write(f'      <color rgba="1 0 1 1" />\n')
            file.write(f'    </material>\n')
            file.write(f'  </visual>\n\n')

            # collision

            file.write(f'  <collision>\n')
            file.write(f'    <origin xyz="{0} 0 {0}" rpy="{1.57} 0 {0}" />\n')
            file.write(f'    <geometry>\n')
            file.write(f'      <cylinder radius="0.01" length="{length}" />\n')
            file.write(f'    </geometry>\n')
            file.write(f'    <contact_coefficients mu="0" kp="1000.0" kd="1.0"/>\n')
            file.write(f'  </collision>\n')

            file.write(f'</link>\n\n')

            # joint

            file.write(f'<joint name="{leg1connect}" type="{fixed}">\n')
            file.write(f'  <origin xyz="{0} 0 {0}" />\n')
            file.write(f'  <parent link="{axis}"/>\n')
            file.write(f'  <child link="{leg1}"/>\n')
            file.write(f'</joint>\n\n')

        file.write(f'</robot>\n')

def main():
    node = StatePublisher()

if __name__ == '__main__':
    main()