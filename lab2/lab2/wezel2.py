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
        # tilt = 0.
        # tinc = degree
        # swivel = 0.
        # angle = 0.
        # height = 0.
        # hinc = 0.
        basedr=0.
        drtr=0.
        trczw=0.
        angle=0.

        # message declarations
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'pierwszy'
        joint_state = JointState()

        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = ['pierwszy_to_drugi', 'drugi_to_trzeci', 'trzeci_to_czwarty'] # DO ZMIANY, może już nie?
                joint_state.position = [basedr, drtr, trczw] # DO ZMIANY, to nie wiem co to jest xd

                # update transform
                # (moving in a circle with radius=2) DO ZMIANY
                odom_trans.header.stamp = now.to_msg()
                odom_trans.transform.translation.x = float(0) #cos(angle)*2
                odom_trans.transform.translation.y = float(0) # sin(angle)*2
                odom_trans.transform.translation.z = float(0)
                odom_trans.transform.rotation = \
                    euler_to_quaternion(0, 0, angle + pi/2) # roll,pitch,yaw

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

def normalize_angle(angle):
    while angle > pi * 2 or angle < -pi * 2:
        if angle > pi * 2:
            angle -= pi * 2
        elif angle < -pi * 2:
            angle += pi * 2
    return angle

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
def create_urdf(filename_out, rob_name, filename_in):
    dh = read_from_csv(filename_in)
    dh = dh[1:]
    orix = 0
    oriy = 0
    oriz = 0
    with open(filename_out, 'w+') as file:
        file.write(f'<robot name="{str(rob_name)}">\n\n')
        for i in range(len(dh)):
            name = dh[i][4]

            # link
            file.write(f'<link name="{name}">\n')

            
            # inertia

            # file.write(f'  <inertial>\n')
            # file.write(f'    <mass value="1"/>\n')
            # file.write(f'    <inertia ixx="100" ixy="0" ixz="0" iyy="100" iyz="0" izz="100" />\n')
            # file.write(f'    <origin/>\n')
            # file.write(f'  </inertial>\n\n')

            # visual

            orix += float(dh[i][1])/2
            oriz += float(dh[i][0])/2

            length = math.sqrt(math.pow(float(dh[i][0]), 2) + math.pow(float(dh[i][1]), 2))

            file.write('  <visual>\n')
            file.write(f'    <origin xyz="{orix} {oriy} {oriz}" rpy="0 {dh[i][2]} {0}" />\n') # {float(dh[i][0])/2} {float(dh[i][1])/2} # TRZEBA ZMIENIĆ RPY
            file.write('    <geometry>\n')
            file.write(f'      <cylinder radius="{length}" length="{length}" />\n') # random.random()
            file.write('    </geometry>\n')
            # zmiana kolorów, ale nie wiem czemu to NIE DZIAŁA, cociaż urdf robi się chyba dobry
            materials = {"white": "1 1 1 1", "magenta": "1 0 1 1"}
            if i%2==0:
                material="white"
            else:
                material="magenta"
            file.write(f'    <material name="{material}">\n')
            file.write(f'      <color rgba="{materials[material]}" />\n')
            file.write('    </material>\n')
            file.write('  </visual>\n')

            orix += float(dh[i][1])/2
            oriz += float(dh[i][0])/2

            # collision

            # file.write('  <collision>\n')
            # file.write(f'    <origin xyz="{0} 0 {0}" rpy="0 {1.57} {0}" />\n')
            # file.write('    <geometry>\n')
            # file.write(f'      <cylinder radius="0.01" length="{length}" />\n')
            # file.write('    </geometry>\n')
            # file.write('    <contact_coefficients mu="0" kp="1000.0" kd="1.0"/>\n')
            # file.write('  </collision>\n')

            file.write('</link>\n\n')

            # joint
            if i >= 1:
                prev_name = dh[i-1][4]
                joint_name = prev_name + "_to_" + name
                types = {'0': "fixed", "var": "revolute"}
                file.write(f'<joint name="{joint_name}" type="{types[str(dh[i][3])]}">\n')
                file.write(f'  <origin xyz="{orix - float(dh[i][1])} 0 {oriz - float(dh[i][0])}" />\n') # CHYBA ŹLE? ALE NWM, dla przypadku statycznego to chyba nawet ok
                file.write(f'  <parent link="{prev_name}"/>\n')
                file.write(f'  <child link="{name}"/>\n')
                if types[dh[i][3]]=="revolute": # TO PONIŻEJ CHYBA NIE POWINNY BYĆ STAŁE WARTOŚCI?
                    file.write(f'    <axis xyz="0 1 0" />\n')
                    file.write(f'    <limit upper="0" lower="-0.5" effort="10" velocity="10" />\n')
                file.write('</joint>\n\n')

        file.write('</robot>\n')

def main():
    create_urdf("kobylecki_palczuk/lab2/urdf/test_bogson.urdf.xml", \
        "test_bogson", "kobylecki_palczuk/lab2/config/DH.csv")
    node = StatePublisher()

if __name__ == '__main__':
    main()