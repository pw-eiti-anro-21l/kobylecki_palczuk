import yaml
import os
import rclpy
import numpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster, TransformStamped
from rclpy.clock import ROSClock
from PyKDL import *

def read_from_yaml(filename):
    with open(filename, 'r') as file:
        reader = yaml.load(file, Loader=yaml.FullLoader)
    return reader

def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

class KDL_DKIN(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('NONKDL_DKIN')
        filename = os.path.join(get_package_share_directory('lab3'), 'DH.yaml') # 'src/kobylecki_palczuk/lab3/urdf/DH.yaml'
        # {'use_sim_time': use_sim_time, 'robot_description': Command(['xacro', ' ', os.path.join(get_package_share_directory('lab3'), xacro_file_name)])}
        self.transformations = []
        self.positions = [0.0, 0.0, 0.0]
        self.filename = filename
        self.stamped = PoseStamped()
        self.sub = self.create_subscription(JointState, 'joint_states', self.get_positions, 10)
        self.pub = self.create_publisher(PoseStamped, 'pose_stamped_NONKDL_DKIN', 10)
        self.publish_positions()

    def publish_positions(self):
        self.calculate()
        self.stamped.header.stamp = self.get_clock().now().to_msg()
        self.stamped.header.frame_id = 'odom'
        self.pub.publish(self.stamped)

    def my_pykdl(self):
        params = read_from_yaml(self.filename)
        gengis = Chain()
        fr = Frame()
        for element in params:
            gengis.addSegment(Joint(Joint.RotZ), fr.DH(element['a'], element['alpha'], element['d'], element['theta'])) #do zmiany wartosci w wektorze
        jnts = JntArray(3)
        for i, element in enumerate(self.positions):
            jnts[i] = element
        solvepls = ChainFkSolverPos_recursive(chain)
        solvepls.JntToCart(jnts, fr)
        xyz = fr.p
        quack = fr.M.GetQuaternion()
        self.stamped.pose.position.x = float(xyz[0])
        self.stamped.pose.position.y = float(xyz[1])
        self.stamped.pose.position.z = float(xyz[2])
        self.stamped.pose.orientation = Quaternion(x=float(quack[0]), y=float(quack[1]), z=float(quack[2]), w=float(quack[3]))

def main():
    wenzel = KDL_DKIN()
    rclpy.spin(wenzel)

if __name__ == '__main__':
    main()