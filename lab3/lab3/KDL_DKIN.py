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
        super().__init__('KDL_DKIN')
        filename = os.path.join(get_package_share_directory('lab3'), 'DH.yaml') # 'src/kobylecki_palczuk/lab3/urdf/DH.yaml'
        # {'use_sim_time': use_sim_time, 'robot_description': Command(['xacro', ' ', os.path.join(get_package_share_directory('lab3'), xacro_file_name)])}
        self.filename = filename
        self.stamped = PoseStamped()
        self.sub = self.create_subscription(JointState, 'joint_states', self.my_pykdl, 10)
        self.pub = self.create_publisher(PoseStamped, 'pose_stamped_KDL_DKIN', 10)

    def publish_positions(self):
        self.stamped.header.stamp = self.get_clock().now().to_msg()
        self.stamped.header.frame_id = 'base'
        self.pub.publish(self.stamped)

    def my_pykdl(self, msg):
        params = read_from_yaml(self.filename)
        gengis = Chain()
        decha = [[0, 0, 0, 0],[0, 0, 0, 0], [0,0,0,0]]
        #decha = [[3, 0, 0, 0],[3, 1.57075, 0, 0],[3, 0, 0, 0],[0, 0, 0, 0]]
        for i, element in enumerate(params):
            for j, el in enumerate(params[element]):
                #if el == "d" or el == "theta":
                decha[i][j] = params[element][el]
                #else:
                    #decha[i+1][j] = params[element][el]
        print(decha)
        #decha[][]=
        fr = Frame()
        for i in range(len(decha)):
            if i != 1:
                gengis.addSegment(Segment(Joint(Joint.RotZ), fr.DH(decha[i][0], decha[i][2], decha[i][1], decha[i][3])))
            else:
                gengis.addSegment(Segment(Joint(Joint.RotY), fr.DH(decha[i][0], decha[i][2], decha[i][1], decha[i][3])))
        jnts = JntArray(3)
        jnts[0] = msg.position[0]
        jnts[1] = msg.position[1]
        jnts[2] = msg.position[2]

        solvepls = ChainFkSolverPos_recursive(gengis)
        fr1 = Frame()
        solvepls.JntToCart(jnts, fr1)
        xyz = fr1.p
        quack = fr1.M.GetQuaternion()
        self.stamped.pose.position.x = float(xyz[0])
        self.stamped.pose.position.y = float(xyz[1])
        self.stamped.pose.position.z = float(xyz[2])
        self.stamped.pose.orientation = Quaternion(x=float(quack[0]), y=float(quack[1]), z=float(quack[2]), w=float(quack[3]))
        self.publish_positions()

def main():
    wenzel = KDL_DKIN()
    rclpy.spin(wenzel)

if __name__ == '__main__':
    main()