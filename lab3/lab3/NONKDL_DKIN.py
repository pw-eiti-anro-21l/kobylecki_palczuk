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

class NONKDL_DKIN(Node):
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
        self.pub = self.create_publisher(PoseStamped, '/pose_stamped_NONKDL_DKIN', 10)

    def create_transformations(self):
        params = read_from_yaml(self.filename)
        trans = []
        i=0
        for element in params:
            a = params[element]['a']
            d = params[element]['d']
            alpha = params[element]['alpha']
            # theta = params[element]['theta']
            theta = self.positions[i]
            i+=1
            trans.append(self.transform(a, d, alpha, theta))
        self.transformations = trans

    def publish_positions(self):
        self.calculate()
        self.stamped.header.stamp = self.get_clock().now().to_msg()
        self.stamped.header.frame_id = 'base'
        self.pub.publish(self.stamped)

    def calculate(self):
        self.create_transformations()
        print("-----")
        print(self.transformations)
        print("----------")
        matrix = self.transformations[len(self.transformations) - 1]
        print("i = ", len(self.transformations)-1)
        print(matrix)
        # print("len = ", len(self.transformations))
        for i in range(len(self.transformations)-2, -1, -1):
            # j = len(self.transformations) - i
            # if i > 0:
            matrix = numpy.matmul(self.transformations[i], matrix)
            print("i = ", i)
            print(matrix)
        narzedzie = numpy.matmul(matrix, numpy.array([[0], [0], [1], [1]]))
        self.stamped.pose.position.x = float(narzedzie[0])
        self.stamped.pose.position.y = float(narzedzie[1])
        self.stamped.pose.position.z = float(narzedzie[2])
        (r, p, y) = self.rotation_TORPY(matrix)
        self.stamped.pose.orientation = euler_to_quaternion(r, p, y)

    def rotation_TORPY(self, r):
        gamma = math.atan2(r[2][1], r[2][2])
        alpha = math.atan2(r[1][0], r[0][0])
        beta = math.atan2(-r[2][0], math.sqrt(math.pow(r[2][1], 2) + math.pow(r[2][2], 2)))
        return (gamma, beta, alpha)

    def get_positions(self, msg):
        self.positions = msg.position
        self.publish_positions()

    def makeTransMatrix(self, x, y, z):
        matr = numpy.zeros((4, 4))
        matr[0][3] = x
        matr[1][3] = y
        matr[2][3] = z
        return matr

    def rotateX(self, alpha):
        return numpy.array([[1, 0, 0, 0], [0, math.cos(alpha), -math.sin(alpha), 0], [0, math.sin(alpha), math.cos(alpha), 0], [0, 0, 0, 1]])

    # def rotateY(self, beta):
    #     return numpy.array([[math.cos(beta), 0, math.sin(beta), 0], [0, 1, 0, 0], [-math.sin(beta), 0, math.cos(beta), 0], [0, 0, 0, 1]])

    def rotateZ(self, theta):
        return numpy.array([[math.cos(theta), -math.sin(theta), 0, 0], [math.sin(theta), math.cos(theta), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    def transform(self, a, d, alpha, theta):
        alphaRot = self.rotateX(alpha)
        thetaRot = self.rotateZ(theta)
        transMatr = self.makeTransMatrix(a, 0, d)
        return numpy.matmul(alphaRot, thetaRot) + transMatr

def main():
    wenzel = NONKDL_DKIN()
    rclpy.spin(wenzel)

if __name__ == '__main__':
    main()