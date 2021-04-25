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

# chyba niepotrzebne? narazie nie używamy
def transformacja():
    yml = read_from_yaml('kobylecki_palczuk/lab2/urdf/rpy.yaml')

    for element in yml:
        a = yml[element]['a']
        alpha = yml[element]['alpha']
        theta = yml[element]['theta']
        x = x + a

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

class NONKDL_DKIN(Node):
    def __init__(self, filename):
        super().__init__('NONKDL_DKIN')
        self.transformations = []
        self.positions = []
        self.filename = filename
        self.stamped = PoseStamped()
        self.sub = self.create_subscription(JointState, 'joint_states', self.get_positions, 10)
        self.pub = self.create_publisher(PoseStamped, 'pose_stamped_NONKDL_DKIN', 10)
        

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

            # to jest algorytm i nwm co to ani po co to
            # trans = numpy.eye(4)
            # trans[2][3] = 0.2 # o co tu chodzi wg???

            trans.append(self.transform(a, d, alpha, theta))
        self.transformations = trans

    def publish_positions(self):
        self.calculate()
        self.stamped.header.stamp = self.get_clock().now().to_msg()
        self.stamped.header.frame_id = 'odom'
        self.pub.publish(self.stamped)

    def calculate(self):
        self.create_transformations()
        matrix = self.transformations[len(self.transformations)]
        for i in range(len(self.transformations)):
            j = len(self.transformations) - i
            if j >= 0:
                matrix = numpy.matmul(self.transformations[j-1], matrix)
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

    # zły pomysł chyba
    def makeVector(self, x, y, z):
        return numpy.array([[x], [y], [z], [1]])

    def makeTransMatrix(self, x, y, z):
        # return numpy.array([[0, 0, 0, x], [0, 0, 0, y], [0, 0, 0, z], [0, 0, 0, 1]])
        matr = numpy.zeros((4, 4))
        matr[0][3] = x
        matr[1][3] = y
        matr[2][3] = z
        return matr

    # te też chyba są złym pomysłem i niepotrzebne
    def transformX(self, a):
        out = numpy.eye(4)
        out[0][3] = a
        return out

    def transformY(self, y):
        out = numpy.eye(4)
        out[1][3] = y
        return out

    def transformZ(self, d):
        out = numpy.eye(4)
        out[2][3] = d
        return out

    def rotateX(self, alpha):
        return numpy.array([[1, 0, 0, 0], [0, math.cos(alpha), -math.sin(alpha), 0], [0, math.sin(alpha), math.cos(alpha), 0], [0, 0, 0, 1]])

    def rotateY(self, beta):
        return numpy.array([[math.cos(beta), 0, math.sin(beta), 0], [0, 1, 0, 0], [-math.sin(beta), 0, math.cos(beta), 0], [0, 0, 0, 1]])

    def rotateZ(self, theta):
        return numpy.array([[math.cos(theta), -math.sin(theta), 0, 0], [math.sin(theta), math.cos(theta), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    def transform(self, a, d, alpha, theta):
        # x = self.transformX(a)
        # z = self.transformZ(d)
        alpha = self.rotateX(alpha)
        theta = self.rotateZ(theta)
        matr = self.makeTransMatrix(a, 0, d)
        return numpy.matmul(alpha, theta) + matr

def main():
    wenzel = NONKDL_DKIN('kobylecki_palczuk/lab3/urdf/DH.yaml')
    rclpy.spin(wenzel)
    print('Hi from lab3.')

if __name__ == '__main__':
    main()