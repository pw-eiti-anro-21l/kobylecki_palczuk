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

class NONKDL_DKIN(Node):
    def __init__(self, filename):
        super().__init__('NONKDL_DKIN')
        self.transformations = []
        self.filename = filename
        self.sub = self.create_subscription(JointState, 'joint_states', self.listen, 10)
        transform  = self.transform(a, d, alpha, theta)
        # trzeba cos dokonczyc

    def listen(self, msg):
        params = read_from_yaml(self.filename)
        self.transformations = []
        for element in params:
            a = params[element]['a']
            d = params[element]['d']
            theta = params[element]['theta']
            alpha = params[element]['alpha']

            # to jest algorytm i nwm co to ani po co to
            trans = numpy.eye(4)
            trans[2][3] = 0.2 # o co tu chodzi wg???

            self.transformations.append(self.transform(a, d, alpha, theta))

    def makeVector(self, x, y, z):
        return numpy.array([[x], [y], [z], [1]])

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
        x = self.transformX(a)
        z = self.transformZ(d)
        alpha = self.rotateX(alpha)
        theta = self.rotateZ(theta)
        vec = makeVector(x, 0, z)
        return numpy.matmul(theta, alpha) + vec

def main():
    print('Hi from lab3.')

if __name__ == '__main__':
    main()