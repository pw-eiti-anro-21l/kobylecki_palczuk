import yaml
import os
import rclpy
import numpy as numpy
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
        self.filename = filename
        self.sub = self.create_subscription(JointState, 'joint_states', self.listen, 10)

    def listen(self, msg):
        params = read_from_yaml(self.filename)
        for element in params:
            a = params[element]['a']
            d = params[element]['d']
            theta = params[element]['theta']
            alpha = params[element]['alpha']
            trans = np.eye(4)
            trans[2][3] = 0.2

    def transX(self, a) -> np.array:
        out = np.eye(4)
        out[0][3] = a
        return out

    def transZ(self, d) -> np.array:
        out = np.eye(4)
        out[2][3] = d
        return out

    def rotX(self, alpha) -> np.array:
        return np.array([[1, 0, 0, 0], [0, cos(alpha), -sin(alpha), 0], [0, sin(alpha), cos(alpha), 0], [0, 0, 0, 1]])

    def rotZ(self, theta) -> np.array:
        return np.array([[cos(theta), -sin(theta), 0, 0], [sin(theta), cos(theta), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    
def main():
    print('Hi from lab3.')


if __name__ == '__main__':
    main()