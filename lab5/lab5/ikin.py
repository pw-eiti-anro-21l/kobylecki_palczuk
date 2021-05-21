import os
import rclpy
import numpy
import math
import threading
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Quaternion, PoseStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from rclpy.clock import ROSClock
from lab4_srv.srv import JintControlSrv, OintControlSrv



class Ikin(Node):
	def __init__(self):
        rclpy.init()
        super().__init__('ikin')
        self.filename = os.path.join(get_package_share_directory('lab5'), 'DH.yaml')
        self.params = read_from_yaml(self.filename)

def read_from_yaml(filename):
    with open(filename, 'r') as file:
        reader = yaml.load(file, Loader=yaml.FullLoader)
    return reader

def main():
    wenzel = Ikin()
    rclpy.spin(wenzel)


if __name__ == '__main__':
    main()
