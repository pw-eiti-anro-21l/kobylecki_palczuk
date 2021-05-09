import yaml
import os
import rclpy
import numpy
import math
import threading
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster, TransformStamped
from rclpy.clock import ROSClock
from lab4_srv.srv import JintControlSrv

class Oint(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('oint')

def main():
    wenzel = Oint()
    rclpy.spin(wenzel)

if __name__ == '__main__':
    main()