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
        qos_profile = QoSProfile(depth=10)
        self.pose_pub = self.create_publisher(PoseStamped, 'ikin_pose', qos_profile)
        self.OintControlSrv = self.create_service(OintControlSrv, "oint_control_srv", self.interpol_callback)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} initiated. Beep boop beep.".format(self.nodeName))
        self.declare_params()

def main():
    wenzel = Ikin()
    rclpy.spin(wenzel)


if __name__ == '__main__':
    main()
