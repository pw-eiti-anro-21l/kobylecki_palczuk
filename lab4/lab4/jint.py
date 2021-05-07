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


class Jint(Node):
	def __init__(self):
		rclpy.init()
		super().__init__('jint')
		self.joint_pub = self.create_publisher(JointState, 'joint_states')
		self.jint_control_srv = self.create_service(JintControlSrv, "interpolation_parameters", self.interpol_callback)

	def interpol_lin(self, start, end, tstart, tend, tserv): #interpolacja liniowa question mark?
		return ((end-start)/(tend-tstart))*(tserv-tstart)+start

	def interpol_trap(self): # tu bedzie interpolacja trapezowa
		pass

	def interpol_callback(self, req): # tu bedzie do callbacku
		pass


def main():
    print('Hi from lab4.')


if __name__ == '__main__':
    main()
