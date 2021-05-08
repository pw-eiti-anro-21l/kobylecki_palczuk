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


class Jint(Node):
	def __init__(self):
		rclpy.init()
		super().__init__('jint')
		qos_profile = QoSProfile(depth=10)
		self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
		self.JintControlSrv = self.create_service(JintControlSrv, "interpolation_parameters", self.interpol_callback)
		self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
		self.nodeName = self.get_name()
		self.get_logger().info("{0} initiated. Beep boop beep.".format(self.nodeName))

		# pX_Y - pozycja jointa nr X w "chwili" Y

		self.declare_parameter('p1_1', 0.)
		self.declare_parameter('p2_1', 0.)
		self.declare_parameter('p3_1', 0.)
		self.p1_1 = self.get_parameter('p1_1').get_parameter_value().double_value
		self.p2_1 = self.get_parameter('p2_1').get_parameter_value().double_value
		self.p3_1 = self.get_parameter('p3_1').get_parameter_value().double_value

		self.p1_0 = self.p1_1
		self.p2_0 = self.p2_1
		self.p3_0 = self.p3_1

		self.p1_2 = 0.
		self.p2_2 = 0.
		self.p3_2 = 0.  

		self.meth = ""

		self.odom_trans = TransformStamped()
		self.odom_trans.header.frame_id = 'base'
		self.joint_state = JointState()

		pub = threading.Thread(target=self.publish_state)
		pub.start()

	def interpol(self, start, end, tstart, tend, tserv, meth): # interpolacja liniowa question mark?
		if meth == "linear":
			return ((end-start)/(tend-tstart))*(tserv-tstart)+start
		elif meth == "spline":
			pass
		else:
			return 0

	def interpol_callback(self, req, out): # tu bedzie do callbacku
		self.time = 0.
		self.success = False

		self.p1_2 = req.p1_2
		self.p2_2 = req.p2_2
		self.p3_2 = req.p3_2

		self.p1_0 = self.p1_1
		self.p2_0 = self.p2_1
		self.p3_0 = self.p3_1
		self.targ_time = req.time

		self.meth = req.meth

		thread = threading.Thread(target=self.update_state)

		out.operation = "Success!"

		return out

	def publish_state(self):
		while True:
			try:
				# update joint_state
				now = self.get_clock().now()
				self.joint_state.header.stamp = now.to_msg()
				self.joint_state.name = ["base_to_link1", "link1_to_link2", "link2_to_link3"]
				self.joint_state.position = [self.p1_1, self.p2_1, self.p3_1]

				# update transform
				self.odom_trans.header.stamp = now.to_msg()

				# send the joint state and transform
				self.joint_pub.publish(self.joint_state)
				self.broadcaster.sendTransform(self.odom_trans)

				# This will adjust as needed per iteration
				time.sleep(0.05)

			except KeyboardInterrupt:
				pass

	def update_state(self):
		while True:
			if self.p1_1 != self.p1_2:
				self.p1_1 = interpol(self.p1_0, p1_2, 0, self.targ_time, self.time, self.meth)
			if self.p2_1 != self.p2_2:
				self.p2_1 = interpol(self.p2_0, p2_2, 0, self.targ_time, self.time, self.meth)
			if self.p3_1 != self.p3_2:
				self.p3_1 = interpol(self.p3_0, p3_2, 0, self.targ_time, self.time, self.meth)


def main():
	wenzel = Jint()
	rclpy.spin(wenzel)


if __name__ == '__main__':
	main()
