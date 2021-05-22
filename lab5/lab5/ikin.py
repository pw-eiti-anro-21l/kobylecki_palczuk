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
        # self.params = read_from_yaml(self.filename)
        self.position = [0.0, 0.0, 0.0] # [x, y, z]
        self.joints = [0.0, 0.0, 0.0]
        # self.stamped = PoseStamped()
        # self.sub = self.create_subscription(PoseStamped, 'pose_stamped_Ikin', self.get_position, 10)
        self.timer = self.create_timer(0.1, self.get_position)
        self.pub = self.create_publisher(JointState, 'joint_states', 10)

    # do gory o 1, kazdy dlugi na 3
    def get_position(self):#, msg):
        # self.stamped = msg
        # [x, y, z] = self.position
        # x = msg.pose.position.x
        # y = msg.pose.position.y
        # z = msg.pose.position.z
        x = 5
        y = 5
        z = 4
        # warunek na zasieg
        if math.sqrt(math.pow(x, 2) + math.pow(y, 2) + math.pow((z - 1), 2)) <= 9:
            # znajdowanie kata obrotu podstawy
            if x == 0:
                if y >= 0:
                    alpha = math.pi / 2
                else:
                    alpha = math.pi * 3/2
            else:
                if x >= 0 and y >= 0:
                    alpha = math.atan(y/x)
                elif x < 0 and y >= 0:
                    x = -x
                    alpha = math.atan(y/x) + math.pi / 2
                elif x < 0 and y < 0:
                    alpha = math.atan(y/x) + math.pi
                elif x >= 0 and y < 0:
                    y = -y
                    alpha = math.atan(y/x) + math.pi * 3/2
            self.joints[0] = alpha
            # znajdowanie ostatnich 2 linkow/jointow
            horiz_dist = math.sqrt(math.pow(x, 2) + math.pow(y, 2)) - 3
            vert_dist = z - 1
            length = math.sqrt(math.pow(horiz_dist, 2) + math.pow(vert_dist, 2))
            # self.get_logger().info("horiz_dist = " + str(horiz_dist))
            # self.get_logger().info("vert_dist = " + str(vert_dist))
            # self.get_logger().info("length = " + str(length))
            if horiz_dist == 0:
                # self.joints[1] = 0
                # self.joints[2] = 0
                horiz_dist = 0.001
            # else:
            theta = math.acos(length / 6)
            # if theta > 1:
            #     theta = theta - math.floor(theta)
            alpha = math.atan(vert_dist/horiz_dist) - theta
            # if
            # self.get_logger().info(str(horiz_dist / 3 - math.cos(alpha)))
            # self.get_logger().info("theta = " + str(theta))
            # self.get_logger().info("alpha = " + str(alpha))
            # tu wychodzi -1.7... a moze byc od -1 do 1
            # beta = math.asin(horiz_dist / 3 - math.cos(alpha))
            beta = math.pi / 2 - alpha - 2 * theta
            psi = math.pi / 2 - beta - alpha
            self.joints[1] = alpha
            self.joints[2] = psi
            # publikowanie
            self.joint_state = JointState()

            # self.stamped.header.stamp = self.get_clock().now().to_msg()
            # self.stamped.header.frame_id = 'base'
            # self.pub.publish(self.stamped)
            try:
                # update joint_state
                now = self.get_clock().now()
                self.joint_state.header.stamp = now.to_msg()
                self.joint_state.name = ["base_to_link1", "link1_to_link2", "link2_to_link3"]
                self.joint_state.position = [self.joints[0], self.joints[1], self.joints[2]]

                # # update transform
                # self.odom_trans.header.stamp = now.to_msg()

                # send the joint state and transform
                self.pub.publish(self.joint_state)
                # self.broadcaster.sendTransform(self.odom_trans)

                # time.sleep(self.period)

            except KeyboardInterrupt:
                pass

        else:
            self.get_logger().info("To poza moimi mozliwosciami byczq!")

    # def sin_to_cos(self, sin):
    #     angle = math.asin(sin)
    #     angle = angle - (math.pi / 2)
    #     return math.cos(angle)

    # def cos_to_sin(self, cos):
    #     angle = math.acos(cos)
    #     angle = angle + (math.pi / 2)
    #     return math.sin(angle)

# def read_from_yaml(filename):
#     with open(filename, 'r') as file:
#         reader = yaml.load(file, Loader=yaml.FullLoader)
#     return reader

def main():
    wenzel = Ikin()
    rclpy.spin(wenzel)

if __name__ == '__main__':
    main()
