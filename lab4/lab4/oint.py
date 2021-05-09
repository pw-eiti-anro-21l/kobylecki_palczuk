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

class Oint(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('oint')
        qos_profile = QoSProfile(depth=10)
        self.pose_pub = self.create_publisher(PoseStamped, 'pose_2137', qos_profile)
        self.OintControlSrv = self.create_service(OintControlSrv, "oint_control_srv", self.interpol_callback)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile) # potrzeba?
        self.nodeName = self.get_name()
        self.get_logger().info("{0} initiated. Beep boop beep.".format(self.nodeName))
        self.declare_params()

    def declare_params(self):
        # pX_Y - pozycja jointa nr X w "chwili" Y

        # self.declare_parameter('p1_1', 0.)
        # self.declare_parameter('p2_1', 0.)
        # self.declare_parameter('p3_1', 0.)
        # self.p1_1 = self.get_parameter('p1_1').get_parameter_value().double_value
        # self.p2_1 = self.get_parameter('p2_1').get_parameter_value().double_value
        # self.p3_1 = self.get_parameter('p3_1').get_parameter_value().double_value
        self.p1_1 = 0.
        self.p2_1 = 0.
        self.p3_1 = 0.

        self.p1_0 = 0.
        self.p2_0 = 0.
        self.p3_0 = 0.

        self.p1_2 = 0.
        self.p2_2 = 0.
        self.p3_2 = 0.  

        self.meth = ""
        self.time = 0
        self.targ_time = 0
        self.success = False

        # okres co ile liczona jest nowa pozycja w interpolacji
        self.period = 0.05

        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = 'base'
        self.pose_stamped = PoseStamped()

        self.timer = self.create_timer(self.period, self.update_state)

        pub = threading.Thread(target=self.publish_state)
        pub.start()

    def interpol_callback(self, req, out): # tu bedzie do callbacku
        if self.success:
            # granica = math.pi / 2
            # rozne wyjatki
            if self.p1_2 == req.xx and self.p2_2 == req.yy and self.p3_2 == req.zz:
                out.operation = "Juz to zrobilem byczq!"
            elif req.meth != "linear" and req.meth != "spline":
                out.operation = "Nie znam takiej interpolacji byczq!"
            elif req.time <= 0:
                out.operation = "Potrzebuje wiecej czasu byczq!"
            # elif abs(req.p1) > granica or abs(req.p2) > granica or abs(req.p3) > granica:
            #     out.operation = "To poza moimi mozliwosciami byczq!"
            else:
                self.time = 0.
                self.success = False

                self.p1_0 = self.p1_2
                self.p2_0 = self.p2_2
                self.p3_0 = self.p3_2

                self.p1_2 = req.xx
                self.p2_2 = req.yy
                self.p3_2 = req.zz

                self.targ_time = req.time

                self.meth = req.meth

                # thread = threading.Thread(target=self.update_state)

                out.operation = "Sukces byczq!"
        else:
            out.operation = "Jeszcze sie ruszam, chilluj wora!"
        return out

    def update_state(self):
        # while True:
        if self.time < self.targ_time:
            self.time = self.time + self.period
        # if self.p1_1 != self.p1_2:
            self.p1_1 = self.interpol(self.p1_0, self.p1_2, 0, self.targ_time, self.time, self.meth)
        # if self.p2_1 != self.p2_2:
            self.p2_1 = self.interpol(self.p2_0, self.p2_2, 0, self.targ_time, self.time, self.meth)
        # if self.p3_1 != self.p3_2:
            self.p3_1 = self.interpol(self.p3_0, self.p3_2, 0, self.targ_time, self.time, self.meth)
        else:
            self.success = True

    def interpol(self, poz_start, poz_end, t_start, t_end, t_now, meth): # interpolacja liniowa question mark?
        if meth == "linear":
            return ((poz_end-poz_start)/(t_end-t_start))*(t_now-t_start)+poz_start

        elif meth == "spline":
            # t_bufor = 1
            # poz_bufor = math.pi/4
            # is_max_speed = True
            # # dopasowanie jak ma nie osiagac pelnej predkosci, bo za malo czasu albo miejsca
            # if t_end-t_start < 2*t_bufor or poz_end-poz_start < 2* poz_bufor:
            # 	t_bufor = (t_end-t_start)/2
            # 	poz_bufor = (poz_end-poz_start)/2
            # 	is_max_speed = False

            # if is_max_speed:
            # 	der = (poz_end-poz_start-2*poz_bufor)/(t_end-t_start-2*t_bufor)
            # else:
            # 	der = 1

            # # rozpedzanie
            # if t_now <= t_bufor:
            # 	pass
            # # hamowanie
            # elif t_now >= t_end-t_bufor:
            # 	pass
            # # pelna predkosc (zachodzi tylko przypadku gdy max_speed nie jest osiagane)
            # else:
            # 	poz_start = poz_start + poz_bufor
            # 	poz_end = poz_end - poz_bufor
            # 	t_start = t_start + t_bufor
            # 	t_end = t_end - t_bufor
            # 	# t_now = t_now + t_bufor
            # 	return ((poz_end-poz_start)/(t_end-t_start))*(t_now-t_start)+poz_start
            k1=0
            k2=0
            a = k1*(t_end-t_start) - (poz_end-poz_start)
            b = -k2*(t_end-t_start) + (poz_end-poz_start)
            t = (t_now-t_start)/(t_end-t_start)
            return (1-t)*poz_start + t*poz_end + t*(1-t)*((1-t)*a + t*b)

        else:
            return 0

    def publish_state(self):
        while True:
            try:
                # update joint_state
                now = self.get_clock().now()
                self.pose_stamped.header.stamp = now.to_msg()
                # self.pose_stamped.name = ["base_to_link1", "link1_to_link2", "link2_to_link3"]
                self.pose_stamped.pose.position.x = self.p1_1
                self.pose_stamped.pose.position.y = self.p2_1
                self.pose_stamped.pose.position.z = self.p3_1

                # update transform
                self.odom_trans.header.stamp = now.to_msg()

                # send the joint state and transform
                self.pose_pub.publish(self.pose_stamped)
                # self.broadcaster.sendTransform(self.odom_trans)

                # This will adjust as needed per iteration
                time.sleep(self.period)

            except KeyboardInterrupt:
                pass

def main():
    wenzel = Oint()
    rclpy.spin(wenzel)

if __name__ == '__main__':
    main()