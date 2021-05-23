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
from lab5_srv.srv import OintControlSrv

class Oint(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('oint')
        qos_profile = QoSProfile(depth=10)
        self.pose_pub = self.create_publisher(PoseStamped, 'pose_stamped_Ikin', qos_profile)
        self.OintControlSrv = self.create_service(OintControlSrv, "oint_control_srv", self.interpol_callback)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} initiated. Beep boop beep.".format(self.nodeName))
        self.declare_params()

    def declare_params(self):
        # pX_Y - pozycja jointa nr X w "chwili" Y
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
        self.success = True
        # self.general_success = False

        # okres co ile liczona jest nowa pozycja w interpolacji
        self.period = 0.05

        self.pose_stamped = PoseStamped()

        self.timer = self.create_timer(self.period, self.update_state)

        pub = threading.Thread(target=self.publish_state)
        pub.start()

    def interpol_callback(self, req, out):
        # if self.general_success:
        # rozne wyjatki
        # if self.p1_2 == req.xx and self.p2_2 == req.yy and self.p3_2 == req.zz:
        #     out.operation = "Juz to zrobilem byczq!"
        prop = req.aa / req.bb
        angle = math.atan2(req.aa, req.bb)
        x = math.sin(angle * 3)
        y = math.cos(angle * 3)
        if req.aa <= 0 and req.bb <= 0 or req.zz > math.sqrt(36 - 9):
            out.operation = "Nie siegne tam byczq!"
        # elif math.sqrt(math.pow((x - req.aa/2), 2) + math.pow((y - req.bb/2), 2) + math.pow((req.zz - 1), 2)) > 6:
        #     out.operation = "Nie siegne tam byczq!"
        elif req.zz < 1:
            out.operation = "Tam jest podłoga byczq!"
        elif req.meth != "linear" and req.meth != "spline":
            out.operation = "Nie znam takiej interpolacji byczq!"
        elif req.shape != "rectangle" and req.shape != "ellipse":
            out.operation = "Nie znam takiego kształtu byczq!"
        elif req.time <= 0:
            out.operation = "Potrzebuje wiecej czasu byczq!"
        else:
            # prostokąt
            a = req.aa
            b = req.bb
            if req.shape == "rectangle":
                # podążanie do pozycji startowej
                self.draw(a/2, b/2, req.zz, 3, req.meth)
                # rysowanie prostokąta
                while self.success == False:
                    time.sleep(0.1)
                op_time = a / (2*a + 2*b) * req.time
                self.draw(-a/2, b/2, req.zz, op_time, req.meth)

                while self.success == False:
                    time.sleep(0.1)
                op_time = b / (2*a + 2*b) * req.time
                self.draw(-a/2, -b/2, req.zz, op_time, req.meth)

                while self.success == False:
                    time.sleep(0.1)
                op_time = a / (2*a + 2*b) * req.time
                self.draw(a/2, -b/2, req.zz, op_time, req.meth)

                while self.success == False:
                    time.sleep(0.1)
                op_time = b / (2*a + 2*b) * req.time
                self.draw(a/2, b/2, req.zz, op_time, req.meth)

                out.operation = "Sukces byczq!"

            # out.operation = "Sukces byczq!"
        # else:
        #     out.operation = "Jeszcze sie ruszam, chilluj wora!"
        return out
    
    def draw(self, x, y, z, op_time, meth):
        self.time = 0.
        self.success = False

        self.p1_0 = self.p1_2
        self.p2_0 = self.p2_2
        self.p3_0 = self.p3_2

        self.p1_2 = x
        self.p2_2 = y
        self.p3_2 = z

        self.targ_time = op_time

        self.meth = meth
        # time.sleep(op_time)

    def update_state(self):
        if self.time < self.targ_time:
            # time increment
            self.time = self.time + self.period
            # position
            self.p1_1 = self.interpol(self.p1_0, self.p1_2, 0, self.targ_time, self.time, self.meth)
            self.p2_1 = self.interpol(self.p2_0, self.p2_2, 0, self.targ_time, self.time, self.meth)
            self.p3_1 = self.interpol(self.p3_0, self.p3_2, 0, self.targ_time, self.time, self.meth)
        else:
            self.success = True

    def interpol(self, poz_start, poz_end, t_start, t_end, t_now, meth):
        if meth == "linear":
            return ((poz_end-poz_start)/(t_end-t_start))*(t_now-t_start)+poz_start
        elif meth == "spline":
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
                # update pose_stamped
                now = self.get_clock().now()
                self.pose_stamped.header.stamp = now.to_msg()
                self.pose_stamped.header.frame_id = 'base'
                self.pose_stamped.pose.position.x = float(self.p1_1)
                self.pose_stamped.pose.position.y = float(self.p2_1)
                self.pose_stamped.pose.position.z = float(self.p3_1)

                self.pose_pub.publish(self.pose_stamped)

                time.sleep(self.period)

            except KeyboardInterrupt:
                pass

def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    wenzel = Oint()
    rclpy.spin(wenzel)

if __name__ == '__main__':
    main()
