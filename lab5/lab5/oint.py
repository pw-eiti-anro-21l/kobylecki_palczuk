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
from visualization_msgs.msg import Marker, MarkerArray

class Oint(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('oint')
        qos_profile = QoSProfile(depth=10)
        self.pose_pub = self.create_publisher(PoseStamped, 'pose_stamped_Ikin', qos_profile)
        self.mark_pub = self.create_publisher(MarkerArray, '/oh_hai_mark', qos_profile)
        self.markerArray = MarkerArray()
        self.marker = Marker()
        self.marker.header.frame_id = "base"
        self.marker.id = 0
        self.marker.action = Marker.DELETEALL
        self.count = 0
        self.MARKERS_MAX = 500

        self.markerArray.markers.append(self.marker)
        self.mark_pub.publish(self.markerArray)
        self.marker.type = self.marker.SPHERE
        self.marker.action = self.marker.ADD
        self.marker.scale.x = 0.03
        self.marker.scale.y = 0.03
        self.marker.scale.z = 0.03
        self.marker.color.a = 0.5
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.pose.orientation.x = 1.0
        self.marker.pose.orientation.y = 1.0
        self.marker.pose.orientation.z = 1.0
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
        self.success = False

        # okres co ile liczona jest nowa pozycja w interpolacji
        self.period = 0.05

        self.pose_stamped = PoseStamped()

        self.timer = self.create_timer(self.period, self.update_state)

        pub = threading.Thread(target=self.publish_state)
        pub.start()

    def interpol_callback(self, req, out):
        if self.success:
            # rozne wyjatki
            if self.p1_2 == req.xx and self.p2_2 == req.yy and self.p3_2 == req.zz:
                out.operation = "Juz to zrobilem byczq!"
            elif req.meth != "linear" and req.meth != "spline":
                out.operation = "Nie znam takiej interpolacji byczq!"
            elif req.time <= 0:
                out.operation = "Potrzebuje wiecej czasu byczq!"
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

                out.operation = "Sukces byczq!"
        else:
            out.operation = "Jeszcze sie ruszam, chilluj wora!"
        return out

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
                self.marker.pose.position.x = self.pose_stamped.pose.position.x
                self.marker.pose.position.y = self.pose_stamped.pose.position.y
                self.marker.pose.position.z = self.pose_stamped.pose.position.z
                if(self.count > self.MARKERS_MAX):
                        self.markerArray.markers.pop(0)
                self.markerArray.markers.append(self.marker)

                i = 0
                for mark in self.markerArray.markers:
                    mark.id = i
                    i += 1

                self.mark_pub.publish(self.markerArray)
                self.count += 1
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
