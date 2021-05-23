import sys
import rclpy
from rclpy.node import Node
from lab5_srv.srv import OintControlSrv

class Ocmd(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('Ocmd')
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.time = 0.0
        self.meth = ""
        self.cli = self.create_client(OintControlSrv, "oint_control_srv")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = OintControlSrv.Request()

    def send_request(self):
        self.req.xx = float(self.x)
        self.req.yy = float(self.y)
        self.req.zz = float(self.z)
        self.req.time = float(self.time)
        self.req.meth = str(self.meth)
        self.future = self.cli.call_async(self.req)

def main():
    ocmd = Ocmd()
    # aa = sys.argv[1]

    # a = float(input("A = "))
    # b = float(input("B = "))
    # z = float(input("Z = "))
    # full_time = float(input("Time = "))
    # method = input("Method = ")

    a = 8.0
    b = 8.0
    z = 5.0
    full_time = 20.0
    method = "linear"

    # jazda do pozycji poczatkowej
    ocmd.x = a/2
    ocmd.y = b/2
    ocmd.z = z
    ocmd.time = 5
    ocmd.meth = method

    ocmd.send_request()

    while rclpy.ok():
        rclpy.spin_once(ocmd)
        if ocmd.future.done():
            try:
                response = ocmd.future.result()
            except Exception as e:
                ocmd.get_logger().info(
                    'Service call failed %r' % (e,))
            break

    # pierwsze a
    ocmd.x = -a/2
    ocmd.y = b/2
    ocmd.z = z
    ocmd.time = a / (2*a + 2*b) * full_time
    ocmd.meth = method

    ocmd.send_request()

    while rclpy.ok():
        rclpy.spin_once(ocmd)
        if ocmd.future.done():
            try:
                response = ocmd.future.result()
            except Exception as e:
                ocmd.get_logger().info(
                    'Service call failed %r' % (e,))
            break

    # pierwsze b
    ocmd.x = -a/2
    ocmd.y = -b/2
    ocmd.z = z
    ocmd.time = b / (2*a + 2*b) * full_time
    ocmd.meth = method

    ocmd.send_request()

    while rclpy.ok():
        rclpy.spin_once(ocmd)
        if ocmd.future.done():
            try:
                response = ocmd.future.result()
            except Exception as e:
                ocmd.get_logger().info(
                    'Service call failed %r' % (e,))
            break

    # drugie a
    ocmd.x = a/2
    ocmd.y = -b/2
    ocmd.z = z
    ocmd.time = a / (2*a + 2*b) * full_time
    ocmd.meth = method

    ocmd.send_request()

    while rclpy.ok():
        rclpy.spin_once(ocmd)
        if ocmd.future.done():
            try:
                response = ocmd.future.result()
            except Exception as e:
                ocmd.get_logger().info(
                    'Service call failed %r' % (e,))
            break

    # drugie b
    ocmd.x = a/2
    ocmd.y = b/2
    ocmd.z = z
    ocmd.time = b / (2*a + 2*b) * full_time
    ocmd.meth = method

    ocmd.send_request()

    while rclpy.ok():
        rclpy.spin_once(ocmd)
        if ocmd.future.done():
            try:
                response = ocmd.future.result()
            except Exception as e:
                ocmd.get_logger().info(
                    'Service call failed %r' % (e,))
            break

    ocmd.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()