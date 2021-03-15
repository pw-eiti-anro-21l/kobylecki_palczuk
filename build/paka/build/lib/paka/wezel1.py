import rclpy
# import keyboard
from curtsies import Input
from rclpy.node import Node

# from std_msgs.msg import String
from geometry_msgs.msg import Twist


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.lin = 0
        self.ang = 0

    def set_vel(self, lin, ang):
        self.lin = lin
        self.ang = ang

    def timer_callback(self):
        with Input(keynames='curses') as input_generator:
            for e in input_generator:
                print(repr(e))
                if(repr(e) == 'p'):
                    self.set_vel(1, 0)
                elif(repr(e) == 'l'):
                    self.set_vel(-1, 0)
                elif(repr(e) == 'd'):
                    self.set_vel(0, 1)
                elif(repr(e) == 'g'):
                    self.set_vel(0, -1)
        # keyboard.on_press_key('p', self.set_vel(1, 0))
        # keyboard.on_press_key('l', self.set_vel(-1, 0))
        # keyboard.on_press_key('d', self.set_vel(0, 1))
        # keyboard.on_press_key('g', self.set_vel(0, -1))
        msg = Twist()
        msg.linear.x = self.lin
        msg.angular.z = self.ang
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()