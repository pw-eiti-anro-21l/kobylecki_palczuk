import rclpy
# import keyboard
from curtsies import Input
from rclpy.node import Node

# from std_msgs.msg import String
from geometry_msgs.msg import Twist


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.declare_parameter('test_parameter', 'world')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.move()
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.lin = 0
        self.ang = 0

    def set_vel(self, l, a):
        self.lin = float(l)
        self.ang = float(a)

    def move(self):
        with Input(keynames='curses') as input_generator:
            for e in input_generator:
                self.lin = float(0)
                self.ang = float(0)
                if(str(e) == 'p'):
                    self.set_vel(1, 0)
                elif(str(e) == 'l'):
                    self.set_vel(-1, 0)
                elif(str(e) == 'd'):
                    self.set_vel(0, 1)
                elif(str(e) == 'g'):
                    self.set_vel(0, -1)
                msg = Twist()
                msg.linear.x = self.lin
                msg.angular.z = self.ang
                self.publisher_.publish(msg)

    def timer_callback(self):
        my_param = self.get_parameter('test_parameter').get_parameter_value().string_value
        # self.get_logger().info('Buenos dias %s' % my_param)
        my_new_param = rclpy.parameter.Parameter('test_parameter', rclpy.Parameter.Type.STRING, 'world')
        all_new_parameters = [my_new_param]
        self.set_parameters(all_new_parameters)
        # with Input(keynames='curses') as input_generator:
        #     for e in input_generator:
        #         self.lin = float(0)
        #         self.ang = float(0)
        #         if(str(e) == 'p'):
        #             self.set_vel(1, 0)
        #         elif(str(e) == 'l'):
        #             self.set_vel(-1, 0)
        #         elif(str(e) == 'd'):
        #             self.set_vel(0, 1)
        #         elif(str(e) == 'g'):
        #             self.set_vel(0, -1)
        #         msg = Twist()
        #         msg.linear.x = self.lin
        #         msg.angular.z = self.ang
        #         self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # minimal_publisher.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()