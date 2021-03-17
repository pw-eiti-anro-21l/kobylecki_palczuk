import rclpy
import rclpy.node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType
from curtsies import Input
from rclpy.node import Node
from geometry_msgs.msg import Twist


class KeyboardControls(Node):

    def __init__(self):
        super().__init__('keyboard_controls')
        self.declare_parameter('right', 'd')
        self.declare_parameter('left', 'a')
        self.declare_parameter('up', 'w')
        self.declare_parameter('down', 's')
        self.declare_parameter('stop', 'q')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.create_param()
        self.get_logger().info('Buenos diaaaaaas')
        self.lin = float(0)
        self.ang = float(0)
        self.move()

    def set_vel(self, l, a):
        self.lin = float(l) 
        self.ang = float(a)

    def move(self):
        counter=1
        with Input(keynames='curses') as input_generator:
            input_generator.send(0.1)
            up = self.get_parameter('up').get_parameter_value().string_value
            down = self.get_parameter('down').get_parameter_value().string_value
            left = self.get_parameter('left').get_parameter_value().string_value
            right = self.get_parameter('right').get_parameter_value().string_value
            stop = self.get_parameter('stop').get_parameter_value().string_value
            self.get_logger().info('Para los controles, use las teclas '+up+', '+down+', '+left+' y '+right+', para detener el uso de '+stop+', cualquier otra pulsación de tecla apagará el nodo.')
            while True:
                e = input_generator.send(0.1)
                self.lin = float(0)
                self.ang = float(0)
                if(str(e) == up):
                    self.set_vel(1, 0)
                elif(str(e) == down):
                    self.set_vel(-1, 0)
                elif(str(e) == left):
                    self.set_vel(0, 1)
                elif(str(e) == right):
                    self.set_vel(0, -1)
                elif(str(e) == stop):
                    self.set_vel(0, 0)
                # else:
                #     self.destroy_node()
                #     rclpy.shutdown()
                msg = Twist()
                msg.linear.x = self.lin
                msg.angular.z = self.ang
                self.publisher_.publish(msg)
                self.get_logger().info('Buenos diaaaaaas por %s vez' % counter)
                counter+=1
                

    # def create_param(self):
    #     my_param = self.get_parameter('test_parameter').get_parameter_value().string_value
    #     self.get_logger().info('Buenos dias %s' % my_param)
    #     my_new_param = rclpy.parameter.Parameter('test_parameter', rclpy.Parameter.Type.STRING, 'world')
    #     all_new_parameters = [my_new_param]
    #     self.set_parameters(all_new_parameters)

    # def timer_callback(self):
        # my_param = self.get_parameter('test_parameter').get_parameter_value().string_value
        # self.get_logger().info('Buenos dias %s' % my_param)
        # my_new_param = rclpy.parameter.Parameter('test_parameter', rclpy.Parameter.Type.STRING, 'world')
        # all_new_parameters = [my_new_param]
        # self.set_parameters(all_new_parameters)
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

    keyboard_controls = KeyboardControls()

    rclpy.spin(keyboard_controls)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # minimal_publisher.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()