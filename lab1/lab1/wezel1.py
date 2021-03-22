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
        self.lin=0
        self.ang=0
        self.up=0
        self.down=0
        self.left=0
        self.right=0
        self.stop=0
        self.declare_parameter('right', 'd')
        self.declare_parameter('left', 'a')
        self.declare_parameter('up', 'w')
        self.declare_parameter('down', 's')
        self.declare_parameter('stop', 'q')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer_period = 0.1  # seconds
        self.counter=0
        self.get_logger().info('Buenos diaaaaaas')
        self.set_vel(0, 0)
        self.setup()
        self.timer = self.create_timer(self.timer_period, self.move)

    # sets value of velocity
    def set_vel(self, l, a):
        self.lin = float(l) 
        self.ang = float(a)
        self.counter=0

    # makes initial setup for keybindings and prints message
    def setup(self):
        self.up = self.get_parameter('up').get_parameter_value().string_value
        self.down = self.get_parameter('down').get_parameter_value().string_value
        self.left = self.get_parameter('left').get_parameter_value().string_value
        self.right = self.get_parameter('right').get_parameter_value().string_value
        self.stop = self.get_parameter('stop').get_parameter_value().string_value

        self.get_logger().info('Para los controles, use las teclas "'+self.up+'", "'+self.down+'", "'+self.left+'" y "'+self.right+'", para detener el uso de "'+self.stop+'", cualquier otra pulsación de tecla apagará el nodo.')

    # makes sure that turtle is going in single direction for max 1 second
    def control(self):
        if(self.counter*self.timer_period > 1):
            self.set_vel(0, 0)
            self.counter=0

    # gathers info on keys and publishes new velocity
    def move(self):
        self.setup()
        counter=1
        with Input(keynames='curses') as input_generator:
            key = input_generator.send(0.1)
            if(str(key) == self.up):
                self.set_vel(1, 0)
            elif(str(key) == self.down):
                self.set_vel(-1, 0)
            elif(str(key) == self.left):
                self.set_vel(0, 1)
            elif(str(key) == self.right):
                self.set_vel(0, -1)
            elif(str(key) == self.stop):
                self.set_vel(0, 0)
            elif(key == None):
                self.counter+=1
            else:
                self.destroy_node()
                rclpy.shutdown()
            self.control()

            # publishing
            msg = Twist()
            msg.linear.x = self.lin
            msg.angular.z = self.ang
            self.publisher_.publish(msg)   

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
