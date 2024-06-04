import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist interface from the geometry_msgs package
from geometry_msgs.msg import Twist

from crazyflie_interfaces.msg import  Position
from crazyflie_py import Crazyswarm



class SimplePublisher(Node):

    def __init__(self):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        super().__init__('simple_publisher')
        # create the publisher object
        # 创建发布者对象
        # in this case, the publisher will publish on /cmd_vel Topic with a queue size of 10 messages.
        # use the Twist module for /cmd_vel Topic
        # self.publisher_ = self.create_publisher(信息模型（接口）, '目标topic名', queue size )
        self.publisher_ = self.create_publisher(Twist, 'cf_position', 10)
        
        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        self.cf = self.swarm.allcfs.crazyflies[1]



        # define the timer period for 0.5 seconds
        timer_period = 0.1
        # create a timer sending two parameters:
        # - the duration between 2 callbacks (0.5 seconds)
        # - the timer function (timer_callback)
        # 创建timer self.create_timer(时间间隔, 回调函数)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # 创建回调函数
        # Here you have the callback method
        # create a Twist message
       
        # 定义massage 类型为 twist
        msg = Position()
        # define the linear x-axis velocity of /cmd_vel Topic parameter to 0.5
        E
        # define the angular z-axis velocity of /cmd_vel Topic parameter to 0.5
        msg.angular.z = 0.5
        
        # Publish the message to the Topic
        # 发布信息
        self.publisher_.publish(msg)
        # Display the message on the console
        self.get_logger().info('Publishing: "%s"' % msg)
            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    # 实例化
    simple_publisher = SimplePublisher()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(simple_publisher)
    # Explicity destroys the node
    simple_publisher.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()