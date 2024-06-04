import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import math
import numpy as np

from crazyflie_py import Crazyswarm

# from cflib.crazyflie.commander import Commander

TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5.0


class CF2Controller(Node):
    def __init__(self):
        super().__init__('cf2_listener')

        self.timeHelper = self.swarm.timeHelper
        self.cf = self.swarm.allcfs.crazyflies[1]
        self.cf.takeoff(targetHeight=1.0, duration=TAKEOFF_DURATION)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.2, self.update)  # 10Hz
        self.swarm = Crazyswarm()

    def send_command(self, x, y, z):
        # Assuming cf is an instance of a Crazyflie connected and set up
        pos = np.array(np.array([x,y,z]) + np.array([0,0,1.0]))
        self.cf.goTo(pos, 0, 2.0)  # Take off and hover at 1 meter

    def update(self):
        try:
            now = rclpy.time.Time()
            # Assuming tf2_ros is being used to broadcast transforms
            transform = self.tf_buffer.lookup_transform('world', 'cf_2', now)
            self.control_drone(transform)
        except Exception as e:
            self.get_logger().info('Could not transform from world to cf2_base_link: ' + str(e))

    def control_drone(self, transform: TransformStamped):
        # Here you would put the logic to control the drone
        # For simplicity, just logging the target position
        x, y, z = transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z
        self.get_logger().info(f'Moving CF2 to x: {x}, y: {y}, z: {z}')
        # Add your drone control code here, e.g., sending commands via MAVLink or Crazyflie Python API
        self.send_command(x,y,z)






def main(args=None):
    rclpy.init(args=args)
    controller = CF2Controller()
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
