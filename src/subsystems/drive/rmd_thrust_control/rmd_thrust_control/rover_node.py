import rclpy
from rclpy.node import Node
from msgs.msg import CANA
from umdloop_thrustmaster_messages.msg import Velocitycontrol, Motorvelocity

class Rover(Node):
    def __init__(self):
        super().__init__('rover_node')

        self.ctl_sub = self.create_subscription(Velocitycontrol, 'thrust_control', self.ctl_callback, 10)
        self.can = self.create_publisher(CANA, '/can_tx', 10)

        self.m1 = self.create_publisher(Motorvelocity, 'motor_141', 10)
        self.m2 = self.create_publisher(Motorvelocity, 'motor_143', 10)
        self.m3 = self.create_publisher(Motorvelocity, 'motor_142', 10)
        self.m4 = self.create_publisher(Motorvelocity, 'motor_149', 10)

    def ctl_callback(self, msg):
        left = msg.left
        right = msg.right * -1
        m1 = Motorvelocity()
        m2 = Motorvelocity()
        m3 = Motorvelocity()
        m4 = Motorvelocity()

        m1.velocity = left
        m2.velocity = left

        m3.velocity = right
        m4.velocity = right

        self.m1.publish(m1)
        self.m2.publish(m2)
        self.m3.publish(m3)
        self.m4.publish(m4)


def main(args=None):
    rclpy.init(args=args)
    rover_node = Rover()
    rclpy.spin(rover_node)
    rover_node.destroy_node()
    rclpy.quit()

if __name__ == '__main__':
    main()
