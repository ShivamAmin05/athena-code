import rclpy
import pygame
from rclpy.node import Node
from umdloop_thrustmaster_messages.msg import TMJoystick

class ThrustmasterNode(Node):
    def __init__(self):
        super().__init__('thrustmaster_node')

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.publisher_ = self.create_publisher(TMJoystick, "tm_joystick", 10)

        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise Exception("No joystick connected")
        
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        self.tm = TMJoystick()

        self.epsilon = 0.01
        
        self.get_logger().info(f"Joystick: {self.joystick.get_name()} connected.")

    def timer_callback(self):
        pygame.event.pump()
        x_axis = self.joystick.get_axis(0)
        y_axis = self.joystick.get_axis(1)
        z_rotation = self.joystick.get_axis(2)
        slider = self.joystick.get_axis(3)
        self.tm.x_axis = x_axis if abs(x_axis) > self.epsilon else 0.0
        self.tm.y_axis = y_axis if abs(y_axis) > self.epsilon else 0.0
        self.tm.z_rotation = z_rotation if abs(z_rotation) > self.epsilon else 0.0
        self.tm.slider = slider if abs(slider) > self.epsilon else 0.0
        self.tm.but0 = bool(self.joystick.get_button(0))
        self.tm.but1 = bool(self.joystick.get_button(1))
        self.tm.but2 = bool(self.joystick.get_button(2))
        self.tm.but3 = bool(self.joystick.get_button(3))
        self.tm.but4 = bool(self.joystick.get_button(4))
        self.tm.but5 = bool(self.joystick.get_button(5))
        self.tm.but6 = bool(self.joystick.get_button(6))
        self.tm.but7 = bool(self.joystick.get_button(7))
        self.tm.but8 = bool(self.joystick.get_button(8))
        self.tm.but9 = bool(self.joystick.get_button(9))
        self.tm.but10 = bool(self.joystick.get_button(10))
        self.tm.but11 = bool(self.joystick.get_button(11))
        self.tm.but12 = bool(self.joystick.get_button(12))
        self.tm.but13 = bool(self.joystick.get_button(13))
        self.tm.but14 = bool(self.joystick.get_button(14))
        self.tm.but15 = bool(self.joystick.get_button(15))

        self.publisher_.publish(self.tm)

def main(args=None):
    rclpy.init(args=args)
    tm_node = ThrustmasterNode()
    rclpy.spin(tm_node)
    tm_node.destroy_node()
    pygame.joystick.quit()
    pygame.quit()
    rclpy.shutdown()


if __name__ == '__main__':
    main()