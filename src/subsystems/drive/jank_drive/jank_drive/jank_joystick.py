#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
import json


class JankJoystickNode(Node):
    def __init__(self):
        super().__init__('jank_joystick_node')
        self.get_logger().info('Initializing Jank Joystick Node')

        # Initialize Pygame and the joystick
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No joystick found!")
            rclpy.shutdown()
            return
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        # Create a Jank publisher
        self.jank_pub = self.create_publisher(
            String,
            'cmd_jank',
            10
        )

        # Create a timer to publish joystick data at a fixed rate
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

    def timer_callback(self):
        """
        Timer callback function to read and publish joystick data.
        """
        pygame.event.pump()  # Update Pygame's event queue
        # Create a Twist message
        """
        jank_msg = String(data=json.dumps({
            'forward': -(self.joystick.get_axis(5) + 1) * .5,
            'backward': (self.joystick.get_axis(2) + 1) * .5,
            'turn': self.joystick.get_axis(0)
        }))

        """
        jank_msg = String(data=json.dumps({
            'forward': self.joystick.get_axis(1),
            'backward': 0,
            'turn': self.joystick.get_axis(2),
            'isTurbo': self.joystick.get_button(0),
        }))

        #self.get_logger().info(f'x speed: {twist_msg.linear.x} | y speed: {twist_msg.linear.y} | rotation : {twist_msg.angular.z}')

        # Publish the Jank message
        self.jank_pub.publish(jank_msg)

def main(args=None):
    rclpy.init(args=args)
    jank_joystick_node = JankJoystickNode()
    rclpy.spin(jank_joystick_node)
    jank_joystick_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()