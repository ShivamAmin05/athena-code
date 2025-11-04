#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from std_msgs.msg import String
import json
from msgs.msg import CANA  # or CANB
import can
import threading
import struct
import math
import sys
import time


class EventDrivenCAN:
    def __init__(self, channel="can0", bustype="socketcan"):
        """
        Initializes the CAN interface.

        :param channel: CAN interface channel (e.g., 'can0')
        :param bustype: Type of CAN bus (e.g., 'socketcan')
        :param callback: Function pointer to handle received messages
        """
        self.bus = can.Bus(channel=channel, bustype=bustype)
        self.running = False
        self.listener_thread = None
        self.counter = 0

        self.left_node_id = 6  # TODO: Update if needed
        self.right_node_id = 5  # TODO: Update if needed

        self.leftIsClosed = False
        self.rightIsClosed = False
        self.leftTurnPos = 0.0
        self.leftVel = 0.0
        self.rightTurnPos = 0.0
        self.rightVel = 0.0

    def can_callback(self, msg):
        """Handles incoming CAN messages."""
        if not self.leftIsClosed and msg.arbitration_id == (self.left_node_id << 5 | 0x01):
            error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
            if state == 8:  # CLOSED_LOOP_CONTROL
                self.leftIsClosed = True
                

        if not self.rightIsClosed and msg.arbitration_id == (self.right_node_id << 5 | 0x01):
            error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
            if state == 8:  # CLOSED_LOOP_CONTROL
                self.rightIsClosed = True
                

        if msg.arbitration_id == (self.left_node_id << 5 | 0x09):  # Get Encoder Estimates
            self.leftTurnPos, self.leftVel = struct.unpack('<ff', bytes(msg.data))

        if msg.arbitration_id == (self.right_node_id << 5 | 0x09):  # Get Encoder Estimates
            self.rightTurnPos, self.rightVel = struct.unpack('<ff', bytes(msg.data))
        

    def _listener(self):
        """Private method to listen for incoming CAN messages."""
        
        while self.running:
            msg = self.bus.recv(timeout=1.0)  # Non-blocking wait
            if msg:
                self.can_callback(msg)  # Call the user-defined callback

    def start(self):
        """Starts the event-driven CAN listener in a separate thread."""
        if self.running:
            return  # Already running
        self.running = True
        self.listener_thread = threading.Thread(target=self._listener, daemon=True)
        self.listener_thread.start()
        print("CAN listener started.")

    def stop(self):
        """Stops the CAN listener thread."""
        self.running = False
        if self.listener_thread:
            self.listener_thread.join()
        print("CAN listener stopped.")

    def send_message(self, arbitration_id, data):
        """
        Sends a CAN message.

        :param arbitration_id: The CAN ID of the message
        :param data: A list or bytearray containing up to 8 bytes of data
        """
        message = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
        self.bus.send(message)
        #print(f"Sent CAN message: {message}")


class JankDriveSubsystem(Node):
    def __init__(self, can_handler):
        super().__init__('jank_drive_subsystem')
        self.get_logger().info('Initializing Jank Drive Subsystem')

        # ROS2 Subscribers and Publishers
        self.jank_sub = self.create_subscription(String, 'cmd_jank', self.jank_callback, 10)

        self.can_handler = can_handler

        # Start CAN listener
        self.can_handler.start()

        # Send initial CAN commands to enter CLOSED_LOOP_CONTROL
        self.enter_closed_loop()

        # Wait for both motors to enter closed-loop control
        while not (self.can_handler.leftIsClosed and self.can_handler.rightIsClosed):
            time.sleep(0.5)
            self.get_logger().info("Turn motors not ready, retrying...")
            self.enter_closed_loop()

        self.enter_pos_control()
        
        self.get_logger().info("Left motor entered CLOSED_LOOP_CONTROL")
        self.get_logger().info("Right motor entered CLOSED_LOOP_CONTROL")


    def enter_closed_loop(self):
        """Send command to enter CLOSED_LOOP_CONTROL mode for both left and right nodes."""
        self.can_handler.send_message(self.can_handler.left_node_id << 5 | 0x07, struct.pack('<I', 8))
        self.can_handler.send_message(self.can_handler.right_node_id << 5 | 0x07, struct.pack('<I', 8))

    def enter_pos_control(self):
        self.can_handler.send_message(self.can_handler.left_node_id << 5 | 0x0b, struct.pack('<II', 3, 5))
        self.can_handler.send_message(self.can_handler.right_node_id << 5 | 0x0b, struct.pack('<II', 3, 5))

    def jank_callback(self, msg):
        """Processes joystick command and sends CAN messages accordingly."""
        self.max_vel = 620 * 600  # TODO: Verify velocity scaling
        jank_dict = json.loads(msg.data)

        
        #self.get_logger().info(f"Right position: {self.can_handler.rightTurnPos:.3f} [turns], velocity: {self.can_handler.rightVel:.3f} [turns/s]")

        forward = self.apply_deadband(jank_dict['forward'], 0.05)
        backward = self.apply_deadband(jank_dict['backward'], 0.05)
        if jank_dict['isTurbo']:
            forward = math.pow(forward, 3)
        else:
            forward = .025 * math.pow(forward, 3)
        """
        if forward < -.8:
            forward = 1.215 * math.pow(forward, 7) + .215
        elif forward < 0:
            forward *= .01
        elif forward < .8:
            forward *= .01
        else:
            forward = 1.215 * math.pow(forward, 7) - .215


        if backward < -.75:
            backward = 1.301 * math.pow(backward, 5) + .301
        elif backward < 0:
            backward *= .01
        elif backward < .75:
            backward *= .01
        else:
            backward = 1.301 * math.pow(backward, 5) - .301
        """



        turn = (self.apply_deadband(jank_dict['turn'], 0.05) * 0.15)# TODO: Adjust scaling

        self.get_logger().info(f"Left position: {self.can_handler.leftTurnPos:.3f} [turns], velocity: {self.can_handler.leftVel:.3f} [turns/s], desired position: {(self.can_handler.leftTurnPos+turn):.3f}")

        drive_vel = self.max_vel * (forward + backward)

        # Send drive velocity commands
        drive_msg_left = bytearray(b'\xA2\x00\x00\x00') + int(drive_vel).to_bytes(4, byteorder='little', signed=True)
        drive_msg_right = bytearray(b'\xA2\x00\x00\x00') + int(drive_vel * -1).to_bytes(4, byteorder='little', signed=True)

        self.can_handler.send_message(0x140 + 3, drive_msg_left)
        self.can_handler.send_message(0x140 + 2, drive_msg_right)
        # drive_msg_left = bytearray(b'\xA2\x00\x00\x00') + int(drive_vel).to_bytes(4, byteorder='little', signed=True)
        # drive_msg_right = bytearray(b'\xA2\x00\x00\x00') + int(drive_vel * -1).to_bytes(4, byteorder='little', signed=True)
        self.can_handler.send_message(0x140 + 4, drive_msg_left)
        self.can_handler.send_message(0x140 + 9, drive_msg_right) # test motor
        # self.can_handler.send_message(0x140 + 5, drive_msg_right)

        """
        if jank_dict['b12']:
            turn_msg_left=struct.pack('<ff', .25, 0.0)
            turn_msg_right=struct.pack('<ff', .25, 0.0)
            self.can_handler.send_message(self.can_handler.left_node_id << 5 | 0x0D, turn_msg_left)
            self.can_handler.send_message(self.can_handler.right_node_id << 5 | 0x0D, turn_msg_right)
        if jank_dict['b11']:
            turn_msg_left=struct.pack('<ff', -.25, 0.0)
            turn_msg_right=struct.pack('<ff', -.25, 0.0)
            self.can_handler.send_message(self.can_handler.left_node_id << 5 | 0x0D, turn_msg_left)
            self.can_handler.send_message(self.can_handler.right_node_id << 5 | 0x0D, turn_msg_right)
        if jank_dict['b10']:
            turn_msg_left=struct.pack('<ff', 0.0, 0.0)
            turn_msg_right=struct.pack('<ff', 0.0, 0.0)
            self.can_handler.send_message(self.can_handler.left_node_id << 5 | 0x0D, turn_msg_left)
            self.can_handler.send_message(self.can_handler.right_node_id << 5 | 0x0D, turn_msg_right)
        """

        # Send turn position commands
        # turn_msg_left = struct.pack('<fhh', turn + self.can_handler.leftTurnPos, 0, 0)
        turn_msg_left = struct.pack('<fhh', turn, 0, 0)
        turn_msg_right = struct.pack('<fhh', turn, 0, 0)
        #turn_msg_right = struct.pack('<fhh', turn + self.can_handler.rightTurnPos, 0, 0)

        self.can_handler.send_message(self.can_handler.left_node_id << 5 | 0x0C, turn_msg_left)
        self.can_handler.send_message(self.can_handler.right_node_id << 5 | 0x0C, turn_msg_right)

    def apply_deadband(self, value, deadband):
        # return 0.0 if abs(value) < deadband else value
        return value


def main(args=None):
    rclpy.init(args=args)

    # Initialize CAN handler
    can_handler = EventDrivenCAN()  # Callback will be set in JankDriveSubsystem

    # Create the JankDriveSubsystem and set the CAN callback
    jank_drive_subsystem = JankDriveSubsystem(can_handler)
    
    try:
        rclpy.spin(jank_drive_subsystem)
    except KeyboardInterrupt:
        jank_drive_subsystem.get_logger().info("Shutting down...")

    # Cleanup
    jank_drive_subsystem.destroy_node()
    can_handler.stop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
