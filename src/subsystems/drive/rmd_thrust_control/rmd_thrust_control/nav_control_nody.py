import rclpy
from rclpy.node import Node
# from umdloop_thrustmaster_messages.msg import TMJoystick
# from umdloop_thrustmaster_messages.msg import Velocitycontrol
from umdloop_thrustmaster_messages.msg import MotorVelocity
from msgs.msg import CANA
from sensor_msgs import JointState
import struct

class TMControlNode(Node):
    def __init__(self):
        super().__init__('tm_control_node')

        # self.r_ctl_pub = self.create_publisher(Velocitycontrol, 'thrust_control', 10)
        self.can_pub = self.create_publisher(CANA, "/can_tx", 10)
        self.m1_pub = self.create_publisher(MotorVelocity, "motor_141", 10)
        self.m2_pub = self.create_publisher(MotorVelocity, "motor_143", 10)
        self.m3_pub = self.create_publisher(MotorVelocity, "motor_142", 10)
        self.m4_pub = self.create_publisher(MotorVelocity, "motor_149", 10)

        self.js_sub = self.create_subscription(JointState, "nav_joint_states", self.process_joint_states, 10)

        # self.vel_msg = Velocitycontrol()
        # self.timer = self.create_timer(0.01, self.add_counter)

        # self.axis_state_counter = 0
        # self.adjust_counter = 0

        self.left_node_id = 6
        self.right_node_id = 5

        # self.adjust_lock = False

        # self.curr_abs_pos = None
        self.m1_vel_msg = MotorVelocity()

    def process_joint_states(self, msg):
        pass

    def add_counter(self):
        self.axis_state_counter += 1
        self.adjust_counter += 1

    def joystick_callback(self, msg):
        y = msg.y_axis
        x = msg.x_axis * 0.17

        if self.adjust_lock and x > 0.005:
            return
        elif self.adjust_lock and x < 0.005:
            self.adjust_lock = False
        
        if msg.but1 and self.axis_state_counter > 100:
            self.enter_closed_loop()

        if msg.but4 and self.adjust_counter > 100:
            self.vel_msg.left = 0.0
            self.vel_msg.right = 0.0
            self.r_ctl_pub.publish(self.vel_msg)

        if msg.but10 and self.adjust_counter > 100:  # Adjusted from but4 → but10
            self.vel_msg.left = 0.0
            self.vel_msg.right = 0.0
            self.r_ctl_pub.publish(self.vel_msg)

        # if msg.but0:
        #     return
        

        if msg.but15 and self.adjust_counter > 100:  # Adjusted from but9 → but15
            self.set_right_abs_pos(msg)

        if msg.but9 and self.adjust_counter > 100:
            self.set_left_abs_pos(msg)

        
        if msg.but4:
            self.adjust_left_pos(msg)
            return
        
        if msg.but10:  # Adjusted from but4 → but10
            self.adjust_right_pos(msg)
            return
        
        if msg.but0:
            y *= 10

        left_speed = y
        right_speed = y

        left_speed -= x
        right_speed += x

        self.vel_msg.left = left_speed
        self.vel_msg.right = right_speed

        self.r_ctl_pub.publish(self.vel_msg)

        left_can = CANA()
        right_can = CANA()

        left_can.id = self.left_node_id << 5 | 0x0C
        right_can.id = self.right_node_id << 5 | 0x0C

        # Convert turn input (-1 to 1) into packed data
        left_can.data = list(struct.pack('<fhh', x, 0, 0))
        right_can.data = list(struct.pack('<fhh', x, 0, 0))
        # If an additional right turn position adjustment is needed:
        # right_can.data = list(struct.pack('<fhh', x + self.can_handler.rightTurnPos, 0, 0))

        # Publish CAN messages
        self.can_pub.publish(left_can)
        self.can_pub.publish(right_can)

    def set_right_abs_pos(self, msg):
        right_can = CANA()

        right_can.id = self.right_node_id << 5 | 0x19  # Command ID: 0x19 (Set_Absolute_Position)

        # Pack the position estimate as a float32
        position_estimate = 0
        right_can.data = list(struct.pack('<f', position_estimate))

        self.can_pub.publish(right_can)

        self.adjust_counter = 0

        self.adjust_lock = True


    def adjust_right_pos(self, msg):
        right_can = CANA()
        right_can.id = self.right_node_id << 5 | 0x0C
        x = msg.x_axis * 0.2
        right_can.data = list(struct.pack('<fhh', x, 0, 0))
        self.can_pub.publish(right_can)

    def set_left_abs_pos(self, msg):
        left_can = CANA()

        left_can.id = self.left_node_id << 5 | 0x19  # Command ID: 0x19 (Set_Absolute_Position)

        # Pack the position estimate as a float32
        position_estimate = 0
        left_can.data = list(struct.pack('<f', position_estimate))

        self.can_pub.publish(left_can)

        self.adjust_counter = 0

        self.adjust_lock = True


    def adjust_left_pos(self, msg):
        left_can = CANA()
        left_can.id = self.left_node_id << 5 | 0x0C
        x = msg.x_axis * 0.2
        left_can.data = list(struct.pack('<fhh', x, 0, 0))
        self.can_pub.publish(left_can)



    def enter_closed_loop(self):
        """Send command to enter CLOSED_LOOP_CONTROL mode for both left and right nodes."""
        left_can = CANA()
        right_can = CANA()

        left_can.id = self.left_node_id << 5 | 0x07
        right_can.id = self.right_node_id << 5 | 0x07

        left_can.data = list(struct.pack('<I', 8))
        right_can.data = list(struct.pack('<I', 8))

        self.can_pub.publish(left_can)
        self.can_pub.publish(right_can)

        # Enter pos control

        left_can = CANA()
        right_can = CANA()

        left_can.id = self.left_node_id << 5 | 0x0b
        right_can.id = self.right_node_id << 5 | 0x0b

        left_can.data = list(struct.pack('<II', 3, 5))  # Packing two uint32 values
        right_can.data = list(struct.pack('<II', 3, 5))

        self.can_pub.publish(left_can)
        self.can_pub.publish(right_can)

        self.axis_state_counter = 0

        # self.can_handler.send_message(self.can_handler.left_node_id << 5 | 0x07, struct.pack('<I', 8))
        # self.can_handler.send_message(self.can_handler.right_node_id << 5 | 0x07, struct.pack('<I', 8))


def main(args=None):
    rclpy.init(args=args)
    tm_control_node = TMControlNode()
    rclpy.spin(tm_control_node)
    tm_control_node.destroy_node()
    rclpy.quit()

if __name__ == '__main__':
    main()