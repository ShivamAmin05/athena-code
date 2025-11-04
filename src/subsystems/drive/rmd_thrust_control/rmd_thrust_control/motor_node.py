import rclpy
from rclpy.node import Node
from umdloop_thrustmaster_messages.msg import Motorvelocity
from msgs.msg import CANA
import math

class RMDMotor(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.declare_parameter('id')
        self.str_id = self.get_parameter('id').get_parameter_value().string_value
        self.hex_id = int(self.str_id, 16) & 0xFFFF
        self.subscription = self.create_subscription(Motorvelocity, 'motor_' + str(self.str_id), self.callback, 10)
        self.publisher_ = self.create_publisher(CANA, '/can_tx', 10)
        self.get_logger().info(f"Initialized motor with id: {self.str_id}. Listening on topic 'motor_{self.str_id}")
        self.rated_rps = 70
        self.gear_reduction = 36
        self.max_rps = self.rated_rps / self.gear_reduction
        self.max_100dps = 36000 * self.max_rps

        self.wheel_r = .127
        self.wheel_c = 2 * math.pi * self.wheel_r

        self.vel2dps = 36000 / self.wheel_c * self.gear_reduction * 0.005

        self.vel_arr = [0xA2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]


    def callback(self, msg):
        arr = [0] * 4
        vel = msg.velocity
        dps = vel * self.vel2dps
        dps = int(dps) if dps <= self.max_100dps else int(self.max_100dps)

        arr[0] = dps & 0xFF
        arr[1] = (dps >> 8) & 0xFF
        arr[2] = (dps >> 16) & 0xFF
        arr[3] = (dps >> 24) & 0xFF

        self.send_vel_command(arr)

    def send_vel_command(self, dps):
        can_msg = CANA()
        vel_arr = [0xA2, 0, 0, 0, dps[0], dps[1], dps[2], dps[3]]
        can_msg.id = self.hex_id
        can_msg.data = vel_arr
        self.publisher_.publish(can_msg)
        

def main(args=None):
    rclpy.init(args=args)
    motor_node = RMDMotor()
    rclpy.spin(motor_node)
    motor_node.destroy_node()
    rclpy.quit()

if __name__ == '__main__':
    main()