import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import can
import struct
import math

class JointState2Control(Node):
    def __init__(self):
        super().__init__('JointState2Control')
        
        self.can_bus = can.interface.Bus(channel='can0', bustype='socketcan')
        
        self.motor_ids = {
            'BL_wheel_joint': 0x141,  # Back Left
            'FL_wheel_joint': 0x143,  # Front Left
            'FR_wheel_joint': 0x142,  # Front Right
            'BR_wheel_joint': 0x149,  # Back Right
            
            'BL_steer_joint': 0x151,  # Back Left Steering
            'FL_steer_joint': 0x153,  # Front Left Steering
            'FR_steer_joint': 0x152,  # Front Right Steering
            'BR_steer_joint': 0x159   # Back Right Steering
        }
        
        self.rated_rps = 70
        self.gear_reduction = 36
        self.max_rps = self.rated_rps / self.gear_reduction
        self.max_100dps = 36000 * self.max_rps
        
        self.wheel_radius = 0.127  # meters
        self.wheel_circumference = 2 * math.pi * self.wheel_radius
        
        # Conversion factors
        self.vel2dps = 36000 / self.wheel_circumference * self.gear_reduction * 0.005  # m/s to 0.01 degrees/sec
        self.steering2dps = (180/math.pi) * 100  # radians to 0.01 degrees
        
        self.declare_parameter("joint_state_topic", 'nav_joint_states')
        
        self.create_subscription(
            JointState, 
            self.get_parameter("joint_state_topic").get_parameter_value().string_value,
            self.set_motor_state,
            10
        )
        
        self.initialize_steering_motors()
        
    def initialize_steering_motors(self):
        """Initialize steering motors in closed loop control mode."""
        steering_ids = [
            self.motor_ids['BL_steer_joint'],
            self.motor_ids['FL_steer_joint'],
            self.motor_ids['FR_steer_joint'],
            self.motor_ids['BR_steer_joint']
        ]
        
        for motor_id in steering_ids:
            closed_loop_msg = can.Message(
                arbitration_id=motor_id,
                data=struct.pack('<I', 8),
                is_extended_id=False
            )
            self.can_bus.send(closed_loop_msg)
            
            pos_control_msg = can.Message(
                arbitration_id=motor_id,
                data=struct.pack('<II', 3, 5),
                is_extended_id=False
            )
            self.can_bus.send(pos_control_msg)
    
    def send_velocity_command(self, motor_id, velocity):
        """Send velocity command to drive motors."""
        dps = int(min(velocity * self.vel2dps, self.max_100dps))
        
        vel_data = [0xA2, 0, 0, 0]  # Command prefix
        vel_data.extend([
            dps & 0xFF,
            (dps >> 8) & 0xFF,
            (dps >> 16) & 0xFF,
            (dps >> 24) & 0xFF
        ])
        
        can_msg = can.Message(
            arbitration_id=motor_id,
            data=vel_data,
            is_extended_id=False
        )
        self.can_bus.send(can_msg)

    def send_position_command(self, motor_id, position):
        """Send position command to steering motors."""
        pos_100deg = int(position * self.steering2dps)
        
        pos_data = [0xA4, 0, 0, 0]
        pos_data.extend([
            pos_100deg & 0xFF,
            (pos_100deg >> 8) & 0xFF,
            (pos_100deg >> 16) & 0xFF,
            (pos_100deg >> 24) & 0xFF
        ])
        
        can_msg = can.Message(
            arbitration_id=motor_id,
            data=pos_data,
            is_extended_id=False
        )
        self.can_bus.send(can_msg)
        
    def set_motor_state(self, msg):
        """Handle incoming joint state messages and send appropriate CAN commands."""
        joint_indices = {name: idx for idx, name in enumerate(msg.name)}
        
        # Process each joint
        for joint_name, motor_id in self.motor_ids.items():
            if joint_name in joint_indices:
                idx = joint_indices[joint_name]
                
                if 'wheel_joint' in joint_name:
                    velocity = msg.velocity[idx]
                    self.send_velocity_command(motor_id, velocity)
                    
                elif 'steer_joint' in joint_name:
                    position = msg.position[idx]
                    self.send_position_command(motor_id, position)

def main(args=None):
    rclpy.init(args=args)
    node = JointState2Control()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()