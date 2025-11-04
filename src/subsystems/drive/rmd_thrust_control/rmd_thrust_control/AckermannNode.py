import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import math
import time

class AckermannDriveController:
    def __init__(self):
        # ALL METERS
        self.wheelbase = 1.0  # Distance between front and rear axles
        self.track_width = 0.8  # Distance between left and right wheels
        self.wheel_radius = 0.15  # Wheel radius in meters
        
        self.max_turn_angle = math.radians(30)  # Maximum steering angle (spec in deg)
        

        ## ALL of calculate ackermann steering is AI generated...may (probably) won't work
    def calculate_ackermann_steering(self, linear_vel, angular_vel):
        """
        Calculate steering angles and wheel velocities based on Ackermann geometry.
        
        Args:
            linear_vel: Desired linear velocity in m/s
            angular_vel: Desired angular velocity in rad/s
            
        Returns:
            A tuple containing:
            - left_front_angle: Steering angle for left front wheel (radians)
            - right_front_angle: Steering angle for right front wheel (radians)
            - bl_vel: Velocity for back left wheel (m/s)
            - fl_vel: Velocity for front left wheel (m/s)
            - fr_vel: Velocity for front right wheel (m/s)
            - br_vel: Velocity for back right wheel (m/s)
        """
        # Zero velocity case
        if abs(linear_vel) < 0.001 and abs(angular_vel) < 0.001:
            return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            
        # Straight motion case
        if abs(angular_vel) < 0.001:
            # Convert linear velocity to wheel angular velocity (rad/s)
            wheel_angular_vel = linear_vel / self.wheel_radius
            return 0.0, 0.0, wheel_angular_vel, wheel_angular_vel, wheel_angular_vel, wheel_angular_vel
            
        # Calculate turn radius from the center of the rear axle
        turn_radius = abs(linear_vel / angular_vel)
        
        # Determine if we're turning left or right
        turning_left = angular_vel > 0
        
        # Calculate the position of the instant center of rotation (ICR)
        icr_x = -turn_radius if turning_left else turn_radius
        
        # Calculate steering angles
        inner_angle = math.atan(self.wheelbase / (abs(icr_x) - self.track_width/2))
        outer_angle = math.atan(self.wheelbase / (abs(icr_x) + self.track_width/2))
        
        # Set angles based on turning direction
        if turning_left:
            left_angle = inner_angle
            right_angle = outer_angle
        else:
            left_angle = -outer_angle
            right_angle = -inner_angle
            
        # Clamp steering angles to maximum
        left_angle = max(min(left_angle, self.max_turn_angle), -self.max_turn_angle)
        right_angle = max(min(right_angle, self.max_turn_angle), -self.max_turn_angle)
        
        # Calculate wheel velocities
        # Distance from each wheel to the ICR
        if turning_left:
            fl_radius = math.sqrt((abs(icr_x) - self.track_width/2)**2 + self.wheelbase**2)
            fr_radius = math.sqrt((abs(icr_x) + self.track_width/2)**2 + self.wheelbase**2)
            bl_radius = abs(icr_x) - self.track_width/2
            br_radius = abs(icr_x) + self.track_width/2
        else:
            fl_radius = math.sqrt((abs(icr_x) + self.track_width/2)**2 + self.wheelbase**2)
            fr_radius = math.sqrt((abs(icr_x) - self.track_width/2)**2 + self.wheelbase**2)
            bl_radius = abs(icr_x) + self.track_width/2
            br_radius = abs(icr_x) - self.track_width/2
        
        # linear velocity v = radius * angular velocity
        bl_vel = abs(angular_vel) * bl_radius
        fl_vel = abs(angular_vel) * fl_radius
        fr_vel = abs(angular_vel) * fr_radius
        br_vel = abs(angular_vel) * br_radius
        
        dir_multiplier = 1 if linear_vel >= 0 else -1
        bl_vel *= dir_multiplier
        fl_vel *= dir_multiplier
        fr_vel *= dir_multiplier
        br_vel *= dir_multiplier
        
        return left_angle, right_angle, bl_angular_vel, fl_angular_vel, fr_angular_vel, br_angular_vel


class AckermannDriveNode(Node):
    def __init__(self):
        super().__init__('ackermann_drive_node')
        
        self.controller = AckermannDriveController()
        
        # takes in a twist message
        # TODO: make this take somethign in from twist mux rather than hardcode
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'nav_cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.joint_pub = self.create_publisher(
            JointState,
            'nav_joint_states',
            10
        )
        
        # the joints that give us translation
        self.wheel_joint_names = [
            "BL_wheel_joint", 
            "FL_wheel_joint",
            "FR_wheel_joint", 
            "BR_wheel_joint" 
        ]
        
        self.steering_joint_names = [
            "FL_steering_joint",
            "FR_steering_joint" 
        ]
        
        self.joint_state = JointState()
        self.joint_state.name = self.wheel_joint_names + self.steering_joint_names
        self.joint_state.position = [0.0] * 6 
        self.joint_state.velocity = [0.0] * 6
        self.joint_state.effort = [0.0] * 6
        
        self.timer = self.create_timer(0.02, self.publish_joint_states)
        
        self.get_logger().info("Ackermann drive controller initialized")
        
    def cmd_vel_callback(self, msg):
        """Handle incoming velocity commands."""
        # we can't control anything else
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        # calculate velocities
        left_angle, right_angle, bl_vel, fl_vel, fr_vel, br_vel = self.controller.calculate_ackermann_steering(
            linear_vel, angular_vel
        )
        
        # Update joint state message with new values
        # Positions for wheels are continuous rotation
        # Steering joints (FL, FR) positions are updated with steering angles
        self.joint_state.velocity[0] = bl_vel  # BL wheel
        self.joint_state.velocity[1] = fl_vel  # FL wheel
        self.joint_state.velocity[2] = fr_vel  # FR wheel
        self.joint_state.velocity[3] = br_vel  # BR wheel
        
        # Update steering positions
        self.joint_state.position[4] = left_angle   # FL steering
        self.joint_state.position[5] = right_angle  # FR steering
        
        self.get_logger().debug(f"Steering: L={math.degrees(left_angle):.1f}° R={math.degrees(right_angle):.1f}°")
        self.get_logger().debug(f"Velocities: BL={bl_vel:.1f} FL={fl_vel:.1f} FR={fr_vel:.1f} BR={br_vel:.1f}")
    
    def publish_joint_states(self):
        """Publish current joint states."""
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_pub.publish(self.joint_state)


def main(args=None):
    rclpy.init(args=args)
    
    node = AckermannDriveNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()