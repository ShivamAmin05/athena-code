import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import math
from rcl_interfaces.msg import ParameterDescriptor

class AckermannDriveController:
    def __init__(self):
        # ALL METERS
        self.wheelbase = 1.0  # Distance between front and rear axles
        self.track_width = 0.8  # Distance between left and right wheels
        self.wheel_radius = 0.15  # Wheel radius in meters
        
        self.max_turn_angle = math.radians(30)  # Maximum steering angle (spec in deg)
        
    def calculate_joint_states(self, linear_vel, angular_vel):
        """
        Calculate steering angles and wheel velocities based on Ackermann geometry.
        
        Args:
            linear_vel: Desired linear velocity in m/s
            angular_vel: Desired angular velocity in rad/s
            
        Returns:
            A tuple containing:
            - bl_vel: Velocity for back left wheel (m/s)
            - fl_vel: Velocity for front left wheel (m/s)
            - fr_vel: Velocity for front right wheel (m/s)
            - br_vel: Velocity for back right wheel (m/s)
            - bl_steer: Steering angle for back left wheel (radians)
            - fl_steer: Steering angle for front left wheel (radians)
            - fr_steer: Steering angle for front right wheel (radians)
            - br_steer: Steering angle for back right wheel (radians)
            
        """

        # Zero velocity case
        if abs(linear_vel) < 0.001 and abs(angular_vel) < 0.001:
            return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            
        if abs(angular_vel) < 0.001:
            return 0.0, 0.0, linear_vel, linear_vel, linear_vel, linear_vel
            
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
        fl_angle = max(min(left_angle, self.max_turn_angle), -self.max_turn_angle)
        fr_angle = max(min(right_angle, self.max_turn_angle), -self.max_turn_angle)
        
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
        
        # Calculate linear velocities for each wheel
        bl_vel = abs(angular_vel) * bl_radius
        fl_vel = abs(angular_vel) * fl_radius
        fr_vel = abs(angular_vel) * fr_radius
        br_vel = abs(angular_vel) * br_radius
        
        # Apply direction multiplier based on desired linear velocity
        dir_multiplier = 1 if linear_vel >= 0 else -1
        bl_vel *= dir_multiplier
        fl_vel *= dir_multiplier
        fr_vel *= dir_multiplier
        br_vel *= dir_multiplier
        
        return bl_vel, fl_vel, fr_vel, br_vel, 0, fl_angle, fr_angle,0


class Twist2JointStates(Node):
    def __init__(self):
        super().__init__('twist_2_joint_states')
        
        
        self.declare_parameter('cmd_vel_topic', 'nav_cmd_vel')
        self.declare_parameter('drive_type', 'Ackermann', ParameterDescriptor(description='Drive type for control. Avaliable types are: Ackermann'))
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            self.get_parameter("cmd_vel_topic"),
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
            "BL_steer_joint", 
            "FL_steer_joint",
            "FR_steer_joint", 
            "BR_steer_joint" 
        ]
        
        self.joint_state = JointState()
        self.joint_state.name = self.wheel_joint_names + self.steering_joint_names
        self.joint_state.position = [0.0] * 8
        self.joint_state.velocity = [0.0] * 8
        self.joint_state.effort = [0.0] * 8
        
        self.timer = self.create_timer(0.02, self.publish_joint_states)
        
        # default to ackermann if nothing is specified.
        if(self.get_parameter("drive_type") == "Ackermann"):
            self.controller = AckermannDriveController()
        else:
            self.controller = AckermannDriveController()
        
        
        self.get_logger().info("Twist2JointStates initialized")
        
    def cmd_vel_callback(self, msg):
        """Handle incoming velocity commands."""
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        # calculate velocities
        bl_vel, fl_vel, fr_vel, br_vel, bl_steer, fl_steer, fr_steer, br_steer = self.controller.calculate_joint_states(
            linear_vel, angular_vel
        )
        #[vel,vel,vel,vel,pos,pos,pos,pos]
        #[BL, FL, FR, BR, BL, FL, FR, BR]
        self.joint_state.velocity[0] = bl_vel  
        self.joint_state.velocity[1] = fl_vel  
        self.joint_state.velocity[2] = fr_vel  
        self.joint_state.velocity[3] = br_vel 
        
        self.joint_state.position[4] = bl_steer
        self.joint_state.position[5] = fl_steer
        self.joint_state.position[6] = fr_steer
        self.joint_state.position[7] = br_steer
        
        self.get_logger().debug(f"Steering: BL={math.degrees(bl_steer):.1f}° FL={math.degrees(fl_steer):.1f}° FR={math.degrees(fr_steer):.1f}° BR={math.degrees(br_steer):.1f}°")
        self.get_logger().debug(f"Velocities: BL={bl_vel:.1f} FL={fl_vel:.1f} FR={fr_vel:.1f} BR={br_vel:.1f}")
    
    def publish_joint_states(self):
        """Publish current joint states."""
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_pub.publish(self.joint_state)


def main(args=None):
    rclpy.init(args=args)
    
    node = Twist2JointStates()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()