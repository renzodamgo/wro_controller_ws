import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import SetAckerServoState
from sensor_msgs.msg import LaserScan
import numpy as np

class AckerLidarController(Node):
    def __init__(self):
        super().__init__('acker_lidar_controller')
        
        # Publishers and Subscribers
        self.acker_publisher = self.create_publisher(
            SetAckerServoState,
            '/ros_robot_controller/acker_servo/set_state',
            10
        )
        # self.get_logger().info(f'Publisher created on topic: /ros_robot_controller/acker_servo/set_state')
        
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # PID constants
        self.Kp = 300
        self.Ki = 0.0
        self.Kd = 70
        
        # PID variables
        self.previous_error = 0.0
        self.integral = 0.0
        self.current_error = 0.0  # Initialize current_error
        
        # Servo positions
        self.center_position = 1500
        self.max_position = 1800
        self.min_position = 1200  # Updated to match your hardware
        
        # Timer for control loop
        self.create_timer(0.1, self.control_loop)  # 10Hz control loop
        self.get_logger().info('Acker Lidar Controller started')


    def send_servo_command(self, position, duration):
        """Helper function to send servo commands"""
        msg = SetAckerServoState()
        msg.position = position
        msg.duration = duration
        self.acker_publisher.publish(msg)
        self.get_logger().info(f'Published: position={position}, duration={duration}')

    def scan_callback(self, msg):
        """Process LIDAR data"""
        try:
            # Calculate indices for left and right sides
            # For a 360-degree scan with angle_min = 0.0 and angle_max = 2π
            total_angles = len(msg.ranges)
            
            # Left side (around 60 degrees)
            left_center = int(total_angles * (60 / 360))  # 60 degrees
            left_start = left_center - 5
            left_end = left_center + 5
            
            # Right side (around 300 degrees)
            right_center = int(total_angles * (300 / 360))  # 300 degrees
            right_start = right_center - 5
            right_end = right_center + 5
            
            # Get distances for both sides
            left_distances = msg.ranges[left_start:left_end]
            right_distances = msg.ranges[right_start:right_end]
            
            # Filter valid readings and handle NaN values
            valid_left = [d for d in left_distances if not np.isnan(d) and msg.range_min < d < msg.range_max]
            valid_right = [d for d in right_distances if not np.isnan(d) and msg.range_min < d < msg.range_max]
            
            # Debug information
            # self.get_logger().info('Left distances: {}'.format(valid_left))
            # self.get_logger().info('Right distances: {}'.format(valid_right))
            
            # Check if we have valid readings
            if len(valid_left) == 0 or len(valid_right) == 0:
                self.get_logger().warn('No valid readings')
                self.current_error = 0.0
                return
            
            # Calculate averages
            left_avg = sum(valid_left) / len(valid_left)
            right_avg = sum(valid_right) / len(valid_right)
            
            # Calculate error (positive when should turn left, negative when should turn right)
            self.current_error = right_avg - left_avg  # Reversed from previous version
            
            self.get_logger().info('Left avg: {:.2f}, Right avg: {:.2f}, Error: {:.2f}'.format(
                left_avg, right_avg, self.current_error))
            
        except Exception as e:
            self.get_logger().error('Error in scan_callback: {}'.format(str(e)))
            self.current_error = 0.0

    
    def pid_control(self):
        """PID control calculation"""

        try:
            # Calculate PID terms
            proportional = self.Kp * self.current_error
            self.integral += self.Ki * self.current_error
            derivative = self.Kd * (self.current_error - self.previous_error)
            
            # Calculate control output
            output = proportional + self.integral + derivative
            
            # Update previous error
            self.previous_error = self.current_error
            
            return output
            
        except Exception as e:
            self.get_logger().error(f'Error in pid_control: {str(e)}')
            return 0.0

    def control_loop(self):
        """Main control loop"""
        try:
            # Calculate steering adjustment
            steering = self.pid_control()
            
            # Handle NaN case
            if np.isnan(steering):
                position = self.center_position
            else:
                # Convert to servo position with bounds checking
                position = int(self.center_position + steering)
                position = min(max(position, self.min_position), self.max_position)
            
            self.send_servo_command(position, 0.1)
            
        except Exception as e:
            self.get_logger().error(f'Error in control_loop: {str(e)}')
            # Try to center servo in case of error
            try:
                position = self.center_position
                self.send_servo_command(position, 0.1)
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    try:
        controller = AckerLidarController()
        rclpy.spin(controller)
    except Exception as e:
        print(f'Error in main: {str(e)}')
    finally:
        # Ensure servo returns to center before shutdown
        try:
            msg = SetAckerServoState()
            msg.position = 1500
            msg.duration = 0.2
            controller.acker_publisher.publish(msg)
        except:
            pass
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
