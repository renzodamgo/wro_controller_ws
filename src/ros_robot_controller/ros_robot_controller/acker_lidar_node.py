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
        
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # PID constants
        self.Kp = 0.5
        self.Ki = 0.1
        self.Kd = 0.2
        
        # PID variables
        self.previous_error = 0.0
        self.integral = 0.0
        
        # Center position for servo (typically 1500)
        self.center_position = 1500
        self.max_position = 2000
        self.min_position = 1000
        
        # Timer for control loop
        self.create_timer(0.1, self.control_loop)  # 10Hz control loop

    def scan_callback(self, msg):
        """Process LIDAR data"""
        # Get front angles (-30 to 30 degrees)
        front_start_idx = int((30 + 180) * len(msg.ranges) / 360)
        front_end_idx = int((330 + 180) * len(msg.ranges) / 360)
        
        # Get left and right distances
        left_distances = msg.ranges[front_start_idx:front_start_idx + 10]
        right_distances = msg.ranges[front_end_idx - 10:front_end_idx]
        
        # Calculate average distances, filtering out invalid readings
        left_avg = np.mean([d for d in left_distances if msg.range_min < d < msg.range_max])
        right_avg = np.mean([d for d in right_distances if msg.range_min < d < msg.range_max])
        
        # Calculate error (difference between left and right distances)
        self.current_error = left_avg - right_avg

    def pid_control(self):
        """PID control calculation"""
        if not hasattr(self, 'current_error'):
            return 0
            
        # Calculate PID terms
        proportional = self.Kp * self.current_error
        self.integral += self.Ki * self.current_error
        derivative = self.Kd * (self.current_error - self.previous_error)
        
        # Calculate control output
        output = proportional + self.integral + derivative
        
        # Update previous error
        self.previous_error = self.current_error
        
        return output

    def control_loop(self):
        """Main control loop"""
        # Calculate steering adjustment
        steering = self.pid_control()
        
        # Convert to servo position
        position = int(self.center_position + steering)
        position = min(max(position, self.min_position), self.max_position)
        
        # Create and publish message
        msg = SetAckerServoState()
        msg.position = position
        msg.duration = 0.1
        
        self.acker_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = AckerLidarController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
