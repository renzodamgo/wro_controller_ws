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
        
        # Servo positions
        self.center_position = 1500
        self.max_position = 1800
        self.min_position = 1200
        
        # Follow the gap parameters
        self.bubble_radius = 0.2  # Safety bubble radius in meters
        self.gap_threshold = 1.0  # Minimum gap width to consider
        self.angle_range = 120  # Total angle range to consider (degrees)
        
        # Timer for control loop
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Acker Lidar Controller started')
        
        # Store processed scan data
        self.processed_ranges = None
        self.best_heading = 0.0

    def send_servo_command(self, position, duration):
        msg = SetAckerServoState()
        msg.position = position
        msg.duration = duration
        self.acker_publisher.publish(msg)
        self.get_logger().info(f'Published: position={position}, duration={duration}')

    def preprocess_lidar_data(self, ranges, angle_min, angle_increment):
        """Preprocess LIDAR data to handle invalid measurements"""
        # Convert inf and nan to maximum range
        processed = np.array(ranges)
        processed[np.isinf(processed)] = 0.0
        processed[np.isnan(processed)] = 0.0
        return processed

    def find_max_gap(self, ranges, start_idx, end_idx):
        """Find the largest gap in the given range"""
        gaps = []
        gap_start = start_idx
        
        for i in range(start_idx + 1, end_idx):
            if ranges[i] > self.gap_threshold and ranges[i-1] <= self.gap_threshold:
                gap_start = i
            elif ranges[i] <= self.gap_threshold and ranges[i-1] > self.gap_threshold:
                gap_end = i
                gaps.append((gap_start, gap_end))
        
        if not gaps:
            return None, None
        
        # Find the largest gap
        largest_gap = max(gaps, key=lambda x: x[1] - x[0])
        return largest_gap

    def find_best_point(self, ranges, gap_start, gap_end):
        """Find the best point in the gap to aim for"""
        if gap_start is None or gap_end is None:
            return len(ranges) // 2  # Return center if no gap found
        
        # Find the middle of the gap
        gap_center = (gap_start + gap_end) // 2
        
        # Find the point with maximum clearance in the vicinity of the gap center
        window = 5  # Look 5 points around the center
        start_idx = max(gap_center - window, gap_start)
        end_idx = min(gap_center + window, gap_end)
        
        max_range_idx = start_idx + np.argmax(ranges[start_idx:end_idx])
        return max_range_idx

    def scan_callback(self, msg):
        try:
            # Get the number of samples in the scan
            num_samples = len(msg.ranges)
            
            # Calculate indices for the angle range we want to consider
            angle_range_rad = np.radians(self.angle_range)
            samples_per_side = int((angle_range_rad / 2) / msg.angle_increment)
            center_idx = num_samples // 2
            
            start_idx = center_idx - samples_per_side
            end_idx = center_idx + samples_per_side
            
            # Preprocess the LIDAR data
            self.processed_ranges = self.preprocess_lidar_data(
                msg.ranges[start_idx:end_idx],
                msg.angle_min,
                msg.angle_increment
            )
            
            # Find the largest gap
            gap_start, gap_end = self.find_max_gap(
                self.processed_ranges,
                0,
                len(self.processed_ranges)
            )
            
            # Find the best point to aim for
            best_point_idx = self.find_best_point(
                self.processed_ranges,
                gap_start,
                gap_end
            )
            
            # Convert to angle
            center_angle = len(self.processed_ranges) // 2
            self.best_heading = (best_point_idx - center_angle) * msg.angle_increment
            
        except Exception as e:
            self.get_logger().error(f'Error in scan_callback: {str(e)}')
            self.best_heading = 0.0

    def control_loop(self):
        try:
            if self.best_heading is None:
                return
                
            # Convert heading angle to servo position
            # Scale the heading to servo range
            angle_scale = 300  # Tuning parameter
            steering = self.best_heading * angle_scale
            
            # Convert to servo position
            position = int(self.center_position + steering)
            position = min(max(position, self.min_position), self.max_position)
            
            self.send_servo_command(position, 0.1)
            
        except Exception as e:
            self.get_logger().error(f'Error in control_loop: {str(e)}')
            # Center servo in case of error
            self.send_servo_command(self.center_position, 0.1)

def main(args=None):
    rclpy.init(args=args)
    try:
        controller = AckerLidarController()
        rclpy.spin(controller)
    except Exception as e:
        print(f'Error in main: {str(e)}')
    finally:
        # Center servo before shutdown
        try:
            controller.send_servo_command(1500, 0.2)
        except:
            pass
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
