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
        self.bubble_radius = 0.1  # Safety bubble radius in meters
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

    def find_max_gap(self, processed_ranges):
    # Step 1: Initialize variables to track the gap
        max_gap_size = 0
        max_gap_start = 0
        current_gap_size = 0
        current_gap_start = 0
        
        # Step 2: Find the nearest point
        if len(processed_ranges) > 0:
            min_dist = min(processed_ranges)
            min_idx = processed_ranges.index(min_dist)
            
            # Step 3: Create safety bubble around nearest point
            safety_radius = 0.5  # radius in meters
            for i in range(len(processed_ranges)):
                if abs(i - min_idx) <= safety_radius:
                    processed_ranges[i] = 0
        
        # Step 4: Find maximum length sequence of non-zeros
        for i in range(len(processed_ranges)):
            if processed_ranges[i] > 0:
                if current_gap_size == 0:
                    current_gap_start = i
                current_gap_size += 1
            else:
                if current_gap_size > max_gap_size:
                    max_gap_size = current_gap_size
                    max_gap_start = current_gap_start
                current_gap_size = 0
        
        # Check final gap
        if current_gap_size > max_gap_size:
            max_gap_size = current_gap_size
            max_gap_start = current_gap_start
        
        # Return the center index of the maximum gap
        if max_gap_size > 0:
            return max_gap_start + max_gap_size // 2
        return -1

    def scan_callback(self, msg):
        try:
            total_angles = len(msg.ranges)
            
            # Left side (around 60 degrees)
            left_center = int(total_angles * (60 / 360))  # 60 degrees
            
            # Right side (around 300 degrees)
            right_center = int(total_angles * (300 / 360))  
            
            start_idx = left_center
            end_idx = right_center

            self.processed_ranges = [d for d in msg.ranges[start_idx:end_idx] if not np.isnan(d) and msg.range_min < d < msg.range_max]
            
            idx_max_gap = self.find_max_gap(self.processed_ranges)
            
            # Calculate heading from -300 to 300
            center_idx = len(self.processed_ranges) // 2
            if idx_max_gap == -1:
                self.best_heading = 1500
            elif idx_max_gap < center_idx:
                self.best_heading = 1300
            elif idx_max_gap > center_idx:
                self.best_heading = 1700

            self.get_logger().info(f'Best heading: {self.best_heading}')
            
        except Exception as e:
            self.get_logger().error(f'Error in scan_callback: {str(e)}')
            self.best_heading = 0.0

    def control_loop(self):
        try:
            if self.best_heading is None:
                return
                
            steering = self.best_heading
            
            # Convert to servo position
            # Subtract steering to correct direction (right is positive, left is negative)
            # position = int(self.center_position - steering)
            position = steering
            position = min(max(position, self.min_position), self.max_position)
            
            self.get_logger().info(f'Steering: {steering}, Position: {position}')
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
