import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
import math

class ReactiveFollowGap(Node):
    def __init__(self):
        super().__init__('reactive_node')
        
        # Topics & Subscriptions, Publishers
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        
        # self.drive_pub = self.create_publisher(
        #     AckermannDriveStamped,
        #     '/drive',
            # 10)
        
        # Parameters
        self.BUBBLE_RADIUS = 0.05  # meters
        self.PREPROCESS_CONV_SIZE = 3
        self.MAX_LIDAR_DIST = 2.0  # meters

    def preprocess_lidar(self, proc_ranges):
        """ Preprocess the LiDAR scan array. """
        # Remove large values
        proc_ranges[proc_ranges > self.MAX_LIDAR_DIST] = self.MAX_LIDAR_DIST
        
        proc_ranges[np.isnan(proc_ranges)] = self.MAX_LIDAR_DIST
        
        return proc_ranges

    def find_max_gap(self, range):
        """ Return the start index & end index of the max gap """
        
        return start_i, end_i

    def find_best_point(self, start_i, end_i, ranges):
        """Find the best point in the gap"""
        # Get the ranges of the gap
        gap_ranges = ranges[start_i:end_i]
        
        # Find the furthest point
        best_idx = np.argmax(gap_ranges) + start_i
        
        return best_idx

    def lidar_callback(self, data):
        ranges = np.array(data.ranges)
        total_angles = len(data.ranges)
            
            # Left side (around 60 degrees)
        # left_center = int(total_angles * (60 / 360))  # 60 degrees
        
        # # Right side (around 300 degrees)
        # right_center = int(total_angles * (300 / 360))  
        
        # start_idx = left_center
        # end_idx = right_center
        # Preprocess lidar data
        proc_ranges = self.preprocess_lidar(ranges)
        start_idx = int(len(proc_ranges) * (90/360))
        end_idx = int(len(proc_ranges) * (270/360))

        self.get_logger().info(f"angles: {proc_ranges[start_idx:end_idx]}")
        
        # Find closest point
        closest_point = np.argmin(proc_ranges)
        
        # Create the safety bubble
        bubble_radius_idx = int(self.BUBBLE_RADIUS / data.angle_increment)
        
        # Zero out ranges inside safety bubble
        proc_ranges[max(0, closest_point - bubble_radius_idx):
                   min(len(proc_ranges), closest_point + bubble_radius_idx)] = 0
        
        # Find max length gap
        start_i, end_i = self.find_max_gap(proc_ranges)
        
        # Find the best point in the gap
        best_point_idx = self.find_best_point(start_i, end_i, proc_ranges)
        
        # Calculate steering angle
        steering_angle = best_point_idx / len(proc_ranges)
        
        # Create and publish drive message
        self.get_logger().info(f"Steering angle: {steering_angle}")
        # self.get_logger().info(f"best_point_idx: {best_point_idx}")
        # self.get_logger().info(f"len: {len(proc_ranges)}")


        # self.get_logger().info(f"angle_min: {data.angle_min}")

        # self.get_logger().info(f"angle_increment: {data.angle_increment}")


def main(args=None):
    rclpy.init(args=args)
    controller = ReactiveFollowGap()
    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()