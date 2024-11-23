import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import SetAckerServoState
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class ColorDetectorNode(Node):
  def __init__(self):
    super().__init__('color_detector_node')
    self.image_subcription = self.create_subscription(
        Image,
        '/image_raw',
        self.image_callback,
        10
    )

    self.bridge = CvBridge()
        
        # Initialize parameters for red color detection
    self.lower_red = np.array([0, 100, 100])
    self.upper_red = np.array([10, 255, 255])
    
    # Second range for red (hue wraps around in HSV)
    self.lower_red2 = np.array([160, 100, 100])
    self.upper_red2 = np.array([180, 255, 255])

  def image_callback(self,msg):
      cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
      hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
      mask = cv2.inRange(hsv, self.lower_red, self.upper_red)
      contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, 
                cv2.CHAIN_APPROX_SIMPLE)
            
      for contour in contours:
          if cv2.contourArea(contour) > 500:
              M = cv2.moments(contour)
              if M["m00"] != 0:
                  cx = int(M["m10"] / M["m00"])
                  cy = int(M["m01"] / M["m00"])
                  self.get_logger().info(f'({cx}, {cy})')


def main():
    rclpy.init()
    node = ColorDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
