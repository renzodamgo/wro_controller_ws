import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorPoseEstimator(Node):
    def __init__(self):
        super().__init__('pose_estimator_rpi')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()

        # Camera intrinsics (approx.)
        self.camera_matrix = np.array([[800, 0, 320],
                                       [0, 800, 240],
                                       [0,   0,   1]], dtype=np.float64)
        self.dist_coeffs = np.zeros((4, 1))

        # Marker size (10 cm × 6 cm)
        w, h = 10, 6
        self.obj_points = np.array([
            [-w/2, -h/2, 0],
            [ w/2, -h/2, 0],
            [ w/2,  h/2, 0],
            [-w/2,  h/2, 0]
        ], dtype=np.float64)

        # Color bounds in HSV
        self.color_bounds = self.define_color_bounds()

    def define_color_bounds(self):
        def rgb_to_hsv(rgb):
            color = np.uint8([[rgb]])
            hsv = cv2.cvtColor(color, cv2.COLOR_RGB2HSV)
            return hsv[0][0]

        colors_rgb = {
            "Red": (238, 39, 55),
            "Green": (68, 214, 44),
            "Magenta": (255, 0, 255),
        }

        bounds = {}
        for name, rgb in colors_rgb.items():
            h = rgb_to_hsv(rgb)[0]
            bounds[name] = (np.array([max(h - 10, 0), 100, 100]), np.array([min(h + 10, 179), 255, 255]))
        return bounds

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        for name, (lower, upper) in self.color_bounds.items():
            mask = cv2.inRange(hsv, lower, upper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            best_cnt = max(contours, key=cv2.contourArea, default=None)

            if best_cnt is not None and cv2.contourArea(best_cnt) > 1000:
                rect = cv2.minAreaRect(best_cnt)
                box = cv2.boxPoints(rect)
                box = box.astype(np.float64)

                # Sort corners consistently
                center = np.mean(box, axis=0)
                angles = np.arctan2(box[:,1] - center[1], box[:,0] - center[0])
                box = box[np.argsort(angles)]
                box = np.roll(box, -np.argmin(np.sum(box, axis=1)), axis=0)

                image_points = box.copy()

                success, rvec, tvec = cv2.solvePnP(self.obj_points, image_points, self.camera_matrix, self.dist_coeffs)
                if success:
                    box_np = box.astype(np.int32)
                    cv2.drawContours(frame, [box_np], 0, (0, 255, 255), 2)

                    # Draw axes
                    axis_len = 5
                    axis_3D = np.float32([
                        [0, 0, 0],
                        [axis_len, 0, 0],
                        [0, axis_len, 0],
                        [0, 0, -axis_len]
                    ])
                    imgpts, _ = cv2.projectPoints(axis_3D, rvec, tvec, self.camera_matrix, self.dist_coeffs)
                    origin = tuple(imgpts[0].ravel().astype(int))
                    cv2.line(frame, origin, tuple(imgpts[1].ravel().astype(int)), (0, 0, 255), 2)
                    cv2.line(frame, origin, tuple(imgpts[2].ravel().astype(int)), (0, 255, 0), 2)
                    cv2.line(frame, origin, tuple(imgpts[3].ravel().astype(int)), (255, 0, 0), 2)

                    distance_cm = np.linalg.norm(tvec)
                    cv2.putText(frame, f"Dist: {distance_cm:.1f} cm", (origin[0] + 10, origin[1] + 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                    cv2.putText(frame, name, (origin[0] + 10, origin[1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    self.get_logger().info(f'{name} distance: {distance_cm:.1f} cm')

        cv2.imshow("Pose Estimation", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ColorPoseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
