import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as Rscipy

class CameraPoseEstimator(Node):
    def __init__(self):
        super().__init__('camera_streamer')

        self.bridge = CvBridge()

        # Compressed image publisher
        self.publisher_image = self.create_publisher(CompressedImage, '/image_raw/compressed', 10)

        # Pose publisher
        self.publisher_pose = self.create_publisher(PoseStamped, '/marker_pose', 10)

        # Internal subscriber (used only if needed for internal debugging)
        # self.subscription = self.create_subscription(Image, '/image_raw/compressed', self.image_callback, 10)

        # OpenCV camera stream
        self.cap = cv2.VideoCapture('/dev/video0')
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera /dev/video0")
            return

        # Timer for publishing frames (~30 FPS)
        self.timer = self.create_timer(1.0 / 30.0, self.publish_frame)

        # Intrinsic camera parameters (approximated)
        self.camera_matrix = np.array([[800, 0, 320],
                                       [0, 800, 240],
                                       [0,   0,   1]], dtype=np.float64)
        self.dist_coeffs = np.zeros((4, 1))  # Assume no lens distortion

        # Marker 3D model points (in cm)
        w, h = 10, 6
        self.obj_points = np.array([
            [-w/2, -h/2, 0],
            [ w/2, -h/2, 0],
            [ w/2,  h/2, 0],
            [-w/2,  h/2, 0]
        ], dtype=np.float64)

        # HSV color bounds for marker detection
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
            bounds[name] = (np.array([max(h - 10, 0), 100, 100]),
                            np.array([min(h + 10, 179), 255, 255]))
        return bounds

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame")
            return

        # ========== PUBLISH COMPRESSED IMAGE ==========
        ret_jpeg, buffer = cv2.imencode('.jpg', frame)
        if not ret_jpeg:
            self.get_logger().warn("JPEG compression failed")
            return

        img_msg = CompressedImage()
        img_msg.header = Header()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = "camera_frame"
        img_msg.format = "jpeg"
        img_msg.data = np.array(buffer).tobytes()

        self.publisher_image.publish(img_msg)

        # ========== RUN POSE ESTIMATION ==========
        self.process_pose_estimation(frame)

    def process_pose_estimation(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        for name, (lower, upper) in self.color_bounds.items():
            mask = cv2.inRange(hsv, lower, upper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            best_cnt = max(contours, key=cv2.contourArea, default=None)

            if best_cnt is not None and cv2.contourArea(best_cnt) > 1000:
                rect = cv2.minAreaRect(best_cnt)
                box = cv2.boxPoints(rect).astype(np.float64)

                # Order corners consistently
                center = np.mean(box, axis=0)
                angles = np.arctan2(box[:,1] - center[1], box[:,0] - center[0])
                box = box[np.argsort(angles)]
                box = np.roll(box, -np.argmin(np.sum(box, axis=1)), axis=0)

                success, rvec, tvec = cv2.solvePnP(self.obj_points, box, self.camera_matrix, self.dist_coeffs)
                if success:
                    # ========== POSE MESSAGE ==========
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = name  # Use color as frame ID

                    pose_msg.pose.position.x = float(tvec[0])
                    pose_msg.pose.position.y = float(tvec[1])
                    pose_msg.pose.position.z = float(tvec[2])

                    R_matrix, _ = cv2.Rodrigues(rvec)
                    quat = Rscipy.from_matrix(R_matrix).as_quat()  # [x, y, z, w]
                    pose_msg.pose.orientation.x = quat[0]
                    pose_msg.pose.orientation.y = quat[1]
                    pose_msg.pose.orientation.z = quat[2]
                    pose_msg.pose.orientation.w = quat[3]

                    self.publisher_pose.publish(pose_msg)

                    # ========== OPTIONAL VISUALIZATION ==========
                    cv2.drawContours(frame, [box.astype(int)], 0, (0, 255, 255), 2)
                    origin = tuple(box[0].astype(int))
                    dist = np.linalg.norm(tvec)
                    cv2.putText(frame, f"{name}: {dist:.1f} cm", (origin[0]+10, origin[1]-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # Show window (optional)
        cv2.imshow("Pose Estimation", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CameraPoseEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

