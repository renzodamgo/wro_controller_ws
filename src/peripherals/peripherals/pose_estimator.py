import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import cv2
import numpy as np


def order_box_points(box):
    """
    Order 4 points in consistent order: TL, TR, BR, BL.
    box: (4,2) array-like
    returns: np.array shape (4,2) dtype=float64
    """
    pts = np.asarray(box, dtype=np.float64)
    # compute centroid
    c = pts.mean(axis=0)
    # compute angle from centroid to each point and sort by angle
    angles = np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0])
    # sort by angle ascending
    idx = np.argsort(angles)
    pts = pts[idx]
    # after sorting by angle, order could be: (-pi..pi). We want TL, TR, BR, BL clockwise.
    # rotate so that top-left (min y + min x) becomes first
    sum_coords = pts[:, 0] + pts[:, 1]
    tl_idx = np.argmin(sum_coords)
    ordered = np.roll(pts, -tl_idx, axis=0)
    return ordered


class ColorPoseEstimator(Node):
    def __init__(self):
        super().__init__('pose_estimator')

        # ROS2 subscriptions/publishers
        self.subscription = self.create_subscription(Image, '/usb_cam/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.color_pub = self.create_publisher(String, '/detected_color', 10)
        self.steering_pub = self.create_publisher(Float32, '/steering_angle', 10)
        self.image_pub = self.create_publisher(Image, '/detected_image', 10)

        # Tuned LAB color bounds (adjust for your camera)
        self.color_bounds = {
            "Red": (np.array([5, 134, 130], dtype=np.uint8), np.array([92, 170, 159], dtype=np.uint8)),
            "Green": (np.array([7, 105, 130], dtype=np.uint8), np.array([66, 124, 141], dtype=np.uint8))
        }

        # Kernel for morphology
        self.kernel = np.ones((4, 4), np.uint8)

        # Camera calibration
        self.camera_matrix = np.array([[545.4076, 0, 290.3681],
                                       [0, 545.3683, 208.3471],
                                       [0, 0, 1]], dtype=np.float64)
        self.dist_coeffs = np.array([[-0.5120, 0.3023, 0.0017, -0.0011, -0.0667]], dtype=np.float64)

        # Marker size in cm (width, height)
        self.marker_size = (5.0, 10.0)
        self.obj_points = np.array([
            [-self.marker_size[0] / 2.0, -self.marker_size[1] / 2.0, 0.0],
            [ self.marker_size[0] / 2.0, -self.marker_size[1] / 2.0, 0.0],
            [ self.marker_size[0] / 2.0,  self.marker_size[1] / 2.0, 0.0],
            [-self.marker_size[0] / 2.0,  self.marker_size[1] / 2.0, 0.0]
        ], dtype=np.float64)

        # Misc
        self.min_area = 1000  # contour area threshold
        self.max_display_width = 1280

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab)
        lab = cv2.GaussianBlur(lab, (7, 7), 0)

        detected_box = None
        detected_name = None
        distance_cm = None
        angle_deg = 0.0

        for name, (lower, upper) in self.color_bounds.items():
            # Create mask (LAB space)
            mask = cv2.inRange(lab, lower, upper)
            # morphology with tuned kernel
            mask = cv2.erode(mask, self.kernel, iterations=1)
            mask = cv2.dilate(mask, self.kernel, iterations=1)

            # Find contours (OpenCV >=4 returns contours, hierarchy)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue

            # find largest contour and validate
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            if area < self.min_area:
                continue

            x, y, w, h = cv2.boundingRect(largest_contour)

            # Aspect ratio filtering (avoid false positives)
            if not (h > w and (h / float(w)) > 1.2):
                # skip if shape not tall enough (per your original rule)
                continue

            # compute min area rect and box points
            rect = cv2.minAreaRect(largest_contour)
            box = cv2.boxPoints(rect)  # returns float32 array (4,2)
            # Order points robustly and ensure numpy array
            ordered_box = order_box_points(box)  # returns (4,2) float64
            image_points = np.array(ordered_box, dtype=np.float64)

            # SolvePnP: returns retval (bool), rvec, tvec
            retval, rvec, tvec = cv2.solvePnP(self.obj_points, image_points, self.camera_matrix, self.dist_coeffs)
            if not retval:
                continue

            # success: compute distance (obj_points in cm => tvec in cm)
            distance_cm = float(np.linalg.norm(tvec))
            detected_box = image_points  # np.array shape (4,2)
            detected_name = name
            break  # stop after first valid detection

        if detected_box is not None and detected_name is not None:
            # Draw detection box (convert to int)
            box_np = detected_box.astype(np.int32)
            cv2.drawContours(frame, [box_np], 0, (0, 255, 255), 2)

            # compute bbox center and normalized x
            bbox_center_x = np.mean(detected_box[:, 0])
            image_width = float(frame.shape[1])
            norm_x = float(bbox_center_x / image_width) if image_width > 0 else 0.5

            # write distance and label
            if distance_cm is not None:
                cv2.putText(frame, f"Dist: {distance_cm:.1f} cm", (20, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            cv2.putText(frame, detected_name, (20, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # Steering logic: distance-based
            MAX_ANGLE = 30.0
            if distance_cm is not None:
                if distance_cm < 40.0:
                    if detected_name == "Red":
                        angle_deg = -MAX_ANGLE if norm_x >= 0.5 else -20.0
                    elif detected_name == "Green":
                        angle_deg = 25.0 if norm_x <= 0.5 else 17.0
                    else:
                        angle_deg = 0.0
                else:
                    angle_deg = 0.0
            else:
                angle_deg = 0.0

            # Publish detection and steering
            try:
                self.color_pub.publish(String(data=detected_name))
                self.steering_pub.publish(Float32(data=float(angle_deg)))
            except Exception as e:
                self.get_logger().warn(f"publish error: {e}")

        # publish processed image (optional)
        try:
            proc_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.image_pub.publish(proc_msg)
        except Exception:
            # if publishing fails, ignore to keep node alive
            pass

        # show for debugging (will block in headless setups if no display - optional)
        try:
            cv2.imshow("LAB Detection", frame)
            cv2.waitKey(1)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ColorPoseEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # safe destroy of OpenCV windows
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == '__main__':
    main()

