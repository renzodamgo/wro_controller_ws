import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import subprocess
import os
import time

class RPiCamNode(Node):
    def __init__(self):
        super().__init__('rpicam_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()

        self.pipe_path = '/tmp/vidpipe'
        self.ensure_pipe()
        self.start_camera_process()
        time.sleep(1)  # let the pipe fill

        self.cap = cv2.VideoCapture(self.pipe_path)
        if not self.cap.isOpened():
            self.get_logger().error('Could not open video stream')
            return

        self.timer = self.create_timer(0.03, self.timer_callback)  # ~30 FPS

    def ensure_pipe(self):
        if os.path.exists(self.pipe_path):
            os.remove(self.pipe_path)
        os.mkfifo(self.pipe_path)

    def start_camera_process(self):
        cmd = [
            'rpicam-vid',
            '-t', '0',
            '--nopreview',
            '--width', '640',
            '--height', '480',
            '-o', self.pipe_path,
            '--libav-format', 'mpegts'
        ]
        self.cam_proc = subprocess.Popen(cmd)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame')
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)

    def destroy_node(self):
        self.get_logger().info('Shutting down...')
        if self.cam_proc:
            self.cam_proc.terminate()
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RPiCamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

