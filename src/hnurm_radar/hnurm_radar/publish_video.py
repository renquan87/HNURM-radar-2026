import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.declare_parameter('video_file', 'path/to/video.mp4')
        video_file = self.get_parameter('video_file').get_parameter_value().string_value
        self.get_logger().info('video_file: %s' % video_file)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.publisher_ = self.create_publisher(Image, 'image', qos_profile)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.cap = cv2.VideoCapture(video_file)
        self.bridge = CvBridge()
        self.get_logger().info('Publishing video from: %s' % video_file)
    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
        else:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()
    rclpy.spin(video_publisher)
    video_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()