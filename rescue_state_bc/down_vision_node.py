import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from robot_msgs.msg import Detections as DetectionsMsg
import cv2 as cv
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class DownVisionNode(Node):
    vision_enabled = False
    black_line_size = 40000
    silver_line_size = 70000
    red_line_size = 200000

    lower_redline = np.array([0, 100, 100])
    upper_redline = np.array([10, 255, 255])
    lower_redline2 = np.array([170, 100, 100])
    upper_redline2 = np.array([180, 255, 255])
    def __init__(self):
        super().__init__('down_vision')
        self.msg = DetectionsMsg()
        self.bridge = CvBridge()
        self.declare_parameter('camera_topic', '/down_camera/camera_node/image_raw')

        self.get_logger().fatal(f'{type(self.lower_redline2)}')
        self.get_logger().fatal(f'{type(self.upper_redline2)}')
        self.get_logger().fatal(f'{type(self.lower_redline)}')
        self.get_logger().fatal(f'{type(self.upper_redline)}')
        self.get_logger().fatal(f'{self.lower_redline2}')
        self.get_logger().fatal(f'{self.upper_redline2}')

        # subscriptions

        camera_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.camera_sub = self.create_subscription(
            Image,
            self.get_parameter('camera_topic').value,
            self.image_callback,
            camera_qos
        )
        self.enable_sub = self.create_subscription(
            Bool,
            '/down_vision_enable',
            self.enable_callback,
            10
        )
        # publishers
        self.silver_angle_pub = self.create_publisher(
            Float32,
            '/silver_angle',
            10
        )
        self.silver_present_pub = self.create_publisher(
            Bool,
            '/silver_present',
            10
        )
        self.black_line_pub = self.create_publisher(
            Bool,
            '/black_present',
            10
        )
        self.red_present_pub = self.create_publisher(
            Bool,
            '/red_present',
            10
        )
        self.timer = self.create_timer(0.1, self.timer_callback)
    
    def enable_callback(self, msg):
        self.vision_enabled = msg.data
        if self.vision_enabled:
            self.get_logger().info('Down vision enabled')
        else:
            self.get_logger().info('Down vision disabled')

    def timer_callback(self):
        if hasattr(self, 'current_frame'):
            if self.vision_enabled:
                self.detect_silver(self.current_frame)
                self.silver_strip_angle(self.current_frame)
                self.detect_black(self.current_frame)
                self.detect_red(self.current_frame)

    def image_callback(self, msg):
        if not self.vision_enabled:
            return

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8"
        )
        self.current_frame = frame

    def detect_silver(self, image):
        silver = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        silver = cv.inRange(silver, 245, 255) # really bright spots
        kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (100, 100))
        silver = cv.morphologyEx(silver, cv.MORPH_CLOSE, kernel)
        silver_raw = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        silver_raw = cv.inRange(silver_raw, 245, 255)
        present = Bool()
        if cv.countNonZero(silver_raw) > self.silver_line_size:
            self.get_logger().info('Silver strip detected')
            present.data = True
        else:
            present.data = False
        self.get_logger().info(f'Silver present: {cv.countNonZero(silver_raw)}')
        self.silver_present_pub.publish(present)
        
    def silver_strip_angle(self, image):
        # find the angle of the silver strip relative to the robot
        # can use this to align with the strip and follow it
        pass

    def detect_black(self, image):
        image = cv.inRange(image, (0,0,0), (50,50,50)) # NEED Upadate
        line = Bool()
        if cv.countNonZero(image) > self.black_line_size:
            self.get_logger().info('Black line detected')
            line.data = True
        else:
            line.data = False
        self.get_logger().info(f'Black line present: {cv.countNonZero(image)}')
        self.black_line_pub.publish(line)

    def detect_red(self, image):
        image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        image = cv.GaussianBlur(image, (5, 5), 0)
        image1 = cv.inRange(image, (0, 100, 100), (10, 255, 255))
        image2 = cv.inRange(image, (170, 100, 100), (180, 255, 255))
        image = cv.bitwise_or(image1, image2)
        present = Bool()
        if cv.countNonZero(image) > self.red_line_size:
            self.get_logger().info('Red detected')
            present.data = True
        else:
            present.data = False
            self.get_logger().info(f'Red present: {cv.countNonZero(image)}')
        self.red_present_pub.publish(present)

def main(args=None):
    rclpy.init(args=args)
    node = DownVisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
