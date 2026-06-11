import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from robot_msgs.msg import Detections as DetectionsMsg
import cv2 as cv
import numpy as np

class DownVisionNode(Node):
    vision_enabled = False
    def __init__(self):
        super().__init__('down_vision')
        self.msg = DetectionsMsg()
        self.declare_parameter('camera_topic', '/camera/image_raw')

        # subscriptions
        self.camera_sub = self.create_subscription(
            Image,
            self.get_parameter('camera_topic').get_parameter_value().string_value,
            self.image_callback,
            10
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
                self.detect_silver_strips(self.current_frame)
                self.silver_strip_angle(self.current_frame)
            else:
                self.detect_silver_strips(self.current_frame)

    def image_callback(self, msg):
        if not self.vision_enabled:
            return

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8"
        )
        self.current_frame = frame

    def detect_silver_strips(self, image):
        image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        image = cv.inRange(image, 245, 255) # really bright spots
        present = Bool()
        if cv.countNonZero(image) > 100:
            self.get_logger().info('Silver strip detected')
            present.data = True
        else:
            present.data = False
        self.silver_present_pub.publish(present)
        
    def silver_strip_angle(self, image):
        # find the angle of the silver strip relative to the robot
        # can use this to align with the strip and follow it
        pass

