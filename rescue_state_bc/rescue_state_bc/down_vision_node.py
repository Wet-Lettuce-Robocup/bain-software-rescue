import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from robot_msgs.msg import Detection
import cv2 as cv
import numpy as np

class DownVisionNode(Node):
    vision_enabled = False
    def __init__(self):
        super().__init__('vision')
        self.msg = Detection()

        # subscriptions
        self.camera_sub = self.create_subscription(
            Image,
            self.get_parameter('camera_topic').get_parameter_value().string_value,
            self.process_image,
            10
        )
        # publishers
        self.silver_angle_pub = self.create_publisher(
            float,
            '/scan_detections',
            10
        )
        self.silver_present_pub = self.create_publisher(
            bool,
            '/scan_detections',
            10
        )

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if self.vision_enabled:
            self.detect_silver_strips()
            self.silver_strip_angle()
        else:
            self.detect_silver_strips()

    def image_callback(self, msg):
        if not self.vision_enabled:
            return

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8"
        )
        self.process_frame(frame)

    def detect_silver_strips(self, image):
        image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        image = cv.inRange(image, 245, 255) # really bright spots
        if cv.countNonZero(image) > 100:
            self.logger.info('Silver strip detected')
            return True
        
    def silver_strip_angle(self, image):
        # find the angle of the silver strip relative to the robot
        # can use this to align with the strip and follow it
        pass