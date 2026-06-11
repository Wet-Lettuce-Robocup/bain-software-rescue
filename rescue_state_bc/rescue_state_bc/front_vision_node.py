import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from robot_msgs.msg import Detection
import cv2 as cv
import numpy as np

# LATER: try adaptive thresholding

class Detections(Enum):
    G_TRAY = 0
    R_TRAY = 1
    S_VICTIM = 2
    B_VICTIM = 3

class FrontVisionNode(Node):
    lower_green = np.array([40, 100, 100])
    upper_green = np.array([80, 255, 255])
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])
    black_hsv_min = np.array([0, 0, 0])
    black_hsv_max = np.array([255, 30, 30])
    min_silver_ball_size = 100
    min_black_ball_size = 100
    min_tray_size = 1000 # minimum contour area to be considered a tray
    frame_width = 1536
    frame_height = 864
    center_thres = 10 # double this to get total pixel range
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
        self.scan_pub = self.create_publisher(
            Detections,
            '/scan_detections',
            10
        )

    def image_callback(self, msg):
        if not self.vision_enabled:
            return
        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8"
        )
        self.process_frame(frame)

    def process_frame(self, frame):
        # process the frame to detect trays and victims
        self.detect_green_tray(frame)
        self.detect_red_tray(frame)
        self.detect_silver_victims(frame)
        self.detect_black_victims(frame)
    
    def detect_green_tray(self, image):
        # convert to hsv color space
        image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        image = cv.GaussianBlur(image, (5, 5), 0)
        image = cv.inRange(image, self.lower_green, self.upper_green)
        contours, heirarchy = cv.findContours(image, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv.contourArea(contour) > self.min_tray_size:
                # find the center of mass of the contour
                M = cv.moments(contour)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                # check if the COM is near middle
                if cX < ((self.frame_width/2)+10) and cX > ((self.frame_width/2)-10):                 
                    self.msg.type = Detection.G_TRAY
                    self.msg.visible = True
                    self.msg.bearing = 0
                    self.msg.distance = None
                    self.detection_pub.publish(self.msg)
                    self.logger.info('Green tray detected')

    def detect_red_tray(self, image):
        # convert to hsv color space
        image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        image = cv.GaussianBlur(image, (5, 5), 0)
        image = cv.inRange(image, self.lower_red, self.upper_red)
        contours, heirarchy = cv.findContours(image, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv.contourArea(contour) > self.min_tray_size:
                # find the center of mass of the contour
                M = cv.moments(contour)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                # check if the COM is near middle
                if cX < ((self.frame_width/2)+self.center_thres) and cX > ((self.frame_width/2)-self.center_thres):
                    self.msg.type = Detection.R_TRAY
                    self.msg.visible = True
                    self.msg.bearing = 0
                    self.msg.distance = None
                    self.detection_pub.publish(self.msg)
                    self.logger.info('Red tray detected')

    def detect_silver_victims(self, image):
        image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        image = cv.inRange(image, 245, 255) # really bright spots
        contours, heirarchy = cv.findContours(image, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv.contourArea(contour) > self.min_silver_ball_size:
                # find the center of mass of the contour
                M = cv.moments(contour)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                if cX < ((self.frame_width/2)+self.center_thres) and cX > ((self.frame_width/2)-self.center_thres):
                    self.msg.type = Detection.S_VICTIM
                    self.msg.visible = True
                    self.msg.bearing = 0
                    self.msg.distance = None
                    self.detection_pub.publish(self.msg)
                    self.logger.info('Silver victim detected')

    def detect_black_victims(self, image):
        # convert to hsv
        image = cv.gaussianBlur(image, (5, 5), 0)
        image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        image = cv.inRange(image, self.black_hsv_min, self.black_hsv_max) # really dark spots
        image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        image = cv.inRange(image, 0, 30)
        contours, heirarchy = cv.findContours(image, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv.contourArea(contour) > self.min_black_ball_size:
                # find the center of mass of the contour
                M = cv.moments(contour)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                if cX < ((self.frame_width/2)+self.center_thres) and cX > ((self.frame_width/2)-self.center_thres):
                    self.msg.type = Detection.B_VICTIM
                    self.msg.visible = True
                    self.msg.bearing = 0
                    self.msg.distance = None
                    self.detection_pub.publish(self.msg)