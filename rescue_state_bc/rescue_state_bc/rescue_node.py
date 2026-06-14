import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from lifecycle_msgs.msg import Transition
from enum import Enum, auto
from robot_msgs.msg import Detections, scan_data
from std_msgs.msg import Int32, Bool
from rclpy.action import ActionClient
from robot_msgs.action import Move
import math

class State(Enum):
    ENTER = 0 # initial state when the robot enters the rescue area
    SEARCH = 1 # searching for the victims and ball trays
    APPROACH = 2 # approaching the victim and storing
    RESCUE = 3 # releasing victims into trays
    EXIT = 4 # exiting the rescue area after rescuing the victims

class Movement():
    def __init__(self, node):
        self.node = node
        # setup action clients
        self.move_client = ActionClient(
            node,
            Move,
            "move"
        )

    def drive(self, distance, angle=0, velocity=0.1):
        goal = Move.Goal()

        goal.distance = distance
        goal.angle = angle
        goal.vel = velocity

        self.busy = True

        self.move_client.wait_for_server()

        self.send_goal_future = self.move_client.send_goal_async(goal)

        self.send_goal_future.add_done_callback(
            self.goal_response_callback
        )

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        self.distance_travelled = feedback.distance_travelled 
        self.angle_turned = feedback.angle_turned

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted: # if goal is rejected, log error and set busy to false
            self.node.get_logger().error('Movement Goal rejected')
            self.busy = False
            return

        self.node.get_logger().info('Movement Goal accepted')

        self.get_result_future = goal_handle.get_result_async() # 
        self.get_result_future.add_done_callback( 
            self.result_callback
        )

    def result_callback(self, future):
        result = future.result().result

        if result.success:
            self.node.get_logger().info('Movement Goal success')
        else:
            self.node.get_logger().error('Movement Goal fail')

        self.busy = False
class Rescue(LifecycleNode):
    detected_objects = []
    dist_scan_samples = []
    robot_position = (0, 0) # x, y coordinates of the robot in the rescue area
    current_angle = 0 # angle the robot is currently facing, relative to the direction it
    latest_map = None

    def __init__(self):
        super().__init__('rescue_node')
        self.state = State.ENTER

    def on_configure(self):
        self.get_logger().info('Configuring Rescue Node...')
        self.move = Movement(self)
        # setup subscriptions
        self.camera_detection_subscriber = self.create_subscription(
            Detections,
            '/scan_detections',
            self.detection_callback,
            10
        )
        self.tof_subscriber = self.create_subscription(
            Int32,
            '/tof_front',
            self.laser_scan_callback,
            10
        )

        # setup publishers
        self.front_vision_enable_pub = self.create_publisher(
            Bool,
            '/front_vision_enable',
            10
        )
        self.fan_pub = self.create_publisher(
            Int32,
            '/fan_speed',
            10
        )
        self.claw_pub = self.create_publisher(
            Int32,
            '/claw_command',
            10
        )
        self.drive_pub = self.create_publisher(
            Int32,
            '??',
            10
        )
        self.scan_pub = self.create_publisher(
            scan_data,
            '/scan'
            10
        )

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self):
        self.get_logger().info('Activating...')

        self.create_timer(0.01, self.rescue_control_loop)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self):
        self.get_logger().info('Deactivating...')
        # stop timers
        self.destroy_timer(self.rescue_control_loop)
        return TransitionCallbackReturn.SUCCESS
    
    def detection_callback(self, msg):
        self.get_logger().info(f'Detection callback: {msg}')
        # append

        bearing = msg.bearing + self.move.angle_turned

        self.detected_objects.append(
            {
                'type': msg.type,
                'visible': msg.visible,
                'bearing': bearing,
                'distance': msg.distance
            }
        )

    def laser_scan_callback(self, msg):
        self.tof_distance = msg.data

    def face_bearing(self, bearing: float, rotate_vel=0.1):
        current = self.move.current_angle
        # normalize difference to [-180, 180] so it doesnt rotate the long way around
        diff = (bearing - current + math.pi) % (2*math.pi) - math.pi
        self.move.drive(0, diff, rotate_vel)
        self.move.current_angle += diff

    def rotate(self, angle: float, rotate_vel=0.1):
        self.move.drive(0, angle, rotate_vel)
        self.move.current_angle += angle

    def publish_scan(self):

        
    def rescue_control_loop(self):
        if self.state == State.ENTER:
            self.get_logger().info('Entering rescue area')
            # logic for entering the rescue area
            # drive forward test
            self.move.drive(0.5, 0, 0.1) # drive forward a bit to get into the rescue area

            self.rotate(0, -math.pi/2, 1) # rotate left initially

            self.state = State.SEARCH

        elif self.state == State.START_SEARCH:
            self.get_logger().info('Searching for victims and ball trays')

            # if scan_detections sees something, record the angle turned
            self.front_vision_enable_pub.publish(True)
            self.rotate(0, math.pi, 0.1) # roate slowly to search for objects
            self.state = State.SEARCH
        
        elif self.state == State.SEARCH:
            if self.move.busy == False and len(self.detected_objects) > 2:
                self.front_vision_enable_pub.publish(False)
                self.get_logger().info(f'Detected objects: {self.detected_objects}')
                self.rotate(0, -math.pi/2, 1) # rotate back to original orientation
                self.state = State.MAP

        elif self.state == State.START_MAP:
            self.get_logger().info('Mapping with distance sensor')
            # current point (how it entered) is (0,0) and angle is 0
            print(f"current angle: {self.move.current_angle} at position {self.robot_position} (should be 0 and (0,0))")
            self.rotate(0, 2*math.pi, 0.1) 

            self.state = State.MAP

        elif self.state == State.MAP:
            if self.move.busy == False:
                self.publish_scan()
                self.APPROACH #when it finishes rotating
            
            self.dist_scan_samples.append(
                {
                    'angle': self.move.current_angle,
                    'distance': self.tof_distance
                }
            )
            self.get_logger().info('Mapping victim and ball tray locations')


        elif self.state == State.APPROACH:
            self.get_logger().info('Approaching victim and storing')
            # logic for approaching the victim and storing
            self.state = State.LOCALISE

        elif self.state == State.RESCUE:
            self.get_logger().info('Rescuing victims into trays')
            # logic for rescuing victims into trays

            self.state = State.EXIT


        elif self.state == State.EXIT:
            self.get_logger().info('Exiting rescue area')
            # logic for exiting the rescue area after rescuing the victims

            self.on_deactivate() # deactivate the node after exiting the rescue area