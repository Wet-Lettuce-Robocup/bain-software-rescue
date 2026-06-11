import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from lifecycle_msgs.msg import Transition
from enum import Enum, auto
from robot_msgs.msg import Detections
from std_msgs.msg import Int32
from rclpy.action import ActionClient
from robot_msgs.action import Move

class State(Enum):
    ENTER = 0 # initial state when the robot enters the rescue area
    SEARCH = 1 # searching for the victims and ball trays
    APPROACH = 2 # approaching the victim and storing
    RESCUE = 3 # releasing victims into trays
    EXIT = 4 # exiting the rescue area after rescuing the victims

class Movement():
    def __init__(self, node):
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

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted: # if goal is rejected, log error and set busy to false
            self.get_logger().error('Movement Goal rejected')
            self.busy = False
            return

        self.get_logger().info('Movement Goal accepted')

        self.get_result_future = goal_handle.get_result_async() # 
        self.get_result_future.add_done_callback( 
            self.get_result_callback
        )

    def result_callback(self, future):
        result = future.result().result

        if result.success:
            self.get_logger().info('Movement Goal success')
        else:
            self.get_logger().error('Movement Goal fail')

        self.busy = False
class Rescue(LifecycleNode):
    def __init__(self):
        super().__init__('rescue_node')

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
        self.fan_pub = self.create_publisher(Int32, '/fan_speed', 10)
        self.claw_pub = self.create_publisher(Int32, '/claw_command', 10)
        self.drive_pub = self.create_publisher(Int32, '??', 10)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self):
        self.get_logger().info('Activating...')
        # enable publishers and timers
        self.create_timer(0.01, self.rescue_control_loop)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self):
        self.get_logger().info('Deactivating...')
        # stop timers
        self.destroy_timer(self.rescue_control_loop)
        return TransitionCallbackReturn.SUCCESS

    def rescue_control_loop(self):
        if self.state == State.ENTER:
            self.get_logger().info('Entering rescue area')
            # logic for entering the rescue area
            # drive forward test
            self.move.drive(1.0, 0, 0.1)
            
            self.state = State.SEARCH

        elif self.state == State.SEARCH:
            self.get_logger().info('Searching for victims and ball trays')
            # logic for searching for victims and ball trays
            self.state = State.APPROACH

        elif self.state == State.APPROACH:
            self.get_logger().info('Approaching victim and storing')
            # logic for approaching the victim and storing
            self.state = State.RESCUE

        elif self.state == State.RESCUE:
            self.get_logger().info('Rescuing victims into trays')
            # logic for rescuing victims into trays
            self.state = State.EXIT

        elif self.state == State.EXIT:
            self.get_logger().info('Exiting rescue area')
            # logic for exiting the rescue area after rescuing the victims
            self.on_deactivate() # deactivate the node after exiting the rescue area