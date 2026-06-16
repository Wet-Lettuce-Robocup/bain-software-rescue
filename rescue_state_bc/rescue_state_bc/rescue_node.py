import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from lifecycle_msgs.msg import Transition
from enum import Enum, auto
from robot_msgs.msg import Detections
from std_msgs.msg import Int32, Bool
from sensor_msgs.msg import LaserScan
from rclpy.action import ActionClient
from robot_msgs.action import Move
import math
import matplotlib.pyplot as plt
# HI WILL CAN U SEE. THIS

class State(Enum):
    ENTER = auto()        # initial state when the robot enters the rescue area
    START_SEARCH = auto() # start searching (kick off rotation / vision)
    SEARCH = auto()       # searching for the victims and ball trays
    START_MAP = auto()    # prepare to map (clear samples, start rotation)
    MAP = auto()          # mapping with distance sensor
    LOCALISE = auto()     # localise
    APPROACH = auto()     # approaching the victim and storing
    RESCUE = auto()       # releasing victims into trays
    EXIT = auto()         # exiting the rescue area after rescuing the victims

class Movement():
    def __init__(self, node):
        self.node = node
        # setup action clients
        self.move_client = ActionClient(
            node,
            Move,
            "move"
        )
        # runtime state
        self.busy = False
        self.current_angle = 0.0
        self.distance_travelled = 0.0
        self.angle_turned = 0.0

    def drive(self, distance, angle=0, velocity=0.1):
        goal = Move.Goal()

        goal.distance = distance
        goal.angle = angle
        goal.vel = velocity

        self.busy = True

        # wait for action server to appear (short timeout so we fail fast if it's not available)
        try:
            available = self.move_client.wait_for_server(timeout_sec=2.0)
        except Exception as e:
            self.node.get_logger().error(f'wait_for_server exception: {e}')
            self.busy = False
            return

        if not available:
            self.node.get_logger().error('Action server "move" not available (timeout)')
            self.busy = False
            return

        try:
            # register feedback callback so we get ongoing updates
            self.send_goal_future = self.move_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
            self.send_goal_future.add_done_callback(self.goal_response_callback)
        except Exception as e:
            self.node.get_logger().error(f'Failed to send goal: {e}')
            self.busy = False
            return

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        # update movement feedback when available
        self.distance_travelled = getattr(feedback, 'distance_travelled', self.distance_travelled)
        self.angle_turned = getattr(feedback, 'angle_turned', self.angle_turned)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.node.get_logger().error(f'Goal response future exception: {e}')
            self.busy = False
            return

        if not getattr(goal_handle, 'accepted', False):
            # if goal is rejected, log error and set busy to false
            self.node.get_logger().error('Movement Goal rejected')
            self.busy = False
            return

        self.node.get_logger().info('Movement Goal accepted')

        try:
            self.get_result_future = goal_handle.get_result_async()
            self.get_result_future.add_done_callback(self.result_callback)
        except Exception as e:
            self.node.get_logger().error(f'Failed to request result: {e}')
            self.busy = False
            return

    def result_callback(self, future):
        try:
            res = future.result()
            result = getattr(res, 'result', res)
        except Exception as e:
            self.node.get_logger().error(f'Get result future exception: {e}')
            self.busy = False
            return

        success = getattr(result, 'success', None)
        if success is True:
            self.node.get_logger().info('Movement Goal success')
        elif success is False:
            self.node.get_logger().error('Movement Goal fail')
        else:
            # unknown result type; log for debugging
            self.node.get_logger().info(f'Movement Goal result: {result}')

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
        # control timer handle
        self.control_timer = None
        # sensor offset (m): sensor is 100 mm in front of pivot
        self.sensor_offset = 0.1
        # last sample angle used to avoid duplicate samples
        self._last_sample_angle = None
        # latest tof distance in meters
        self.tof_distance = float('inf')
        # ensure Movement will be created in configure
        self.move = None
        # storage for scan samples collected during mapping
        self.dist_scan_samples = []

    def on_configure(self, state):
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
            '/drive_command',
            10
        )
        self.scan_pub = self.create_publisher(
            LaserScan,
            '/scan',
            10
        )

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('Activating...')
        self.control_timer = self.create_timer(0.01, self.rescue_control_loop)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.get_logger().info('Deactivating...')
        # stop timers
        if self.control_timer is not None:
            try:
                self.destroy_timer(self.control_timer)
            except Exception:
                pass
            self.control_timer = None
        return TransitionCallbackReturn.SUCCESS
    
    def detection_callback(self, msg):
        # record detections from camera with absolute bearing (robot frame)
        try:
            bearing = float(msg.bearing) + float(self.move.current_angle)
        except Exception:
            bearing = float(getattr(msg, 'bearing', 0.0)) + getattr(self.move, 'current_angle', 0.0)

        self.get_logger().debug(f'Detection callback bearing={bearing}')

        self.detected_objects.append({
            'type': getattr(msg, 'type', None),
            'visible': getattr(msg, 'visible', True),
            'bearing': bearing,
            'distance': getattr(msg, 'distance', 0.0)
        })

    def laser_scan_callback(self, msg):
        # tof publishes an integer (mm) — convert to meters
        try:
            self.tof_distance = float(msg.data) / 1000.0
        except Exception:
            self.tof_distance = float(getattr(msg, 'data', float('inf')))

    def face_bearing(self, bearing: float, rotate_vel=0.1):
        current = self.move.current_angle
        # normalize difference to [-pi, pi]
        diff = (bearing - current + math.pi) % (2 * math.pi) - math.pi
        self.move.drive(0, diff, rotate_vel)
        # optimistic update of angle while action runs
        self.move.current_angle = (self.move.current_angle + diff) % (2 * math.pi)

    def rotate(self, distance: float, angle: float, rotate_vel=0.1):
        # wrapper to issue rotate (keeps existing call signatures in code)
        self.move.drive(distance, angle, rotate_vel)
        # optimistic update
        self.move.current_angle = (self.move.current_angle + angle) % (2 * math.pi)

    def publish_scan(self):
        scan = LaserScan()

        if len(self.dist_scan_samples) < 3:
            self.get_logger().warn('Not enough scan samples to publish')
            return

        scan.header.frame_id = "base_link"
        scan.header.stamp = self.get_clock().now().to_msg()

        # data entries: list of {'angle': <rad>, 'distance': <m>}
        data = sorted(self.dist_scan_samples, key=lambda x: x['angle'])

        scan.angle_min = float(data[0]['angle'])
        scan.angle_max = float(data[-1]['angle'])

        n = len(data)
        scan.angle_increment = (scan.angle_max - scan.angle_min) / max(n - 1, 1)

        # convert sensor readings (from sensor origin) to ranges relative to base_link (pivot)
        ranges = []
        for entry in data:
            d = float(entry['distance'])
            if math.isfinite(d) and d > 0.0:
                # add the forward offset of the sensor
                r = d + self.sensor_offset
            else:
                r = float('inf')
            ranges.append(r)

        scan.ranges = ranges
        # optional metadata
        scan.range_min = 0.02
        scan.range_max = 30.0

        self.scan_pub.publish(scan)
        self.get_logger().info(f'Published scan with {n} samples')

        # clear collected samples
        self.dist_scan_samples = []

    def analyse_scan(self):
        # converts the polar samples to cartesian coords
        for sample in self.dist_scan_samples:
            x = sample['distance'] * math.cos(sample['angle'])
            y = sample['distance'] * math.sin(sample['angle'])
            sample['angle']
            self.scan_points.append({
                'x': x,
                'y': y
            })
        
    def draw_map(self):
        # draws a map and saves an image (for debugging)
        x = [p['x'] for p in self.scan_points]
        y = [p['y'] for p in self.scan_points]
        plt.scatter(x, y)
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('Scan Map')
        plt.axis('equal')
        plt.savefig('scan_map.png')
        self.get_logger().info('Saved scan map to scan_map.png')
            
    def rescue_control_loop(self):
        if self.state == State.ENTER:
            self.get_logger().info('Entering rescue area')
            # logic for entering the rescue area
            # drive forward test
            self.move.drive_blocking(0.5, 0, 0.1) # drive forward a bit to get into the rescue area
            self.move.rotate_blocking(0, -math.pi/2, 1) # rotate left initially
            self.get_logger().info('Initial rotation complete, starting search')
            self.state = State.START_SEARCH

        elif self.state == State.START_SEARCH:
            self.get_logger().info('Searching for victims and ball trays')

            # if scan_detections sees something, record the angle turned
            self.front_vision_enable_pub.publish(Bool(data=True)) # enable front vision to start searching
            self.rotate(0, math.pi, 0.1) # roate slowly to search for objects
            self.state = State.SEARCH
        
        elif self.state == State.SEARCH:
            # wait for sweep to finish, or if detections found
            if getattr(self.move, 'busy', False) == False and len(self.detected_objects) > 0:
                self.front_vision_enable_pub.publish(Bool(data=False))
                self.get_logger().info(f'Detected objects: {self.detected_objects}')
                self.rotate(0, -math.pi/2, 1) # rotate back to original orientation
                self.state = State.MAP

        elif self.state == State.START_MAP:
            self.get_logger().info('Mapping with distance sensor')
            # clear previous samples and start a full rotation to map surroundings
            self.dist_scan_samples = []
            self._last_sample_angle = None
            self.rotate(0, 2 * math.pi, 0.1)
            self.state = State.MAP

        elif self.state == State.MAP:
            # while rotating, sample TOF and record (angle, distance)
            try:
                angle = float(self.move.current_angle)
            except Exception:
                angle = float(getattr(self.move, 'current_angle', 0.0))

            do_append = False
            if self._last_sample_angle is None:
                do_append = True
            elif abs(angle - self._last_sample_angle) > 0.01:
                do_append = True

            if do_append:
                self.dist_scan_samples.append({
                    'angle': angle,
                    'distance': float(self.tof_distance)
                })
                self._last_sample_angle = angle

            # if rotation finished, publish and move on
            if getattr(self.move, 'busy', False) == False:
                self.get_logger().info('Completed mapping rotation; publishing scan')
                self.analyse_scan()
                self.state = State.APPROACH


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