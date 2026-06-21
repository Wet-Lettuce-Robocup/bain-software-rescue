import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from lifecycle_msgs.msg import Transition
from enum import Enum, auto
from robot_msgs.msg import Detections, LEDCommand
from std_msgs.msg import Int32, Bool, ColorRGBA
from sensor_msgs.msg import LaserScan
from rclpy.action import ActionClient
from robot_msgs.action import MoveTime
import math
import matplotlib.pyplot as plt

class State(Enum):
    ENTER = auto()        # initial state when the robot enters the rescue area
    ENTER_DRIVE = auto()  # driving into rescue area (non-blocking)
    ENTER_ROTATE = auto() # rotating to initial heading (non-blocking)
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
        # Use the time-based movement action server
        self.move_client = ActionClient(
            node,
            MoveTime,
            "move_time"
        )
        # runtime state
        self.busy = False
        self.current_angle = 0.0
        self.distance_travelled = 0.0
        self.angle_turned = 0.0
        # store last goal velocities so feedback (time) can be mapped to distance/angle
        self._last_goal_vel = 0.0
        self._last_goal_angular_vel = 0.0
        self._last_goal_time = 0.0

        # sequence state used by run_sequence, _advance_sequence
        self._sequence = None
        self._on_complete = None

    def drive(self, distance, angle=0, velocity=0.1):
        # compute required times (guard against zero velocity)
        linear_time = abs(distance) / abs(velocity) if velocity != 0 and distance != 0 else 0.0
        angular_time = abs(angle) / abs(velocity) if velocity != 0 and angle != 0 else 0.0

        time_required = max(linear_time, angular_time)

        if time_required <= 0.0:
            # nothing to do
            self.node.get_logger().warn('Drive called with zero distance and angle; ignoring')
            return

        goal = MoveTime.Goal()
        goal.time = float(time_required)

        # preserve direction using copysign
        linear_vel = math.copysign(velocity, distance) if distance != 0 else 0.0
        angular_vel = math.copysign(velocity, angle) if angle != 0 else 0.0

        goal.vel = float(linear_vel)
        goal.angular_vel = float(angular_vel)

        # remember for feedback mapping
        self._last_goal_vel = float(linear_vel)
        self._last_goal_angular_vel = float(angular_vel)
        self._last_goal_time = float(time_required)

        self.busy = True

        # wait for action server to appear (short timeout so we fail fast if it's not available)
        try:
            available = self.move_client.wait_for_server(timeout_sec=2.0)
        except Exception as e:
            self.node.get_logger().error(f'wait_for_server exception: {e}')
            self.busy = False
            return

        if not available:
            self.node.get_logger().error('Action server "move_time" not available (timeout)')
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

        # New MoveTime feedback exposes elapsed time; map that to distance/angle
        time_elapsed = getattr(feedback, 'time_elapsed', None)
        if time_elapsed is not None:
            # `time_elapsed` may be in seconds or nanoseconds depending on server
            te = float(time_elapsed)
            # heuristic: if value is huge, assume nanoseconds and convert to seconds
            te_sec = te * 1e-9 if te > 1e6 else te

            # map elapsed time to travelled distance and turned angle using last goal velocities
            self.distance_travelled = self._last_goal_vel * te_sec
            self.angle_turned = self._last_goal_angular_vel * te_sec
        else:
            # fallback to legacy feedback fields if present
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

        # if a sequence is running, advance to its next step once result is received
        if self._on_complete is not None:
            self._on_complete()

    def _advance_sequence(self): #underscore since its internal
        if self._sequence is None: # do not advance if no sequence is running
            return
        try:
            next(self._sequence) # advance to next
        except StopIteration: # sequence is complete
            self._sequence = None
            self._on_complete = None

    def run_sequence(self, sequence_gen):
        #   def my_sequence(self):
        #       self.move.drive(0.5, 0, 0.1)
        #       yield
        #       if self.tof_distance < 0.2:
        #           self.move.drive(-0.1, 0, 0.1)
        #       else:
        #           self.move.drive(0.3, 0, 0.1)
        #       yield
        #       self.state = State.RESCUE

        #   elif self.state == State.START_APPROACH:
        #       self.move.run_sequence(self.my_sequence())   # note the () — pass the generator
        #       self.state = State.APPROACH
        #
        #   elif self.state == State.APPROACH:
        #       pass   # nothing needed here; the sequence drives itself to completion

        self._sequence = sequence_gen
        self._on_complete = self._advance_sequence
        # kick off the first step immediately
        self._advance_sequence()


class Rescue(LifecycleNode):
    detected_objects = []
    dist_scan_samples = []
    robot_position = (0, 0) # x, y coordinates of the robot in the rescue area
    current_angle = 0 # angle the robot is currently facing, relative to the direction it
    latest_map = None

    def __init__(self):
        super().__init__('rescue_node')
        self.state = State.ENTER
        self._last_state = None
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

        # LED publisher will be created in on_configure
        self.led_cmd_pub = None

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

        # LED command publisher for first LED (index 0)
        self.led_cmd_pub = self.create_publisher(
            LEDCommand,
            'led_command',
            10,
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
        # record detections from camera with absolute bearing AND pixel location
        try:
            bearing = float(msg.bearing) + float(self.move.current_angle)
        except Exception:
            bearing = float(getattr(msg, 'bearing', 0.0)) + getattr(self.move, 'current_angle', 0.0)

        self.get_logger().debug(f'Detection callback bearing={bearing}')

        self.detected_objects.append({
            'type': getattr(msg, 'type', None),
            'visible': getattr(msg, 'visible', True),
            'bearing': bearing,
            'xpixel': getattr(msg, 'xpixel', None),
            'distance': getattr(msg, 'distance', None)
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
        
    def draw_map(self): #DEBUGGING 
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
            
    def _publish_led_for_state(self, state: State) -> None:
        """Publish an LEDCommand for the first LED (index 0) based on state."""
        if self.led_cmd_pub is None:
            return

        # mapping of states to RGBA colors
        mapping = {
            State.ENTER: (0.0, 0.0, 1.0, 1.0),        # blue
            State.ENTER_DRIVE: (0.0, 1.0, 0.0, 1.0),  # green
            State.ENTER_ROTATE: (1.0, 1.0, 0.0, 1.0), # yellow
            State.START_SEARCH: (1.0, 0.0, 1.0, 1.0), # magenta
            State.SEARCH: (0.0, 1.0, 1.0, 1.0),       # cyan
            State.START_MAP: (1.0, 0.5, 0.0, 1.0),    # orange
            State.MAP: (0.5, 0.0, 0.5, 1.0),          # purple
            State.LOCALISE: (1.0, 1.0, 1.0, 1.0),     # white
            State.APPROACH: (0.5, 1.0, 0.0, 1.0),     # lime
            State.RESCUE: (1.0, 0.0, 0.0, 1.0),       # red
            State.EXIT: (0.0, 0.0, 0.0, 1.0),         # off
        }

        rgba = mapping.get(state, (0.0, 0.0, 0.0, 1.0))

        cmd = LEDCommand()
        cmd.index = 0
        color = ColorRGBA()
        color.r, color.g, color.b, color.a = rgba
        cmd.color = color

        try:
            self.led_cmd_pub.publish(cmd)
        except Exception as e:
            self.get_logger().error(f'Failed to publish LEDCommand: {e}')

    def rescue_control_loop(self):
        self.get_logger().info(f'STATE: {self.state.name}')
        # publish LED color on state changes for the first LED
        if getattr(self, '_last_state', None) != self.state:
            try:
                self._publish_led_for_state(self.state)
            except Exception as e:
                self.get_logger().error(f'Failed to publish LED for state {self.state}: {e}')
            self._last_state = self.state

        if self.state == State.ENTER:
            self.move.drive(0.03, 0, 0.015)
            self.state = State.ENTER_DRIVE

        elif self.state == State.ENTER_DRIVE:
            # wait for the initial drive to finish, then start rotation
            if getattr(self.move, 'busy', False) == False:
                self.move.drive(0, 0.03, 0.015)
                self.state = State.ENTER_ROTATE

        elif self.state == State.ENTER_ROTATE:
            # wait for the initial rotation to finish, then begin searching
            if getattr(self.move, 'busy', False) == False:
                self.state = State.START_SEARCH

        elif self.state == State.START_SEARCH:
            # if scan_detections sees something, record the angle turned
            self.front_vision_enable_pub.publish(Bool(data=True)) # enable front vision to start searching
            self.rotate(0, 0.06, 0.015) # roate slowly to search for objects
            self.state = State.SEARCH
        
        elif self.state == State.SEARCH:
            # wait for sweep to finish, or if detections found
            if getattr(self.move, 'busy', False) == False and len(self.detected_objects) > 0:
                self.front_vision_enable_pub.publish(Bool(data=False))
                self.get_logger().info(f'Detected objects: {self.detected_objects}')
                self.get
                self.rotate(0, 1, 0.1) # rotate back to original orientation
                self.state = State.MAP

        elif self.state == State.START_MAP:
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
            # rotate toward black victim
            
            # check if black victim is detected by front vision
            # check distance to victim
            # drive forward 1/2 of the distance to the victim
            # check again if victim is detected by front vision
            # if still detected, drive forward new distance - 125mm
            # move claw down
            # open claw
            # turn right until claw tof sees <90mm
            # drive forward until claw tof sees <50mm
            # close claw to grab victim
            # drive backward 0.1m
            # lift claw
            # release claw
            # return to start search state
            self.state = State.LOCALISE

        elif self.state == State.RESCUE:
            self.get_logger().info('Rescuing victims into trays')
            # logic for rescuing victims into trays

            self.state = State.EXIT


        elif self.state == State.EXIT:
            self.get_logger().info('Exiting rescue area')
            # logic for exiting the rescue area after rescuing the victims

            self.on_deactivate() # deactivate the node after exiting the rescue area

def main(args=None):
    rclpy.init(args=args)
    node = Rescue()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()