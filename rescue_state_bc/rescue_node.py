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
from geometry_msgs.msg import Twist
import math
import matplotlib.pyplot as plt
from robot_msgs.srv import Inference

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
    # detected objects format: list of {'type': <str>, 'visible': <bool>, 'bearing': <float radians>, 'xpixel': <int>, 'distance': <float meters>}
    dist_scan_samples = []
    silver_victims_collected = 0
    black_victims_collected = 0
    robot_position = (0, 0) # x, y coordinates of the robot in the rescue area
    current_angle = 0 # angle the robot is currently facing, relative to the direction it
    latest_map = None
    exit_kp = 0.5 # proportional gain for exit behavior
    black_line_seen = False

    def __init__(self):
        super().__init__('rescue_node')
        self.state = 1
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

        # inference future (async detection requests)
        self._detection_future = None



    def on_configure(self, state):
        self.get_logger().info('Configuring Rescue Node...')
        self.move = Movement(self)
        # setup subscriptions
        self.front_tof_subscriber = self.create_subscription(
            Int32,
            '/tof_front',
            self.laser_scan_callback,
            10
        )
        self.claw_tof_subscriber = self.create_subscription(
            Int32,
            '/tof_claw',
            self.laser_scan_callback,
            10
        )
        self.side_tof_subscriber = self.create_subscription(
            Int32,
            '/tof_side',
            self.laser_scan_callback,
            10
        )
        self.black_line_subscriber = self.create_subscription(
            Bool,
            '/black_present',
            self.black_line_callback,
            10
        )
        self.ball_client = self.create_client(Inference, '/ml_rescue/detections')
        while not self.ball_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for ball service...')

        # setup publishers
        self.front_vision_enable_pub = self.create_publisher(
            Bool,
            '/front_vision_enable',
            10
        )
        self.down_vision_enable_pub = self.create_publisher(
            Bool,
            '/down_vision_enable',
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
        self.led_cmd_pub = self.create_publisher(
            LEDCommand,
            'led_command',
            10,
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('Activating...')
        self.control_timer = self.create_timer(0.01, self.rescue_control_loop)
        self.move.move_client.wait_for_server()

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

    def _request_detections(self):
        if self._detection_future is not None and not self._detection_future.done():
            return  # previous call still in flight
        self.get_logger().debug('Requesting detections from inference service...')
        req = Inference.Request()
        req.message = 'whereball'
        self._detection_future = self.ball_client.call_async(req)
        self.get_logger().debug('Detection request sent to inference service')

    def _detections_ready(self):
        if self._detection_future is None or not self._detection_future.done():
            return False
        try:
            response = self._detection_future.result()
        except Exception as e:
            self.get_logger().error(f'Inference service call failed: {e}')
            self._detection_future = None
            return False
        self._detection_future = None

        if not response.success:
            self.get_logger().warn('Inference service returned success=False')
            return False

        # bearing is relative to the robot (0 = straight ahead)
        self.detected_objects = [
            {
                'type': t,
                'bearing': b,
                'confidence': c,
                'distance': d,
                'cx': x,
            }
            for t, b, c, d, x in zip(
                response.type,
                response.bearing,
                response.confidence,
                response.distance,
                response.cx,
            )
        ]
        self.get_logger().debug(f'Detections: {len(self.detected_objects)}')
        return True

    def black_line_callback(self, msg):
        if msg.data:
            self.black_line_seen = True

    def laser_scan_callback(self, msg):
        # tof publishes an integer (mm) — convert to meters
        try:
            self.front_tof_distance = float(msg.data) / 1000.0
        except Exception:
            self.front_tof_distance = float(getattr(msg, 'data', float('inf')))
        try:
            self.claw_tof_distance = float(msg.data) / 1000.0
        except Exception:
            self.claw_tof_distance = float(getattr(msg, 'data', float('inf')))
        try:
            self.side_tof_distance = float(msg.data) / 1000.0
        except Exception:
            self.side_tof_distance = float(getattr(msg, 'data', float('inf')))

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
            
    def _publish_led_for_state(self, state) -> None:
        """Publish an LEDCommand for the first LED (index 0) based on state."""
        if self.led_cmd_pub is None:
            return

        # mapping of states to RGBA colors
        mapping = {
            1: (0.0, 0.0, 1.0, 1.0),        # blue
            2: (0.0, 1.0, 0.0, 1.0),  # green
            3: (1.0, 1.0, 0.0, 1.0), # yellow
            4: (1.0, 0.0, 1.0, 1.0), # magenta
            5: (0.0, 1.0, 1.0, 1.0),       # cyan
            6: (1.0, 0.5, 0.0, 1.0),    # orange
            7: (0.5, 0.0, 0.5, 1.0),          # purple
            8: (1.0, 1.0, 1.0, 1.0),     # white
            9: (0.5, 1.0, 0.0, 1.0),     # lime
            10: (1.0, 0.0, 0.0, 1.0),       # red
            11: (0.0, 0.0, 0.0, 1.0),         # off
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

    def publish_cmd_vel(self, linear_x=0.0, angular_z=0.0):
        if self.cmd_vel_pub is None:
            return

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z

        try:
            self.cmd_vel_pub.publish(twist)
        except Exception as e:
            self.get_logger().error(f'Failed to publish cmd_vel: {e}')

    def _find_target(self, target_type):
        for obj in self.detected_objects:
            if obj.get('type') == target_type:
                return obj
        return None

    def _current_target_type(self):
        # Silver first, then black
        return 'silver' if self.silver_victims_collected < 2 else 'black'

    def rescue_control_loop(self):
        #self.get_logger().info(f'STATE: {self.state.name}')
        # publish LED color on state changes for the first LED
        if getattr(self, '_last_state', None) != self.state:
            try:
                self._publish_led_for_state(self.state)
            except Exception as e:
                self.get_logger().error(f'Failed to publish LED for state {self.state}: {e}')
            self._last_state = self.state

        # move in
        if self.state == 1:
                self.move.drive(140, 0, 100)
                self.state = 2

        # wait for drive in to finish then rotate right
        elif self.state == 2:
            if not getattr(self.move, 'busy', False):
                self.move.drive(0, 180, 100)
                self.state = 3

        # search for the current target
        elif self.state == 3:
            if not getattr(self.move, 'busy', False):
                self.current_target = self._current_target_type()
                self._request_detections()
                if self._detections_ready():
                    target = self._find_target(self.current_target)
                    if target is not None:
                        self.target_type = self.current_target
                        self.target_detection = target
                        self.state = 5
                    else:
                        self.move.drive(0, 180, -100)
                        self.state = 4

        # keep scanning by rotating left, then go back to search
        elif self.state == 4:
            if not getattr(self.move, 'busy', False):
                self.state = 3

        # align to target bearing
        elif self.state == 5:
            self._request_detections()
            if self._detections_ready():
                target = self._find_target(self.target_type)
                if target is None:
                    self.get_logger().warn(f'{self.target_type} victim disappeared, returning to search')
                    self.state = 3
                else:
                    bearing = target['bearing']
                    if abs(bearing) < 0.2:
                        # already aligned, go straight to approach
                        self.state = 6
                    else:
                        self.move.drive(0, bearing, 100)
                        self.state = 6

        # wait for alignment, then approach target
        elif self.state == 6:
            if not getattr(self.move, 'busy', False):
                self._request_detections()
                if self._detections_ready():
                    target = self._find_target(self.target_type)
                    if target is None:
                        self.get_logger().warn(f'{self.target_type} victim disappeared, returning to search')
                        self.state = 3
                    else:
                        bearing = target['bearing']
                        if abs(bearing) < 0.2:
                            distance = target['distance']
                            self.move.drive(0.2, distance - 10, 100)
                            self.state = 7
                        else:
                            self.state = 5

        # wait until close, then lower/open claw
        elif self.state == 7:
            if not getattr(self.move, 'busy', False):
                self.publish_cmd_vel(50, 20)
                self.state = 8

        # when claw is close enough, stop and close claw
        elif self.state == 8:
            if self.claw_tof_distance < 30:
                self.publish_cmd_vel(0, 0)
                self.move.drive(100, 0, -100)  # pull victim back
                self.state = 9

        # wait for pull-back to finish, then count victim and return to search
        elif self.state == 9:
            if not getattr(self.move, 'busy', False):
                if self.target_type == 'silver':
                    self.silver_victims_collected += 1
                else:
                    self.black_victims_collected += 1

                self.target_type = None
                self.target_detection = None

                if self.silver_victims_collected < 2:
                    self.state = 3  # collect next silver
                else:
                    self.state = 100  # all silvers done, head for exit

        elif self.state == 99:
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
            pass

        elif self.state == 100: # exit
            kp = (60-self.side_tof_distance) * self.exit_kp # stay constant dist from left wall
            self.publish_cmd_vel(200, kp) # drive forward with proportional control based on distance to exit
            if self.side_tof_distance > 150:
                self.publish_cmd_vel(0, 0)
                self.get_logger().info('found potential exit')
                self.move.drive(0, 180, -100) # rotate left
                self.state = 101
    
        elif self.state == 101:
            if getattr(self.move, 'busy', False) == False:
                self.black_line_seen = False
                # enable down vision 
                self.publish_cmd_vel(90, 0) # drive forward
                if self.black_line_seen:
                    self.publish_cmd_vel(0, 0)
                    self.get_logger().info('Black line seen, exit confirmed')
                    self.move.drive(100, 0, 100) # drive out of exit
                    self.state = 102

        elif self.state == 102:
            pass
        # DEACTIVATE


def main(args=None):
    rclpy.init(args=args)
    node = Rescue()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()