import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from lifecycle_msgs.msg import Transition
from enum import Enum, auto
from robot_msgs.msg import Detections, LEDCommand
from std_msgs.msg import Int32, Bool, ColorRGBA, Float32
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
        self.move_client = ActionClient(node, MoveTime, "move_time")
        self.busy = False
        self.current_angle = 0.0
        self.distance_travelled = 0.0
        self.angle_turned = 0.0
        self._last_goal_vel = 0.0
        self._last_goal_angular_vel = 0.0
        self._last_goal_time = 0.0
        self._sequence = None
        self._on_complete = None

    def drive(self, distance, angle=0, velocity=0.1):
        linear_time = abs(distance) / abs(velocity) if velocity != 0 and distance != 0 else 0.0
        angular_time = abs(angle) / abs(velocity) if velocity != 0 and angle != 0 else 0.0
        time_required = max(linear_time, angular_time)

        if time_required <= 0.0:
            self.node.get_logger().warn('Drive called with zero distance and angle; ignoring')
            return

        goal = MoveTime.Goal()
        goal.time = float(time_required)
        linear_vel = math.copysign(velocity, distance) if distance != 0 else 0.0
        angular_vel = math.copysign(velocity, angle) if angle != 0 else 0.0
        goal.vel = float(linear_vel)
        goal.angular_vel = float(angular_vel)
        self._last_goal_vel = float(linear_vel)
        self._last_goal_angular_vel = float(angular_vel)
        self._last_goal_time = float(time_required)
        self.busy = True

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
            self.send_goal_future = self.move_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
            self.send_goal_future.add_done_callback(self.goal_response_callback)
        except Exception as e:
            self.node.get_logger().error(f'Failed to send goal: {e}')
            self.busy = False
            return

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        time_elapsed = getattr(feedback, 'time_elapsed', None)
        if time_elapsed is not None:
            te = float(time_elapsed)
            te_sec = te * 1e-9 if te > 1e6 else te
            self.distance_travelled = self._last_goal_vel * te_sec
            self.angle_turned = self._last_goal_angular_vel * te_sec
        else:
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
            self.node.get_logger().info(f'Movement Goal result: {result}')

        self.busy = False
        if self._on_complete is not None:
            self._on_complete()

    def _advance_sequence(self):
        if self._sequence is None:
            return
        try:
            next(self._sequence)
        except StopIteration:
            self._sequence = None
            self._on_complete = None

    def run_sequence(self, sequence_gen):
        self._sequence = sequence_gen
        self._on_complete = self._advance_sequence
        self._advance_sequence()


class Rescue(LifecycleNode):
    detected_objects = []
    dist_scan_samples = []
    silver_victims_collected = 0
    black_victims_collected = 0
    robot_position = (0, 0)
    current_angle = 0
    latest_map = None
    exit_kp = 0.5
    black_line_seen = False
    search_step = 0
    rad_to_turn = 90
    m_to_dist = 530

    def __init__(self):
        super().__init__('rescue_node')
        self.state = 1
        self._last_state = None
        self.control_timer = None
        self.sensor_offset = 0.1
        self._last_sample_angle = None
        self.tof_distance = float('inf')
        self.move = None
        self.dist_scan_samples = []
        self.led_cmd_pub = None
        self._detection_future = None

    def on_configure(self, state):
        self.get_logger().info('Configuring Rescue Node...')
        self.move = Movement(self)
        self.front_tof_subscriber = self.create_subscription(Int32, '/tof/front', self.front_tof_callback, 10)
        self.claw_tof_subscriber = self.create_subscription(Int32, '/tof/claw', self.claw_tof_callback, 10)
        self.side_tof_subscriber = self.create_subscription(Int32, '/tof/side', self.side_tof_callback, 10)
        self.black_line_subscriber = self.create_subscription(Bool, '/black_present', self.black_line_callback, 10)

        self.ball_client = self.create_client(Inference, '/ml_rescue/detections')

        self.front_vision_enable_pub = self.create_publisher(Bool, '/front_vision_enable', 10)
        self.down_vision_enable_pub = self.create_publisher(Bool, '/down_vision_enable', 10)
        self.drive_pub = self.create_publisher(Int32, '/drive_command', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.led_cmd_pub = self.create_publisher(LEDCommand, 'led_command', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.claw_pub = self.create_publisher(Float32, '/servo/grab', 10) # 0.5 is open, 1 is closed
        self.lift_pub = self.create_publisher(Float32, '/servo/lift', 10) # up is 2.5, down is 0.2
        self.gate_pub = self.create_publisher(Float32, '/servo/gate', 10) # open is 2.3, closed is 0.8

        self.front_tof_distance = 999999
        self.claw_tof_distance = 999999
        self.side_tof_distance = 999999
        self.get_logger().info('rescue_node configured!!')

        # start down vision
        self.down_vision_enable_pub.publish(Bool(data=True))
        self.get_logger().info('Down vision enabled')

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('Activating...')
        self.control_timer = self.create_timer(0.05, self.rescue_control_loop)
        self.move.move_client.wait_for_server()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.get_logger().info('Deactivating...')
        if self.control_timer is not None:
            try:
                self.destroy_timer(self.control_timer)
            except Exception:
                pass
            self.control_timer = None
        return TransitionCallbackReturn.SUCCESS

    def _request_detections(self):
        if self._detection_future is not None:
            return  # already in flight or done but not yet consumed
        req = Inference.Request()
        req.message = 'whereball'
        self._detection_future = self.ball_client.call_async(req)
        self.get_logger().info('Whereball request sent')

    def _detections_ready(self):
        if self._detection_future is None:
            return False  # remove the noisy log
        if not self._detection_future.done():
            return False  # remove the noisy log
        
        try:
            response = self._detection_future.result()
        except Exception as e:
            self.get_logger().error(f'Inference service call failed: {e}')
            self._detection_future = None
            return False
        self._detection_future = None

        if not response.success:
            self.get_logger().info('Inference service returned nothing')
            self.detected_objects = []
            return True

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
        self.get_logger().info(f'Detections ready: {len(self.detected_objects)} objects')
        return True

    def black_line_callback(self, msg):
        if msg.data:
            self.black_line_seen = True

    def front_tof_callback(self, msg):
        try:
            self.front_tof_distance = float(msg.data) / 1000.0
        except Exception:
            self.front_tof_distance = None

    def claw_tof_callback(self, msg):
        try:
            self.claw_tof_distance = float(msg.data) / 1000.0
        except Exception:
            self.get_logger().error('Failed to parse claw TOF distance, WHAT IS GOING ON BRUHHHH')
            self.get_logger().error(f'Failed to parse claw TOF distance, msg: {msg} bruh')
            self.claw_tof_distance = None

    def side_tof_callback(self, msg):
        try:
            self.side_tof_distance = float(msg.data) / 1000.0
        except Exception:
            self.side_tof_distance = None

    def face_bearing(self, bearing: float, rotate_vel=0.1):
        current = self.move.current_angle
        diff = (bearing - current + math.pi) % (2 * math.pi) - math.pi
        self.move.drive(0, diff, rotate_vel)
        self.move.current_angle = (self.move.current_angle + diff) % (2 * math.pi)

    def rotate(self, distance: float, angle: float, rotate_vel=0.1):
        self.move.drive(distance, angle, rotate_vel)
        self.move.current_angle = (self.move.current_angle + angle) % (2 * math.pi)

    def _publish_led_for_state(self, state) -> None:
        if self.led_cmd_pub is None:
            return
        mapping = {
            1:  (0.0, 0.0, 1.0, 1.0),
            2:  (0.0, 1.0, 0.0, 1.0),
            3:  (1.0, 1.0, 0.0, 1.0),
            4:  (1.0, 0.0, 1.0, 1.0),
            5:  (0.0, 1.0, 1.0, 1.0),
            6:  (1.0, 0.5, 0.0, 1.0),
            7:  (0.5, 0.0, 0.5, 1.0),
            8:  (1.0, 1.0, 1.0, 1.0),
            9:  (0.5, 1.0, 0.0, 1.0),
            10: (1.0, 0.0, 0.0, 1.0),
            11: (0.0, 0.0, 0.0, 1.0),
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
            twist.linear.x = float(linear_x)
            twist.angular.z = float(angular_z)
            self.cmd_vel_pub.publish(twist)

    def _find_target(self, target_type):
        for obj in self.detected_objects:
            if obj.get('type') == target_type:
                return obj
        return None

    def _current_target_type(self):
        return 'silver' if self.silver_victims_collected < 2 else 'black'
    
    def _wait(self, duration, next_state):
        self.get_logger().info(f'Waiting for {duration} seconds before transitioning to state {next_state}')
        self.wait_until = self.get_clock().now() + rclpy.duration.Duration(seconds=duration)
        self.next_state_after_wait = next_state
        self.state = 99
    
    def lift(self, position):
        if position == 'up':
            self.get_logger().info('Lift raised')
            self.lift_pub.publish(Float32(data=2.7))
        elif position == 'down':
            self.get_logger().info('Lift lowered')
            self.lift_pub.publish(Float32(data=0.4))
    
    def grab(self, position):
        if position == 'open':
            self.get_logger().info('Claw opened')
            self.claw_pub.publish(Float32(data=0.5))
        elif position == 'close':
            self.get_logger().info('Claw closed')
            self.claw_pub.publish(Float32(data=1.0))

    def gate(self, position):
        if position == 'open':
            self.get_logger().info('Gate opened')
            self.gate_pub.publish(Float32(data=2.3))
        elif position == 'close':
            self.get_logger().info('Gate closed')
            self.gate_pub.publish(Float32(data=0.8)) # UPDATE

    def rescue_control_loop(self):
        if getattr(self, '_last_state', None) != self.state:
            try:
                self._publish_led_for_state(self.state)
            except Exception as e:
                self.get_logger().error(f'Failed to publish LED for state {self.state}: {e}')
            self._last_state = self.state

        # move in
        if self.state == 1:
            self.move.drive(140, 0, 100)
            self.get_logger().info('state 2 rescue')
            self.grab('close')
            self.lift('up')
            self.turns = 0
            self.state = 2

        # wait for drive in to finish then rotate right
        elif self.state == 2:
            self.lift('up')
            self.grab('close')
            # only goes back after two silver collected
            if not getattr(self.move, 'busy', False):
                if self.search_step == 0: # turn right
                    self.move.drive(0, 210, 100)
                elif self.search_step == 1: # straight
                    self.move.drive(0, 210, 100)
                elif self.search_step == 2: # left 
                    self.move.drive(0, 210, 100)
                elif self.search_step == 3: # back  
                    self.move.drive(0, 210, 100)
                self.turns += 1
                self._wait(5, 5)

      # search for the current target
        elif self.state == 3:
            if not getattr(self.move, 'busy', False):
                if self._detection_future is None:
                    self.current_target = self._current_target_type()
                    self._request_detections()
                    self.state = 4

        elif self.state == 4:
            if not getattr(self.move, 'busy', False):
                if self._detections_ready():
                    target = self._find_target(self.current_target)
                    if target is not None:
                        self.get_logger().info(f'"{self.current_target}" detected after "{self.turns}" turns, aligning...')
                        self.target_type = self.current_target
                        self.target_detection = target
                        self.state = 6
                    else:
                        self.search_step += 1
                        self.get_logger().info(f'No "{self.current_target}" found, rotating...')

                        if self.silver_victims_collected >= 2:
                            max_steps = 4 
                        else:
                            max_steps = 3
                            
                        if self.search_step >= max_steps:
                            self.search_step = 0 # loop back to 0 after reaching end of cycle
                            self.move.drive(0, 210, 100)
                        
                        self.state = 2

        elif self.state == 5:
            if not getattr(self.move, 'busy', False):
                self.state = 3

        # align to target bearing using stored detection
        elif self.state == 6:
            if not getattr(self.move, 'busy', False):
                bearing = self.target_detection['bearing']
                self.move.drive(0, bearing * self.rad_to_turn, 100)
                self.get_logger().info(f'Aligning to target bearing: {bearing*self.rad_to_turn} rad*mult')
                self._wait(4, 61)

        elif self.state == 61:
            if not getattr(self.move, 'busy', False):
                if self._detection_future is None:
                    self.get_logger().info(f'Re-requesting detection for "{self.current_target}" after first alignment...')
                    self._request_detections()
                    self.state = 62

        elif self.state == 62:
            if not getattr(self.move, 'busy', False):
                if self._detections_ready():
                    target = self._find_target(self.current_target)
                    if target is not None:
                        self.target_detection = target
                        bearing = target['bearing']
                        self.move.drive(0, bearing * self.rad_to_turn, 100)
                        self.get_logger().info(f'Second alignment to target bearing: {bearing*self.rad_to_turn:.3f} rad')
                        self._wait(3, 7)
                    else:
                        # lost the target after first alignment rotate and search again
                        self.get_logger().warn(f'Lost "{self.current_target}" during re-detection, returning to search...')
                        self.search_step = 0
                        self.state = 2

        elif self.state == 7:
            if not getattr(self.move, 'busy', False):
                distance = self.target_detection['distance']
                self.move.drive(distance * self.m_to_dist - 40, 0, 100)
                self.get_logger().info(f'Moving towards target, distance: {distance*self.m_to_dist} mm')
                self._wait(6, 71)

        elif self.state == 71:
            if not getattr(self.move, 'busy', False):
                self.get_logger().info(f'Arrived at target, distance to target: {self.claw_tof_distance:.3f} m')
                self._request_detections()
                self.state = 72

        elif self.state == 72:
            if not getattr(self.move, 'busy', False):
                if self._detections_ready():
                    target = self._find_target(self.current_target)
                    if target is not None:
                        self.target_detection = target
                        self.get_logger().info(f'Re-Re-detected "{self.current_target}" at distance: {target["distance"]:.3f} m')
                        self.move.drive(target['distance'] * self.m_to_dist - 40, 0, 100)
                        self._wait(3, 73)
                    else:
                        self.get_logger().warn(f'Lost "{self.current_target}" during re-re-detection, returning to search...')
                        self.search_step = 0
                        self.state = 2

        elif self.state == 73:
            if not getattr(self.move, 'busy', False):
                self._request_detections()
                self.state = 74

        elif self.state == 74:
            if not getattr(self.move, 'busy', False):
                if self._detections_ready():
                    target = self._find_target(self.current_target)
                    if target is not None:
                        self.target_detection = target
                        self.get_logger().info(f'Re-Re-Re-detection of "{self.current_target}" at distance: {target["distance"]:.3f} m')
                        self.move.drive(0, target['bearing'] * self.rad_to_turn, 100)
                        self.get_logger().info(f'Re-Re-Re-Aligning to target bearing: {target["bearing"]*self.rad_to_turn:.3f} rad')
                        self.state = 8
                    else:
                        self.get_logger().warn(f'Lost "{self.current_target}" during re-re-re-detection, returning to search...')
                        self.search_step = 0
                        self.state = 2

        elif self.state == 8:
            if not getattr(self.move, 'busy', False):
                if self.target_type == 'silver' or self.target_type == 'black':
                    self.get_logger().info(f'Victim "{self.target_type}", proceeding with collection')
                    self.state = 9
                elif self.target_type == 'green' or self.target_type == 'red':
                    self.get_logger().info(f'Tray "{self.target_type}", approaching for deposit')
                    self.state = 15
                else:
                    self.get_logger().error(f'Unknown target type: {self.target_type}, how did we get here??')
                    self.state = 2

        # wait until close, then lower/open claw
        elif self.state == 9:
            self.lift('down')
            self.grab('open')
            self._wait(1, 91)

        elif self.state == 91:
            self.get_logger().info(f'Lowered lift and opened claw for victim "{self.target_type}"')
            if not getattr(self.move, 'busy', False):
                self.publish_cmd_vel(0.018, 0.013)
                self.tof_seen = self.get_clock().now()
                self.state = 10

        # when claw is close enough stop
        elif self.state == 10:
            if self.claw_tof_distance is None:
                self.get_logger().warn('Claw TOF distance is None, cannot proceed')
                self.state = 2
                return
            self.get_logger().info(f'Claw TOF distance: {self.claw_tof_distance:.3f} m')
            if self.claw_tof_distance < 0.03 and self.claw_tof_distance > 0.01:
                self.publish_cmd_vel(0, 0)
                self.move.drive(30, 0, 50)
                self.state = 11
            elif self.get_clock().now() - self.tof_seen > rclpy.duration.Duration(seconds=1):
                self.get_logger().warn(f'Could not see victim, canceling')
                self.publish_cmd_vel(0, 0)
                self.move.drive(-100, 0, 50)  # back up a bit
                self.state = 2

        elif self.state == 11:
            if not getattr(self.move, 'busy', False):
                self.grab('close')
                self.move.drive(-60, 0, 100)  # pull victim back
                self._wait(4, 12)

        elif self.state == 12:
            if not getattr(self.move, 'busy', False):
                self.lift('up')
                self._wait(4, 121)

        elif self.state == 121:
            if self.target_type == 'silver':
                self.grab('open')
            self.state = 13

        # count victim and return to search
        elif self.state == 13:
            if not getattr(self.move, 'busy', False):

                if self.target_type == 'silver':
                    self.silver_victims_collected += 1
                else:
                    self.black_victims_collected += 1

                self.target_type = None
                self.target_detection = None

                if self.silver_victims_collected >= 2 and self.black_victims_collected >= 1:
                    self.get_logger().info('All victims collected')
                    self.state = 14
                else:
                    self.state = 2

        # find trays
        elif self.state == 14:
            self.target_type = 'green'
            self.state = 2

        elif self.state == 15:
            if self.target_type == 'green':
                self.publish_cmd_vel(50, 0)
                if True: # WHEN CAMERA IS FULL GREEN CHANGE THIS
                    self.publish_cmd_vel(0, 0)
                    self.lift('up')
                    self.state = 16
            elif self.target_type == 'red':
                self.publish_cmd_vel(50, 0)
                self.grab('open')
                if True: # WHEN CAMERA IS FULL RED CHANGE THiS
                    self.publish_cmd_vel(0, 0)
                    self.lift('up')
                    self.state = 16

        elif self.state == 16:
            if not getattr(self.move, 'busy', False):
                self.move.drive(-100, 0, 100)  # back up from tray
                self.state = 17

        elif self.state == 17:
            if not getattr(self.move, 'busy', False):
                self.move.drive(0, 400, 100) # 360 such that back faces tray
                self.state = 18

        elif self.state == 18:
            if not getattr(self.move, 'busy', False):
                self.move.drive(-100, 0, 100) # reverse into tray
                self.state = 19

        elif self.state == 19:
            if not getattr(self.move, 'busy', False):
                self.gate('open') # release victims
                self._wait(3, 20)

        elif self.state == 20: 
            if not getattr(self.move, 'busy', False):
                self.move.drive(50, 0, 100) # move away from tray incase it climbs tray
                self.state = 21

        elif self.state == 21:
            if not getattr(self.move, 'busy', False):
                self.gate('close')
                self.target_type = 'red' # next going to red to drop black victim
                self._wait(3, 22) # delay: make sure gate is closed

        elif self.state == 22:
            if not getattr(self.move, 'busy', False):
                self.grab('open')
                self._wait(2, 1) # delay: make sure ball can be release from claw before it shuts again in state 1

        elif self.state == 100:  # exit
            kp = (60 - self.side_tof_distance) * self.exit_kp
            self.publish_cmd_vel(200, kp)
            if self.side_tof_distance > 150:
                self.publish_cmd_vel(0, 0)
                self.get_logger().info('found potential exit')
                self.move.drive(0, -180, 100) # rotate left
                self.state = 101
    
        elif self.state == 101:
            if getattr(self.move, 'busy', False) == False:
                self.black_line_seen = False
                # enable down vision 
                self.publish_cmd_vel(20, 0) # drive forward
                if self.black_line_seen:
                    self.publish_cmd_vel(0, 0)
                    self.get_logger().info('Black line seen, exit confirmed')
                    self.move.drive(100, 0, 100) # drive out of exit
                    self.state = 102

        elif self.state == 99:
            if getattr(self, 'wait_until', None) is not None and self.get_clock().now() >= self.wait_until:
                self.state = self.next_state_after_wait
                self.wait_until = None
                self.next_state_after_wait = None

def main(args=None):
    rclpy.init(args=args)
    node = Rescue()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()