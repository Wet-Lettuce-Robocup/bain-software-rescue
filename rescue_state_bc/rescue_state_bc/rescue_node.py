# basic_lifecycle_node.py
import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from lifecycle_msgs.msg import Transition

class Rescue(LifecycleNode):
    def __init__(self):
        super().__init__('rescue_node')

    def on_configure(self, state):
        self.get_logger().info('Configuring...')
        # setup here
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('Activating...')
        # enable publishers and timers
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.get_logger().info('Deactivating...')
        # stop timers
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.get_logger().info('Cleaning up...')
        # cleanup
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self.get_logger().info('Shutting down...')
        # more cleanup?
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state):
        self.get_logger().error('Error state entered')
        # error state
        return TransitionCallbackReturn.SUCCESS