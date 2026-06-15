from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():
	use_sim_time = LaunchConfiguration('use_sim_time')

	return LaunchDescription([
		DeclareLaunchArgument(
			'use_sim_time',
			default_value='false',
			description='Use simulated clock if true',
		),
		LifecycleNode(
			package='rescue_state_bc',
			executable='rescue_node',
			name='rescue_node',
			output='screen',
			parameters=[{'use_sim_time': use_sim_time}],
        ),
		Node(
			package='rescue_state_bc',
			executable='front_vision_node',
			name='front_vision_node',
			output='screen',
			parameters=[{'use_sim_time': use_sim_time}],
        ),
		Node(
			package='rescue_state_bc',
			executable='down_vision_node',
			name='down_vision_node',
			output='screen',
			parameters=[{'use_sim_time': use_sim_time}],
        ),
	])
