import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch_ros.actions import Node

def declare_args():
	return [
		DeclareLaunchArgument(
			"ntopics", default_value=TextSubstitution(text="1")
		),
		DeclareLaunchArgument(
			"npubs", default_value=TextSubstitution(text="1")
		),
		DeclareLaunchArgument(
			"nsubs", default_value=TextSubstitution(text="1")
		),
		DeclareLaunchArgument(
			"nsubs", default_value=TextSubstitution(text="1")
		),
		DeclareLaunchArgument(
			"bs", default_value=TextSubstitution(text="4096")
		),
	]

def generate_launch_description():
	num_topics = LaunchConfiguration(
		"ntopics", default=TextSubstitution(text="1")
	)
	num_publishers = LaunchConfiguration(
		"npubs", default=TextSubstitution(text="1")
	)
	num_subscriptions = LaunchConfiguration(
		"nsubs", default=TextSubstitution(text="1")
	)
	block_size = LaunchConfiguration("bs", default=TextSubstitution(text="4096"))

	return LaunchDescription([
		Node(package="topictalk", executable="topictalk_publisher", parameters=[{"msg_length": block_size }]),
		Node(package="topictalk", executable="topictalk_listener")
	])
