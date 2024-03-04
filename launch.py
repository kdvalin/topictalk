import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
	num_topics = DeclareLaunchArgument(
		"ntopics", default_value=TextSubstitution(text="1")
	)
	num_publishers = DeclareLaunchArgument(
		"npubs", default_value=TextSubstitution(text="1")
	)
	num_subscriptions = DeclareLaunchArgument(
		"nsubs", default_value=TextSubstitution(text="1")
	)
	block_size = DeclareLaunchArgument(
		"bs", default_value=TextSubstitution(text="4096")
	)

	return LaunchDescription([
		num_topics,
		num_publishers,
		num_subscriptions,
		Node(package="topictalk", executable="topictalk_publisher", parameters=[{"msg_length": LaunchConfiguration("bs")}]),
		Node(package="topictalk", executable="topictalk_listener")
	])
