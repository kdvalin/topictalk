import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
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

	return LaunchDescription([
		num_topics,
		num_publishers,
		num_subscriptions,
		Node(package="topictalk", executable="topictalk_publisher"),
		Node(package="topictalk", executable="topictalk_listener")
	])
