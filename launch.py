import os, sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch_ros.actions import Node

def get_arg(name: str, default: str) -> str:
	for arg in sys.argv:
		if arg.startswith(f"{name}:="):
			return arg.partition(":=")[2]
	return default

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
	ld = LaunchDescription()
	num_topics = int(get_arg("topics", "1"))
	num_publishers = int(get_arg("pubs", "1"))
	num_subscriptions = int(get_arg("subs", "1"))
	interval = float(get_arg("interval", "0.5"))

	block_size = LaunchConfiguration("bs", default=TextSubstitution(text="4096"))
	human_readable = LaunchConfiguration("human", default=TextSubstitution(text="true"))

	for i in range(num_topics):
		for j in range(num_publishers):
			ld.add_entity(Node(
				package="topictalk",
				executable="topictalk_publisher",
				parameters=[{"msg_length": block_size, "msg_interval": interval, "human_read": human_readable}],
				namespace=f"topictalk_{i}",
				name=f"topictalk_pub_{j}"
			))
		for j in range(num_subscriptions):
			ld.add_entity(Node(
				package="topictalk",
				executable="topictalk_listener",
				namespace=f"topictalk_{i}",
				name=f"topictalk_sub_{j}"
			))

	return ld