from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    launch_description = LaunchDescription()

    nclt2ros2_node = Node(
        package="nclt2ros2",
        executable="nclt2rosbag2",
        output="screen",
    )

    launch_description.add_action(nclt2ros2_node)

    return launch_description
