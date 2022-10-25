from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os

def generate_launch_description():

    tf_world_to_drone = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "1", "0", "-0.7", "0.7", "world", "drone"]
    )

    tf_drone_to_iwr = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0.05", "0", "0", "0", "drone", "iwr6843_frame"]
    )

    tf_test_node = Node(
        package="tf_test",
        executable="tf_test"
    )

    return LaunchDescription([
        tf_world_to_drone,
        tf_drone_to_iwr,
        tf_test_node
    ])
