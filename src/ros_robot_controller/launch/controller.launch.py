from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('ros_robot_controller')
    config_file = os.path.join(pkg_share, 'config', 'controller_params.yaml')

    return LaunchDescription([
        Node(
            package='ros_robot_controller',
            executable='controller_node',
            name='ros_robot_controller',
            output='screen',
            parameters=[config_file]
        )
    ])