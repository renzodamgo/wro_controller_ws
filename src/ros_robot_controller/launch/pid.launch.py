
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
  peripherals_dir = get_package_share_directory('peripherals')
  ros_control_dir = get_package_share_directory('ros_robot_controller')

  lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_dir, 'launch', 'lidar.launch.py')
        
    )
  )
  
  controller_node = Node(
    package='ros_robot_controller',
    executable='controller_node',
    name='controler_node',
    output='screen'
  )

  acker_node = Node(
    package='ros_robot_controller',
    executable='acker_lidar_controller',
    name='acker_node',
    output='screen'
  )

  return([
    controller_node,
    lidar_launch,
    acker_node
  ])


