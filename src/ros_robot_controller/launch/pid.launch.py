from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
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
    
    #controller_node = TimerAction(
    #    period=5.0,  # delay in seconds
    #    actions=[
    #        Node(
    #            package='ros_robot_controller',
    #            executable='controller_node',
    #            name='controller_node',
    #            output='screen'
    #        )
    #    ]
    #)

    acker_node = TimerAction(
        period=5.0,  # delay in seconds
        actions=[
            Node(
                package='ros_robot_controller',
                executable='acker_lidar_node',
                name='acker_node',
                output='screen'
            )
        ]
    )
    
    camera_node = TimerAction(
        period=8.0,  # delay in seconds
        actions=[
            Node(
                package='peripherals',
                executable='camera_publisher',
                name='camera_node',
                output='screen'
            )
        ]
    )
    
#    pose_node = TimerAction(
 #       period=12.0,  # delay in seconds
  #      actions=[
   #         Node(
    #            package='peripherals',
     #           executable='pose_estimator',
      #          name='pose_node',
       #         output='screen'
        #    )
       # ]
   # )

    return LaunchDescription([
        #controller_node,
        lidar_launch,
        acker_node,
        camera_node
    ])

