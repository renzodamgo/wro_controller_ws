"""Sensores del coche para verlos en RViz desde otra PC (tira LED opcional).

    ros2 launch peripherals sensors.launch.py              # tira LED apagada
    ros2 launch peripherals sensors.launch.py leds:=true   # tira LED en blanco (20 %)

LiDAR LD19 (/scan), camara (/usb_cam/image_raw y /usb_cam/image_raw/compressed),
IMU de la placa (/ros_robot_controller/imu_raw), LiDAR con 0 rad adelante
(/robotek/scan, scan_adapter) y detector de pilares de la simulacion
(/obstacle/markers, /obstacle/debug_image/compressed; detector:=false lo quita).
Con servo:=true mueve SOLO el servo segun el pilar mas cercano (rojo: derecha,
verde: izquierda, nada: centro); el motor no se toca. Con leds:=true, tira LED. El detector y scan_adapter son de ~/Projects/WRO-Strategies:
    source ~/wro_controller_ws/install/setup.bash
    source ~/Projects/WRO-Strategies/install/setup.bash
No correr junto con wro.launch.py: usan los mismos puertos (detenerlo con
~/scripts/stop_ros.sh). En la PC usar el mismo ROS_DOMAIN_ID.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    peripherals_pkg = get_package_share_directory('peripherals')
    controller_pkg = get_package_share_directory('ros_robot_controller')

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_pkg, 'launch', 'lidar.launch.py')))

    camera = Node(
        package='peripherals',
        executable='camera_publisher',
        name='usb_cam',
        output='screen',
    )

    # IMU (y bateria, boton) de la placa RRC Lite.
    controller = Node(
        package='ros_robot_controller',
        executable='controller_node',
        name='ros_robot_controller',
        output='screen',
        parameters=[os.path.join(controller_pkg, 'config', 'controller_params.yaml')],
    )

    detector_on = IfCondition(LaunchConfiguration('detector'))

    # LD19: 0 rad atras, frente en pi, angulos antihorarios (como acker_lidar_node).
    scan_adapter = Node(
        package='robotek_hardware',
        executable='scan_adapter',
        name='scan_adapter',
        output='screen',
        # The LD19 only sees +-110 deg: behind, it hits the car itself at 2-8 cm.
        parameters=[{'angle_offset': math.pi, 'reverse': False, 'frame_id': 'lidar_link',
                     'fov_deg': 110.0, 'min_valid_range': 0.12}],
        remappings=[('scan_in', '/scan'), ('scan_out', '/robotek/scan')],
        condition=detector_on,
    )

    detector = Node(
        package='robotek_obstacle',
        executable='obstacle_detector',
        name='obstacle_detector',
        output='screen',
        remappings=[('/scan', '/robotek/scan'), ('/camera/image', '/usb_cam/image_raw')],
        # Real camera, measured 8 oct 2026 (robotek_common.vehicle REAL_CAMERA_*).
        parameters=[{'camera_hfov': 1.164, 'camera_pitch': 0.17, 'camera_yaw': 0.077}],
        condition=detector_on,
    )

    servo = Node(
        package='robotek_hardware',
        executable='servo_steer',
        name='servo_steer',
        output='screen',
        remappings=[('steer', '/obstacle/steer')],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('servo'), "' == 'true' and '",
            LaunchConfiguration('detector'), "' == 'true'"])),
    )

    leds = Node(
        package='peripherals',
        executable='led_light',
        name='led_light',
        output='screen',
        condition=IfCondition(LaunchConfiguration('leds')),
    )

    # Como wro.launch.py: LiDAR y camara despues de la placa.
    return LaunchDescription([
        DeclareLaunchArgument(
            'leds', default_value='false',
            description='true: enciende la tira LED en blanco'),
        DeclareLaunchArgument(
            'detector', default_value='true',
            description='false: sin scan_adapter ni detector de pilares'),
        DeclareLaunchArgument(
            'servo', default_value='false',
            description='true: el servo gira hacia el lado de paso del pilar mas cercano'),
        leds,
        controller,
        TimerAction(period=5.0, actions=[lidar]),
        TimerAction(period=7.0, actions=[camera]),
        TimerAction(period=8.0, actions=[scan_adapter, detector, servo]),
    ])
