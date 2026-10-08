from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory
import os


# Silencia info/debug/WARN de un nodo (solo deja pasar ERROR y FATAL) y manda su
# stdout/stderr al log en vez de a la pantalla.
# - 'WARN'  -> deja pasar warnings (por eso el led seguía spameando)
# - 'ERROR' -> oculta INFO y WARN, deja errores  (RECOMENDADO)
# - 'FATAL' -> silencio casi total (ojo: ocultarás errores reales)
QUIET = ['--ros-args', '--log-level', 'ERROR']


def generate_launch_description():
    # --- Paquetes ---
    controller_pkg = get_package_share_directory('ros_robot_controller')
    peripherals_pkg = get_package_share_directory('peripherals')

    # --- 1) Controlador (NO es el principal -> silenciado) ---
    controller_params = os.path.join(controller_pkg, 'config', 'controller_params.yaml')
    controller_node = Node(
        package='ros_robot_controller',
        executable='controller_node',
        name='ros_robot_controller',
        output='log',
        arguments=QUIET,
        parameters=[controller_params],
    )

    # --- 2) Acciones diferidas (tras arrancar el controlador) ---
    # LIDAR (otro launch). OJO: sus nodos se configuran DENTRO de lidar.launch.py,
    # no desde aquí. Si imprime, hay que silenciarlo en ese archivo.
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_pkg, 'launch', 'lidar.launch.py')
        )
    )

    # === NODO PRINCIPAL: el ÚNICO con salida en pantalla ===
    acker_node_delayed = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_robot_controller',
                executable='acker_lidar_node',
                name='acker_node',
                output='screen',   # <-- este sí lo vemos
            )
        ]
    )

    # usb_cam publisher (silenciado)
    camera_node_delayed = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='peripherals',
                executable='camera_publisher',
                name='usb_cam',
                output='log',
                arguments=QUIET,
            )
        ]
    )

    # Cámara / procesamiento (silenciado)
    camara_node_delayed = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='ros_robot_controller',
                executable='camara',
                name='camara_node',
                output='log',
                arguments=QUIET,
            )
        ]
    )

    # motor (silenciado)
    motor_node_delayed = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='ros_robot_controller',
                executable='motor',
                name='motor_node',
                output='log',
                arguments=QUIET,
            )
        ]
    )

    # LED (silenciado)
    led_node_delayed = TimerAction(
        period=11.0,
        actions=[
            Node(
                package='ros_robot_controller',
                executable='led',
                name='led_node',
                output='log',
                arguments=QUIET,
            )
        ]
    )

    # --- Agrupamos los que se lanzan tras el controlador ---
    after_controller_starts = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_node,
            on_start=[
                lidar_launch,
                acker_node_delayed,
                camera_node_delayed,
                camara_node_delayed,
                motor_node_delayed,
                led_node_delayed,
            ]
        )
    )

    # --- Lanzamiento final ---
    return LaunchDescription([
        controller_node,
        after_controller_starts,
    ])