from setuptools import setup, find_packages

package_name = 'ros_robot_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, f'{package_name}.ros_robot_controller_sdk'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/controller.launch.py']),
         ('share/' + package_name + '/launch',
         ['launch/pid.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='renzo.damian.go@gmail.com',
    description='ROS 2 Robot Controller Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_node = ros_robot_controller.controller_node:main',
            'acker_lidar_controller = ros_robot_controller.acker_lidar_node:main',
            'follow_gap_node = ros_robot_controller.follow_gap_node:main',
            'color_detection_node = ros_robot_controller.color_detection_node:main',
            'reactive_follow_gap_node = ros_robot_controller.reactive_follow_gap_node:main'
        ],
    },
)
