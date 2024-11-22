## Initial Setup

First, navigate to the workspace and build:

```bash
cd wro_controller_ws
colcon build
source install/setup.bash
```

## Configure Device Permissions

Set permissions for USB devices:

```bash
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyACM0
```

## Launch Core Components

Start the components in separate terminals in this order:

**1. Launch LIDAR**
```bash
ros2 launch peripherals lidar.launch.py
```

**2. Start Controller Node**
```bash
ros2 run ros_robot_controller controller_node
```

**3. Initialize PID Controller**
```bash
ros2 run ros_robot_controller acker_lidar_controller
```

## Activate Motors

To activate the rear motors, publish the following command:

```bash
ros2 topic pub /ros_robot_controller/set_motor ros_robot_controller_msgs/msg/MotorsState "{data: [{id: 1, rps: 0}, {id: 2, rps: -0.7}, {id: 3, rps: 0}, {id: 4, rps: 0.7}]}"
```
