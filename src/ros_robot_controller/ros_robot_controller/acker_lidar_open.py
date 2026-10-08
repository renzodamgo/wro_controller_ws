import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from ros_robot_controller_msgs.msg import SetAckerServoState, ButtonState
import numpy as np
import time

####################OPENNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN
class AckerLidarController(Node):
    def __init__(self):
        super().__init__('acker_lidar_node')

        # Publishers
        self.acker_publisher = self.create_publisher(
            SetAckerServoState,
            '/ros_robot_controller/acker_servo/set_state',
            10
        )
        self.vel_pub = self.create_publisher(Float32, '/motor_vel', 10)

        # Subscribers
        self.scan_subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.button_subscription = self.create_subscription(
            ButtonState, '/ros_robot_controller/button', self.button_callback, 10
        )
        self.obst_subscription = self.create_subscription(
            String, '/obstaculos', self.obst_callback, 10
        )

        # Tiempo en segundos
        self.max_time = 80
        #self.max_time = 60

        # Velocidades
        #self.speedMotor = 100.0
        self.speedMotor = 80.0

        # Uso de cámazrtra
        self.camera = False

        # self.camera = True

        # Servo positions
        self.center_position = 1500
        self.max_position = 2040
        self.min_position = 1010
        self.steering_backup = 1500

        # PID constants
        self.Kp = 200.0
        self.Ki = 0.0
        self.Kd = 35.0

        # PID variables
        self.previous_error = 0.0
        self.integral = 0.0
        self.current_error = 0.0

        # Sesgo de setpoint por obstáculo
        self.declare_parameter('sp_step', 0.00)  # [m]
        self.sp_step = float(self.get_parameter('sp_step').value)
        self.sp_diff = 0.0  # 0 = centrado
        self.front_distance = 0.0
        self.left_avg = 0.0
        self.right_avg = 0.0
        # Estado
        self.button_pressed = False
        self.time = 0

        # Timers
        self.create_timer(0.1, self.control_loop)  # 10 Hz
        self.get_logger().info('Acker Lidar Controller started')
        self.create_timer(1.0, self.timer_loop)
        self.get_logger().info('Timer started')

    # ----------------- Callbacks -----------------
    def obst_callback(self, msg: String):
        ok = self.camera
        data = msg.data.strip().lower()
        if data == 'verde' and ok:
            self.sp_diff = +self.sp_step   # sesgo a la izquierda
        elif data == 'rojo' and ok:
            self.sp_diff = -self.sp_step   # sesgo a la derecha
        else:
            self.sp_diff = 0.0             # centrado

    def timer_loop(self):
        if self.button_pressed:
            self.time += 1
            if self.time == self.max_time:
                self.stop_motors()
                self.button_pressed=False

    def send_servo_command(self, position, duration):
        msg = SetAckerServoState()
        msg.position = int(position)
        msg.duration = float(duration)
        self.acker_publisher.publish(msg)

    def button_callback(self, msg):
        self.button_pressed = True

    def stop_motors(self):
        self.vel_pub.publish(Float32(data=0.0))
        self.get_logger().info('Stopping motor (duty=0%)')

    def scan_callback(self, msg: LaserScan):
        if self.button_pressed:
            try:
                total_angles = len(msg.ranges)

                # --- IZQUIERDA ---
                left_center = int(total_angles * (240 / 360))
                left_start = max(0, left_center - 7)
                left_end = min(total_angles, left_center + 7)

                # --- DERECHA ---
                right_center = int(total_angles * (120 / 360))
                right_start = max(0, right_center - 7)
                right_end = min(total_angles, right_center + 7)

                # --- FRENTE ---
                front_center = int(total_angles * (0 / 360))   # 0° → frente
                front_start = max(0, front_center - 7)
                front_end = min(total_angles, front_center + 7)

                # --- OBTENER DISTANCIAS ---
                left_distances = msg.ranges[left_start:left_end]
                right_distances = msg.ranges[right_start:right_end]
                front_distances = msg.ranges[front_start:front_end]

                # --- FILTRAR VALORES VÁLIDOS ---
                valid_left = [d for d in left_distances if not np.isnan(d) and msg.range_min < d < msg.range_max]
                valid_right = [d for d in right_distances if not np.isnan(d) and msg.range_min < d < msg.range_max]
                valid_front = [d for d in front_distances if not np.isnan(d) and msg.range_min < d < msg.range_max]

                if len(valid_left) == 0 or len(valid_right) == 0:
                    self.get_logger().warn('No valid readings')
                    #self.current_error = 0.0
                    #self.front_distance = 0.0
                    return

                self.left_avg = sum(valid_left) / len(valid_left)
                self.right_avg = sum(valid_right) / len(valid_right)

                if len(valid_front) > 0:
                    self.front_distance = sum(valid_front) / len(valid_front)
                else:
                    self.front_distance = 0.0

                # --- CALCULAR ERROR ---
                self.current_error = (self.right_avg - self.left_avg) - self.sp_diff
                self.get_logger().error(f'{self.current_error}')

            except Exception as e:
                self.get_logger().error(f'Error in scan_callback: {e}')
                self.current_error = 0.0
                self.front_distance = 0.0

    def pid_control(self):
        try:
            proportional = self.Kp * self.current_error
            self.integral += self.Ki * self.current_error
            derivative = self.Kd * (self.current_error - self.previous_error)
            output = proportional + self.integral + derivative
            self.previous_error = self.current_error
            return output
        except Exception as e:
            self.get_logger().error(f'Error in pid_control: {str(e)}')
            return 0.0

    def control_loop(self):
        if self.button_pressed:
            try:
                steering = self.pid_control()
                if np.isnan(steering):
                    position = self.steering_backup
                else:
                    position = int(self.center_position - steering)
                    position = min(max(position, self.min_position), self.max_position)
                    self.steering_backup = position

                if False:
                    if self.left_avg < self.right_avg:
                        self.send_servo_command(self.min_position, 0.1)
                    else:
                        self.send_servo_command(self.max_position, 0.1)
                else:
                    self.send_servo_command(position, 0.1)

                self.vel_pub.publish(Float32(data=self.speedMotor))

            except Exception as e:
                self.get_logger().error(f'Error in control_loop: {str(e)}')
                try:
                    position = self.center_position
                    self.send_servo_command(position, 0.1)
                except Exception:
                    pass
        else:
            self.vel_pub.publish(Float32(data=0.0))


def main(args=None):
    rclpy.init(args=args)
    try:
        controller = AckerLidarController()
        rclpy.spin(controller)
    except Exception as e:
        print(f'Error in main: {e}')
    finally:
        try:
            msg = SetAckerServoState()
            msg.position = 1200
            msg.duration = 0.2
            controller.acker_publisher.publish(msg)
            controller.vel_pub.publish(Float32(data=0.0))
        except Exception:
            pass
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
