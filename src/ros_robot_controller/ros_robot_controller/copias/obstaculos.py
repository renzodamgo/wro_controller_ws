import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from ros_robot_controller_msgs.msg import SetAckerServoState, ButtonState, BuzzerState
import numpy as np
import time


class AckerLidarController(Node):
    def __init__(self):
        super().__init__('acker_lidar_node')

        # Publishers
        self.ack_pub = self.create_publisher(
            SetAckerServoState,
            '/ros_robot_controller/acker_servo/set_state',
            10
        )
        self.vel_pub = self.create_publisher(Float32, '/motor_vel', 10)
        self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 10)

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(ButtonState, '/ros_robot_controller/button', self.button_callback, 10)
        self.create_subscription(String, '/obstaculos', self.obst_callback, 10)

        # Config general
        self.speedMotor = 60.0          # velocidad normal
        self.turn_speed = 60.0          # velocidad en evasión
        self.center_us = 1500           # centro del servo
        self.max_us = 2000              # tope izquierda
        self.min_us = 1050              # tope derecha

        # Estado de obstáculo por visión/color
        self.obstaculo = False          # hay obstáculo detectado por el nodo de color
        self.obstaculo_side = None      # "izquierda" / "derecha"

        # PID seguidor de CENTRO (error = left - right)
        self.Kp = 1200.0
        self.Kd = 400.0
        self.Ki = 4.0
        self.deriv_alpha = 0.4
        self.max_delta_us = 380.0
        self.us_deadzone = 1.0

        self.prev_error = 0.0
        self.deriv_filt = 0.0
        self.int_error = 0.0
        self.int_limit = 0.5
        self.last_t = time.time()

        # Estado sensores
        self.button_pressed = False
        self.left_avg = float('inf')
        self.right_avg = float('inf')
        self.front_distance = float('inf')

        # Lógica de evasión (máquina de estados)
        self.auto_turning = False
        self.evasion_state = "idle"      # "idle", "stop1", "turn", "forward", "stop2"
        self.evasion_end_time = 0.0
        self.turn_us = self.center_us

        # Duraciones de cada fase de evasión
        self.evasion_stop1 = 0.2    # detenerse antes de girar
        self.evasion_turn = 0.2     # tiempo solo girando Ackerman
        self.evasion_forward = 0.4  # avanzar un poco
        self.evasion_stop2 = 0.2    # detenerse y volver al centro

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Acker Lidar Controller listo (seguidor de centro + evasión con pausas).')

    # -------- Utilidades --------

    def beep(self, freq, duration=0.2, repeat=1):
        msg = BuzzerState()
        msg.freq = int(freq)
        msg.on_time = float(duration)
        msg.off_time = 0.0
        msg.repeat = int(repeat)
        # Descomenta si quieres realmente usar el buzzer:
        self.buzzer_pub.publish(msg)

    @staticmethod
    def clamp(x, a, b):
        return max(a, min(b, x))

    def send_servo(self, position_us, duration=0.1):
        msg = SetAckerServoState()
        msg.position = int(self.clamp(position_us, self.min_us, self.max_us))
        msg.duration = float(duration)
        self.ack_pub.publish(msg)

    def stop(self):
        self.vel_pub.publish(Float32(data=0.0))

    # -------- Callbacks --------

    def button_callback(self, _):
        self.button_pressed = True

    def obst_callback(self, msg: String):
        """
        Mensajes del nodo de visión/color:
        - 'derecha'   -> evasión hacia DERECHA
        - 'izquierda' -> evasión hacia IZQUIERDA
        - cualquier otro texto -> sin obstáculo por color
        """
        txt = msg.data.strip().lower()

        if txt == "derecha":
            self.obstaculo = True
            self.obstaculo_side = "derecha"
            self.get_logger().info('Obstáculo por color: DERECHA.')
            self.beep(500, 0.15, 1)
        elif txt == "izquierda":
            self.obstaculo = True
            self.obstaculo_side = "izquierda"
            self.get_logger().info('Obstáculo por color: IZQUIERDA.')
            self.beep(800, 0.15, 1)
        else:
            # Sin obstáculo por color
            self.beep(900,0,0)
            self.obstaculo = False
            self.obstaculo_side = None

    def scan_callback(self, msg: LaserScan):
        try:
            n = len(msg.ranges)
            idx_left = int(n * 240 / 360)
            idx_right = int(n * 120 / 360)
            idx_front = int(n * 180 / 360)
            w = 12

            def clean_slice(center):
                s = max(0, center - w)
                e = min(n, center + w)
                return [
                    d for d in msg.ranges[s:e]
                    if msg.range_min < d < msg.range_max
                ]

            left = clean_slice(idx_left)
            right = clean_slice(idx_right)
            front = clean_slice(idx_front)

            self.left_avg = float(np.median(left)) if left else float('inf')
            self.right_avg = float(np.median(right)) if right else float('inf')
            self.front_distance = float(np.median(front)) if front else float('inf')

        except Exception as e:
            self.get_logger().error(f'scan_callback error: {e}')

    # -------- Lazo de control principal --------

    def control_loop(self):
        if not self.button_pressed:
            self.stop()
            return

        now = time.time()
        dt = max(now - self.last_t, 1e-3)
        self.last_t = now

        # 1) Gestionar máquina de estados de EVASIÓN si está activa
        if self.auto_turning:
            if self.evasion_state == "stop1":
                # Fase 1: detenerse antes de girar
                self.stop()
                self.send_servo(self.center_us)
                if now < self.evasion_end_time:
                    return
                # pasar a fase de giro
                self.evasion_state = "turn"
                self.evasion_end_time = now + self.evasion_turn
                return

            elif self.evasion_state == "turn":
                # Fase 2: girar Ackerman (sin avanzar)
                self.stop()
                self.send_servo(self.turn_us)
                if now < self.evasion_end_time:
                    return
                # pasar a fase de avance
                self.evasion_state = "forward"
                self.evasion_end_time = now + self.evasion_forward
                return

            elif self.evasion_state == "forward":
                # Fase 3: avanzar con Ackerman girado
                self.send_servo(self.turn_us)
                self.vel_pub.publish(Float32(data=self.turn_speed))
                if now < self.evasion_end_time:
                    return
                # pasar a fase de stop2
                self.evasion_state = "stop2"
                self.evasion_end_time = now + self.evasion_stop2
                # al entrar, ya podemos ir regresando al centro
                self.send_servo(self.center_us)
                self.stop()
                return

            elif self.evasion_state == "stop2":
                # Fase 4: detenerse y recentrar Ackerman
                #self.send_servo(self.center_us)
                self.stop()
                if now < self.evasion_end_time:
                    return
                # Fin de evasión: limpiar estados y volver al seguidor de centro
                self.auto_turning = False
                self.evasion_state = "idle"
                self.obstaculo = False
                self.obstaculo_side = None
                # reset PID
                self.int_error = 0.0
                self.prev_error = 0.0
                self.deriv_filt = 0.0
                return

        # 2) ¿Necesito iniciar una NUEVA evasión?
        need_front_evasion = (self.front_distance < 0.5)
        need_color_evasion = self.obstaculo

        if (need_front_evasion or need_color_evasion) and not self.auto_turning:
            self.auto_turning = True

            # Decidir a qué lado girar:
            if need_color_evasion and self.obstaculo_side is not None:
                # Evasión guiada por color
                if self.obstaculo_side == "izquierda":
                    self.turn_us = self.max_us
                    self.get_logger().info('Evasión (color): giro hacia IZQUIERDA.')
                else:  # "derecha"
                    self.turn_us = self.min_us
                    self.get_logger().info('Evasión (color): giro hacia DERECHA.')
            else:
                # Evasión guiada solo por LiDAR frontal -> escoger lado más libre
                if self.left_avg > self.right_avg:
                    self.turn_us = self.max_us
                    self.get_logger().info('Evasión (LiDAR): IZQUIERDA más libre.')
                else:
                    self.turn_us = self.min_us
                    self.get_logger().info('Evasión (LiDAR): DERECHA más libre.')

            # Iniciar en fase stop1
            self.evasion_state = "stop1"
            self.evasion_end_time = now + self.evasion_stop1

            # Reset PID al entrar en evasión
            self.int_error = 0.0
            self.prev_error = 0.0
            self.deriv_filt = 0.0

            # ya estamos en evasión, solo dejamos que la máquina de estados mande
            self.stop()
            self.send_servo(self.center_us)
            return

        # 3) SEGUIDOR DE CENTRO (no hay evasión activa)
        # Necesitamos lecturas válidas a izquierda y derecha
        if not np.isfinite(self.left_avg) or not np.isfinite(self.right_avg):
            # Si alguna lectura está mal, vamos recto
            self.get_logger().info('Lecturas laterales inválidas, avanzando recto.')
            self.send_servo(self.center_us)
            self.vel_pub.publish(Float32(data=self.speedMotor))

            self.int_error = 0.0
            self.prev_error = 0.0
            self.deriv_filt = 0.0
            return

        # Error = diferencia entre lados -> centro del pasillo
        # Si left > right: estás más cerca de derecha => error positivo => girar a izquierda
        error = self.left_avg - self.right_avg

        # Integral
        self.int_error += error * dt
        self.int_error = self.clamp(self.int_error, -self.int_limit, self.int_limit)

        # Derivada filtrada
        deriv = (error - self.prev_error) / dt
        self.deriv_filt = self.deriv_alpha * deriv + (1.0 - self.deriv_alpha) * self.deriv_filt

        # PID
        delta = self.Kp * error + self.Ki * self.int_error + self.Kd * self.deriv_filt

        if abs(delta) < self.us_deadzone:
            delta = 0.0

        delta = self.clamp(delta, -self.max_delta_us, self.max_delta_us)
        target = self.center_us + delta

        self.send_servo(target)
        self.vel_pub.publish(Float32(data=self.speedMotor))
        self.prev_error = error


def main(args=None):
    rclpy.init(args=args)
    node = AckerLidarController()
    try:
        rclpy.spin(node)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
