import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String  # NEW
from ros_robot_controller_msgs.msg import SetAckerServoState, ButtonState
import numpy as np
import math
import time


class AckerLidarController(Node):
    def __init__(self):
        super().__init__('acker_lidar_node')

        # Publishers
        self.ack_pub = self.create_publisher(SetAckerServoState, '/ros_robot_controller/acker_servo/set_state', 10)
        self.vel_pub = self.create_publisher(Float32, '/motor_vel', 10)

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(ButtonState, '/ros_robot_controller/button', self.button_callback, 10)
        self.create_subscription(String, '/obstaculos', self.obst_callback, 10)  # NEW

        # --- Configuración general ---
        self.max_time = 300.0
        self.speedMotor = 80.0
        self.turn_speed = 80.0
        self.center_us = 1500
        self.max_us = 2050      # izquierda (mayor us)
        self.min_us = 1000      # derecha  (menor us)

        # --- Parámetro: lado a seguir ---
        # admite: "right"/"left" o "derecha"/"izquierda"
        self.declare_parameter('follow_side', 'left')
        side_param = self.get_parameter('follow_side').get_parameter_value().string_value.lower()
        if side_param in ['left', 'izquierda']:
            self.follow_side = 'left'
        else:
            self.follow_side = 'right'
        self.get_logger().info(f'Seguidor configurado para pared a la {self.follow_side}.')

        # Control de cambios de lado (antirrebote)  # NEW
        self.last_side_change = 0.0  # NEW
        self.side_change_cooldown = 0.5  # s       # NEW

        # --- PD ---
        self.desired_dist = 0.16      # 16 cm (en m)
        self.Kp = 500.0              # us / m
        self.Kd = 400.0               # us / (m/s)
        self.deriv_alpha = 0.4        # filtro EMA para D
        self.us_deadzone = 1.0
        self.max_delta_us = 380.0

        self.prev_error = 0.0
        self.deriv_filt = 0.0
        self.last_t = time.time()

        # --- Evasión ---
        self.obstacle_thresh = 0.800
        self.turning = False
        self.turn_start_time = None
        self.turn_duration = 0.75
        self.turn_direction = 'izquierda'

        # Estado sensores
        self.button_pressed = False
        self.front_distance = float('inf')
        self.left_avg = float('nan')
        self.right_avg = float('nan')

        # Filtros EMA
        self.ema_alpha = 0.6
        self.left_ema = None
        self.right_ema = None
        self.front_ema = None

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Acker Lidar Controller (PD) listo.')

    # -------- utilidades --------
    @staticmethod
    def clamp(v, vmin, vmax):
        return max(vmin, min(vmax, v))

    @staticmethod
    def safe_mean(arr):
        arr = [d for d in arr if not math.isinf(d) and not math.isnan(d)]
        return float(np.mean(arr)) if arr else float('nan')

    def send_servo(self, position_us, duration_s=0.1):
        msg = SetAckerServoState()
        msg.position = int(self.clamp(position_us, self.min_us, self.max_us))
        msg.duration = float(duration_s)
        self.ack_pub.publish(msg)

    def stop(self):
        self.vel_pub.publish(Float32(data=0.0))

    # -------- callbacks --------
    def button_callback(self, _):
        self.button_pressed = True

    def obst_callback(self, msg: String):  # NEW
        """Recibe 'derecha' o 'izquierda' (o right/left) y ajusta el lado a seguir."""
        txt = msg.data.strip().lower()
        left_aliases = {'izquierda', 'left', 'l', 'izq'}
        right_aliases = {'derecha', 'right', 'r', 'der'}

        now = time.time()
        if now - self.last_side_change < self.side_change_cooldown:
            return  # ignora rebotes rápidos

        if txt in left_aliases and self.follow_side != 'left':
            self.follow_side = 'left'
            self.last_side_change = now
            self.prev_error = 0.0
            self.deriv_filt = 0.0
            self.send_servo(self.center_us, 0.1)
            self.get_logger().info('Cambiando a seguidor de pared por la IZQUIERDA.')
        elif txt in right_aliases and self.follow_side != 'right':
            self.follow_side = 'right'
            self.last_side_change = now
            self.prev_error = 0.0
            self.deriv_filt = 0.0
            self.send_servo(self.center_us, 0.1)
            self.get_logger().info('Cambiando a seguidor de pared por la DERECHA.')
        else:
            # Mensaje no reconocido o ya estamos en ese lado
            pass

    def scan_callback(self, msg: LaserScan):
        try:
            n = len(msg.ranges)
            # convención original: 240° izq, 120° der, 180° frente
            c_left  = int(n * (240.0/360.0))
            c_right = int(n * (120.0/360.0))
            c_front = int(n * (180.0/360.0))
            w_side, w_front = 12, 6

            rs, re = max(0, c_right - w_side), min(n, c_right + w_side)
            ls, le = max(0, c_left  - w_side), min(n, c_left  + w_side)
            fs, fe = max(0, c_front - w_front), min(n, c_front + w_front)

            rmin, rmax = msg.range_min, msg.range_max
            right = [d for d in msg.ranges[rs:re] if rmin < d < rmax]
            left  = [d for d in msg.ranges[ls:le] if rmin < d < rmax]
            front = [d for d in msg.ranges[fs:fe] if rmin < d < rmax]

            r = self.safe_mean(right)
            l = self.safe_mean(left)
            f = self.safe_mean(front)

            if not math.isnan(r):
                self.right_ema = r if self.right_ema is None else (self.ema_alpha*r + (1-self.ema_alpha)*self.right_ema)
                self.right_avg = self.right_ema
            if not math.isnan(l):
                self.left_ema  = l if self.left_ema  is None else (self.ema_alpha*l + (1-self.ema_alpha)*self.left_ema)
                self.left_avg = self.left_ema
            if not math.isnan(f):
                self.front_ema = f if self.front_ema is None else (self.ema_alpha*f + (1-self.ema_alpha)*self.front_ema)
                self.front_distance = self.front_ema

        except Exception as e:
            self.get_logger().error(f'scan_callback error: {e}')

    # -------- helpers de control --------
    def _get_side_distance(self):
        """Devuelve (dist, etiqueta) según lado configurado."""
        if self.follow_side == 'left':
            return self.left_avg, 'izquierda'
        else:
            return self.right_avg, 'derecha'

    def _steer_sign(self):
        """
        Signo para convertir delta_us en giro de servo:
        - right: target = center - delta  (sign = -1)
        - left : target = center + delta  (sign = +1)
        """
        return 1.0 if self.follow_side == 'left' else -1.0

    # -------- lazo de control --------
    def control_loop(self):
        if not self.button_pressed:
            self.stop()
            return

        try:
            now = time.time()
            dt = now - self.last_t
            if dt <= 0.0:
                dt = 1e-3
            self.last_t = now

            # mantener giro de evasión
            if self.turning:
                if time.time() - self.turn_start_time < self.turn_duration:
                    self.vel_pub.publish(Float32(data=self.turn_speed))
                    return
                self.turning = False
               # self.send_servo(self.center_us, 0.1)
               # self.vel_pub.publish(Float32(data=0.0))
                self.get_logger().info('Giro completado, retomando avance.')
                return

            # Modo evasión de obstáculo
            if self.front_distance <= self.obstacle_thresh and (self.left_avg < 10.7 or self.right_avg <10.7):
                self.turning = True
                self.turn_start_time = time.time()
                self.vel_pub.publish(Float32(data=self.turn_speed))

                left_val  = self.left_avg  if not math.isnan(self.left_avg)  else 0.0
                right_val = self.right_avg if not math.isnan(self.right_avg) else 0.0

                if left_val > right_val:
                    self.send_servo(self.max_us, 0.1)  # gira a la IZQ
                    self.turn_direction = 'izquierda'
                else:
                    self.send_servo(self.min_us, 0.1)  # gira a la DER
                    self.turn_direction = 'derecha'

                self.get_logger().info(f'Obstáculo a {self.front_distance:.2f} m. Giro {self.turn_direction}.')
                return

            # ---- Seguidor de pared configurable (PD) ----
            dist_side, side_label = self._get_side_distance()

            if not math.isnan(dist_side):
                error = dist_side - self.desired_dist         # +: lejos de la pared seleccionada
                deriv = (error - self.prev_error) / dt
                # filtro derivativo EMA
                self.deriv_filt = self.deriv_alpha * deriv + (1.0 - self.deriv_alpha) * self.deriv_filt

                delta_us = self.Kp * error + self.Kd * self.deriv_filt
                # límites y zona muerta
                delta_us = self.clamp(delta_us, -self.max_delta_us, self.max_delta_us)
                if abs(delta_us) < self.us_deadzone:
                    delta_us = 0.0

                # aplicar signo según lado
                target_us = self.center_us + self._steer_sign() * delta_us
                target_us = self.clamp(target_us, self.min_us, self.max_us)

                self.send_servo(target_us, 0.1)
                self.vel_pub.publish(Float32(data=self.speedMotor))
                self.prev_error = error

                self.get_logger().info(
                    f'PD {side_label} | dist:{dist_side:.2f} m e:{error:.3f} d:{self.deriv_filt:.3f} servo:{int(target_us)}'
                )
            else:
                # sin lectura del lado elegido: avanza recto y resetea D
                self.send_servo(self.center_us, 0.1)
                self.vel_pub.publish(Float32(data=self.speedMotor))
                self.prev_error = 0.0
                self.deriv_filt = 0.0

        except Exception as e:
            self.get_logger().error(f'control_loop error: {e}')
            self.stop()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AckerLidarController()
        rclpy.spin(node)
    except Exception as e:
        print(f'Error in main: {e}')
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
