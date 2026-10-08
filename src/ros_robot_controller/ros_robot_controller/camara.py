#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
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
        self.create_subscription(String, '/obstaculos', self.obst_callback, 10)

        # --- Configuración general ---
        self.max_time = 300.0
        self.speedMotor = 30.0
        self.turn_speed = 30.0
        self.center_us = 1500
        self.max_us = 2050      # izquierda (mayor us)
        self.min_us = 1000      # derecha  (menor us)

        # --- Parámetro: lado a seguir por defecto ---
        self.declare_parameter('follow_side', 'left')
        side_param = self.get_parameter('follow_side').get_parameter_value().string_value.lower()
        self.follow_side = 'left' if side_param in ('left', 'izquierda') else 'right'
        self.get_logger().info(f'Seguidor configurado para pared a la {self.follow_side}.')

        # --- Mapeo color -> lado de pared a seguir ---
        # Convención WRO: el pilar ROJO se pasa por su derecha  -> seguir pared DERECHA
        #                 el pilar VERDE se pasa por su izquierda -> seguir pared IZQUIERDA
        # Si en tu pista es al revés, solo invierte este diccionario.
        self.color_to_side = {'rojo': 'right', 'verde': 'left'}
        self.dir_to_side = {
            'izquierda': 'left', 'left': 'left', 'l': 'left', 'izq': 'left',
            'derecha': 'right', 'right': 'right', 'r': 'right', 'der': 'right',
        }

        # Antirrebote de cambios de lado
        self.last_side_change = 0.0
        self.side_change_cooldown = 0.5  # s

        # --- PD ---
        self.desired_dist = 0.16      # 16 cm (en m)
        self.Kp = 1300.0              # us / m
        self.Kd = 400.0               # us / (m/s)
        self.deriv_alpha = 0.4        # filtro EMA para D
        self.us_deadzone = 1.0
        self.max_delta_us = 380.0

        self.prev_error = 0.0
        self.deriv_filt = 0.0
        self.last_t = time.time()

        # --- Evasión ---
        self.obstacle_thresh = 0.800
        self.turn_duration = 0.75
        self.turn_direction = 'izquierda'

        # --- Frenos y retroceso ---
        self.brake_duration = 0.5        # s de freno tras la corrección de servo
        self.emergency_dist = 0.10       # 10 cm en el cono frontal
        self.front_cone_deg = 30.0       # cono frontal total (±15°)
        self.reverse_duration = 0.5      # s de retroceso
        self.reverse_speed = -self.speedMotor
        self.emergency_cooldown = 1.0    # s de bloqueo tras una emergencia
        self.last_emergency_end = 0.0

        # --- Máquina de estados ---
        # 'normal' | 'turning' | 'brake_turn' | 'brake_emg' | 'reverse'
        self.state = 'normal'
        self.state_t0 = time.time()

        # Estado sensores
        self.button_pressed = False
        self.front_distance = float('inf')
        self.front_min_cone = float('inf')
        self.left_avg = float('nan')
        self.right_avg = float('nan')

        # Filtros EMA
        self.ema_alpha = 0.6
        self.left_ema = None
        self.right_ema = None
        self.front_ema = None

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Acker Lidar Controller (PD + color + freno/retroceso) listo.')

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

    def _set_state(self, new_state):
        self.state = new_state
        self.state_t0 = time.time()

    def _reset_pd(self):
        self.prev_error = 0.0
        self.deriv_filt = 0.0
        self.last_t = time.time()

    # -------- callbacks --------
    def button_callback(self, _):
        self.button_pressed = True

    def obst_callback(self, msg: String):
        """Acepta colores (ROJO/VERDE/SIN_COLOR) o lados (izquierda/derecha)."""
        txt = msg.data.strip().lower()

        if txt in self.color_to_side:
            target = self.color_to_side[txt]
        elif txt in self.dir_to_side:
            target = self.dir_to_side[txt]
        elif txt in ('sin_color', 'ninguno', 'none', ''):
            return  # sin detección: conserva el lado actual
        else:
            self.get_logger().warn(
                f'Mensaje no reconocido en /obstaculos: "{msg.data}"', throttle_duration_sec=2.0
            )
            return

        if target == self.follow_side:
            return

        now = time.time()
        if now - self.last_side_change < self.side_change_cooldown:
            return  # ignora rebotes rápidos

        self.follow_side = target
        self.last_side_change = now
        self._reset_pd()
        self.send_servo(self.center_us, 0.1)
        label = 'IZQUIERDA' if target == 'left' else 'DERECHA'
        self.get_logger().info(f'[{txt}] Cambiando a seguidor de pared por la {label}.')

    def scan_callback(self, msg: LaserScan):
        try:
            n = len(msg.ranges)
            # convención original: 240° izq, 120° der, 180° frente
            c_left = int(n * (240.0 / 360.0))
            c_right = int(n * (120.0 / 360.0))
            c_front = int(n * (180.0 / 360.0))
            w_side, w_front = 12, 6

            rs, re = max(0, c_right - w_side), min(n, c_right + w_side)
            ls, le = max(0, c_left - w_side), min(n, c_left + w_side)
            fs, fe = max(0, c_front - w_front), min(n, c_front + w_front)

            rmin, rmax = msg.range_min, msg.range_max
            right = [d for d in msg.ranges[rs:re] if rmin < d < rmax]
            left = [d for d in msg.ranges[ls:le] if rmin < d < rmax]
            front = [d for d in msg.ranges[fs:fe] if rmin < d < rmax]

            r = self.safe_mean(right)
            l = self.safe_mean(left)
            f = self.safe_mean(front)

            if not math.isnan(r):
                self.right_ema = r if self.right_ema is None else (self.ema_alpha * r + (1 - self.ema_alpha) * self.right_ema)
                self.right_avg = self.right_ema
            if not math.isnan(l):
                self.left_ema = l if self.left_ema is None else (self.ema_alpha * l + (1 - self.ema_alpha) * self.left_ema)
                self.left_avg = self.left_ema
            if not math.isnan(f):
                self.front_ema = f if self.front_ema is None else (self.ema_alpha * f + (1 - self.ema_alpha) * self.front_ema)
                self.front_distance = self.front_ema

            # Mínimo robusto en el cono frontal de 30° (±15°)
            w_cone = max(1, int(n * (self.front_cone_deg / 2.0) / 360.0))
            cs, ce = max(0, c_front - w_cone), min(n, c_front + w_cone)
            cone = [d for d in msg.ranges[cs:ce] if rmin < d < rmax]

            if len(cone) >= 3:
                # promedio de los 3 más cercanos: evita disparos por un rayo ruidoso
                self.front_min_cone = float(np.mean(sorted(cone)[:3]))
            elif cone:
                self.front_min_cone = float(min(cone))
            else:
                self.front_min_cone = float('inf')

        except Exception as e:
            self.get_logger().error(f'scan_callback error: {e}')

    # -------- helpers de control --------
    def _get_side_distance(self):
        if self.follow_side == 'left':
            return self.left_avg, 'izquierda'
        return self.right_avg, 'derecha'

    def _steer_sign(self):
        """right: target = center - delta (-1) | left: target = center + delta (+1)"""
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

            # ===== 1) EMERGENCIA: obstáculo a <10 cm en el cono frontal de 30° =====
            if (self.state in ('normal', 'turning')
                    and self.front_min_cone <= self.emergency_dist
                    and (now - self.last_emergency_end) > self.emergency_cooldown):
                self.stop()
                self.send_servo(self.center_us, 0.1)
                self._set_state('brake_emg')
                self.get_logger().warn(
                    f'EMERGENCIA: obstáculo a {self.front_min_cone:.3f} m. Freno + retroceso.'
                )
                return

            # ===== 2) Máquina de estados =====
            if self.state == 'brake_emg':
                self.stop()
                self.send_servo(self.center_us, 0.1)
                if now - self.state_t0 >= self.brake_duration:
                    self._set_state('reverse')
                    self.get_logger().info('Retrocediendo 0.5 s...')
                return

            if self.state == 'reverse':
                self.send_servo(self.center_us, 0.1)
                self.vel_pub.publish(Float32(data=self.reverse_speed))
                if now - self.state_t0 >= self.reverse_duration:
                    self.stop()
                    self.last_emergency_end = now
                    self._reset_pd()
                    self._set_state('normal')
                    self.get_logger().info('Retroceso completado, retomando control.')
                return

            if self.state == 'turning':
                if now - self.state_t0 < self.turn_duration:
                    self.vel_pub.publish(Float32(data=self.turn_speed))
                    return
                self.stop()
                self._set_state('brake_turn')
                self.get_logger().info('Giro completado. Frenando 0.5 s.')
                return

            if self.state == 'brake_turn':
                self.stop()
                if now - self.state_t0 >= self.brake_duration:
                    self._reset_pd()
                    self._set_state('normal')
                    self.get_logger().info('Freno terminado, retomando avance.')
                return

            # ===== 3) Evasión de obstáculo frontal =====
            if self.front_distance <= self.obstacle_thresh and (self.left_avg < 10.7 or self.right_avg < 10.7):
                left_val = self.left_avg if not math.isnan(self.left_avg) else 0.0
                right_val = self.right_avg if not math.isnan(self.right_avg) else 0.0

                if left_val > right_val:
                    self.send_servo(self.max_us, 0.1)   # gira a la IZQ
                    self.turn_direction = 'izquierda'
                else:
                    self.send_servo(self.min_us, 0.1)   # gira a la DER
                    self.turn_direction = 'derecha'

                self.vel_pub.publish(Float32(data=self.turn_speed))
                self._set_state('turning')
                self.get_logger().info(f'Obstáculo a {self.front_distance:.2f} m. Giro {self.turn_direction}.')
                return

            # ===== 4) Seguidor de pared (PD) =====
            dist_side, side_label = self._get_side_distance()

            if not math.isnan(dist_side):
                error = dist_side - self.desired_dist         # +: lejos de la pared elegida
                deriv = (error - self.prev_error) / dt
                self.deriv_filt = self.deriv_alpha * deriv + (1.0 - self.deriv_alpha) * self.deriv_filt

                delta_us = self.Kp * error + self.Kd * self.deriv_filt
                delta_us = self.clamp(delta_us, -self.max_delta_us, self.max_delta_us)
                if abs(delta_us) < self.us_deadzone:
                    delta_us = 0.0

                target_us = self.center_us + self._steer_sign() * delta_us
                target_us = self.clamp(target_us, self.min_us, self.max_us)

                self.send_servo(target_us, 0.1)
                self.vel_pub.publish(Float32(data=self.speedMotor))
                self.prev_error = error

                self.get_logger().info(
                    f'PD {side_label} | dist:{dist_side:.2f} e:{error:.3f} d:{self.deriv_filt:.3f} '
                    f'servo:{int(target_us)} cono:{self.front_min_cone:.2f}'
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
