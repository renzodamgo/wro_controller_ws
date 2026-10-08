import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import lgpio
import time
import threading


class MotorPWMNode(Node):
    """
    Escucha /motor_vel (Float32) en el rango [-100, 100]:
      - valor > 0  -> avance
      - valor < 0  -> retroceso
      - valor == 0 -> detenido
    El signo controla los pines de dirección; el valor absoluto controla el duty.
    """

    def __init__(self):
        super().__init__('motor_pwm_node')

        # Configuración GPIO
        self.CHIP = 4
        self.PWM_PIN = 12   # BCM12
        self.DIR1_PIN = 5   # BCM5 (Dirección 1)
        self.DIR2_PIN = 6   # BCM6 (Dirección 2)

        self.freq = 1000    # Hz
        self.period = 1 / self.freq
        self.duty = 0.0     # duty [0.0 - 1.0]
        self.direction = 1  # 1 = avance, -1 = retroceso

        # Inicializar GPIO
        self.h = lgpio.gpiochip_open(self.CHIP)
        lgpio.gpio_claim_output(self.h, self.PWM_PIN)
        lgpio.gpio_claim_output(self.h, self.DIR1_PIN)
        lgpio.gpio_claim_output(self.h, self.DIR2_PIN)

        # Sentido por defecto: avance
        self.set_direction_forward()

        # Suscriptor
        self.subscription = self.create_subscription(
            Float32,
            '/motor_vel',
            self.vel_callback,
            10
        )

        # Hilo para generar PWM por software
        self.running = True
        self.pwm_thread = threading.Thread(target=self.pwm_loop)
        self.pwm_thread.start()

        self.get_logger().info(
            "MotorPWMNode iniciado. Escuchando /motor_vel en [-100, 100] "
            "(negativo = retroceso)."
        )

    # -------- Dirección --------

    def set_direction_forward(self):
        """Avance: DIR1=1, DIR2=0."""
        lgpio.gpio_write(self.h, self.DIR1_PIN, 1)
        lgpio.gpio_write(self.h, self.DIR2_PIN, 0)

    def set_direction_reverse(self):
        """Retroceso: DIR1=0, DIR2=1."""
        lgpio.gpio_write(self.h, self.DIR1_PIN, 0)
        lgpio.gpio_write(self.h, self.DIR2_PIN, 1)

    # -------- Callback --------

    def vel_callback(self, msg):
        v = max(-100.0, min(100.0, msg.data))

        new_dir = 1 if v >= 0 else -1

        # Solo tocamos los pines de dirección si cambia el sentido.
        # Para evitar estrés del puente H, ponemos duty a 0 un instante
        # antes de invertir la dirección.
        if new_dir != self.direction:
            self.duty = 0.0
            time.sleep(0.02)
            if new_dir == 1:
                self.set_direction_forward()
            else:
                self.set_direction_reverse()
            self.direction = new_dir

        self.duty = abs(v) / 100.0

    # -------- Generación de PWM --------

    def pwm_loop(self):
        while self.running:
            if self.duty > 0:
                lgpio.gpio_write(self.h, self.PWM_PIN, 1)
                time.sleep(self.period * self.duty)
                lgpio.gpio_write(self.h, self.PWM_PIN, 0)
                time.sleep(self.period * (1 - self.duty))
            else:
                lgpio.gpio_write(self.h, self.PWM_PIN, 0)
                time.sleep(self.period)

    def destroy_node(self):
        self.running = False
        self.pwm_thread.join()

        # Apagar PWM y pines de dirección
        lgpio.gpio_write(self.h, self.PWM_PIN, 0)
        lgpio.gpio_write(self.h, self.DIR1_PIN, 0)
        lgpio.gpio_write(self.h, self.DIR2_PIN, 0)
        lgpio.gpiochip_close(self.h)

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorPWMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()