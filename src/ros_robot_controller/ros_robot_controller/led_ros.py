import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rpi5_ws2812.ws2812 import Color, WS2812SpiDriver

class ObstaculosLedNode(Node):
    def __init__(self):
        super().__init__('led_obstaculos_subscriber')

        # 1. Configuración de los LEDs (SPI Bus 0, Dispositivo 0)
        self.num_leds = 30  # Cambia esto al número total de LEDs que tienes
        self.driver = WS2812SpiDriver(spi_bus=0, spi_device=0, led_count=self.num_leds)
        self.strip = self.driver.get_strip()

        # Timer para controlar la duración del cambio de color
        self.reset_timer = None

        # Establecer color inicial por defecto (Blanco)
        self.set_white()

        # 2. Suscriptor al tópico /obstaculos
        self.subscription = self.create_subscription(
            String,
            '/obstaculos',
            self.listener_callback,
            10
        )
        self.get_logger().info('Nodo de LEDs para /obstaculos iniciado correctamente.')

    def set_white(self):
        """Método auxiliar para encender todos los LEDs en blanco."""
        # Blanco: (R, G, B) -> (255, 255, 255)
        self.strip.set_all_pixels(Color(255, 255, 255))
        self.strip.show()

    def reset_to_white_callback(self):
        """Callback ejecutado por el timer al pasar 1 segundo."""
        self.set_white()
        self.get_logger().info('Regresado a BLANCO')
        
        # Cancelamos y destruimos el timer para que no se repita
        if self.reset_timer is not None:
            self.reset_timer.cancel()
            self.destroy_timer(self.reset_timer)
            self.reset_timer = None

    def listener_callback(self, msg):
        comando = msg.data.strip().upper()
        self.get_logger().info(f'Mensaje recibido: "{comando}"')

        if comando in ('DERECHA', 'IZQUIERDA'):
            # Si ya había un timer corriendo, lo cancelamos para reiniciar la cuenta de 1s
            if self.reset_timer is not None:
                self.reset_timer.cancel()
                self.destroy_timer(self.reset_timer)

            if comando == 'DERECHA':
                # Rojo: (R, G, B) -> (255, 0, 0)
                self.strip.set_all_pixels(Color(255, 255, 255))
                self.strip.show()
                self.get_logger().info('Cambiado a ROJO por 1 segundo')

            elif comando == 'IZQUIERDA':
                # Verde: (R, G, B) -> (0, 255, 0)
                self.strip.set_all_pixels(Color(255, 255, 255))
                self.strip.show()
                self.get_logger().info('Cambiado a VERDE por 1 segundo')

            # Creamos un timer no bloqueante que ejecutará la función tras 1.0 segundo
            self.reset_timer = self.create_timer(1.0, self.reset_to_white_callback)

        else:
            self.get_logger().warn(f'Comando no reconocido: {comando}')

    def destroy_node(self):
        # Apagar los LEDs de forma limpia al apagar el nodo
        self.strip.set_all_pixels(Color(0, 0, 0))
        self.strip.show()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObstaculosLedNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
