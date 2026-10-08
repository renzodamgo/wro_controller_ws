#!/usr/bin/env python3
"""Enciende la tira WS2812 (SPI0, pin 19 / GPIO10) en blanco mientras corre.

Parametros: led_count (30) y brightness (0.0-1.0, por defecto 0.2).

La usa sensors.launch.py: la tira queda encendida mientras el launch este
activo y se apaga al cerrarlo (Ctrl+C). No escucha topics. Reenvia el color
cada segundo: si la tira se conecta con el nodo ya corriendo, igual se enciende.
"""

import rclpy
from rclpy.node import Node
from rpi5_ws2812.ws2812 import Color, WS2812SpiDriver


class LedLight(Node):
    def __init__(self):
        super().__init__('led_light')
        self.led_count = int(self.declare_parameter('led_count', 30).value)
        brightness = float(self.declare_parameter('brightness', 0.2).value)
        nivel = round(255 * max(0.0, min(1.0, brightness)))
        self.strip = WS2812SpiDriver(
            spi_bus=0, spi_device=0, led_count=self.led_count).get_strip()
        self.color = Color(nivel, nivel, nivel)
        self._mostrar()
        self.create_timer(1.0, self._mostrar)
        self.get_logger().info(
            f'Tira de {self.led_count} LEDs encendida en blanco al {brightness:.0%}')

    def _mostrar(self):
        self.strip.set_all_pixels(self.color)
        self.strip.show()

    def destroy_node(self):
        self.strip.set_all_pixels(Color(0, 0, 0))
        self.strip.show()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LedLight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.try_shutdown()
        except Exception:                             # noqa: BLE001
            pass


if __name__ == '__main__':
    main()
