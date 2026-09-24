#!/usr/bin/env python3
"""
room_announcer_node.py

Nodo ROS2 que escucha las clases detectadas por YOLO (topic std_msgs/String
'/yolo_detected_classes', publicado por detection_node.py) y, una vez por
segundo, imprime en que "habitacion" cree que esta el robot segun los
objetos detectados en el ultimo frame recibido.

Requiere que detection_node.py publique '/yolo_detected_classes' ademas de
la imagen '/yolo_detections' (ver el detection_node.py modificado que
acompana a este archivo).

Reglas (en este orden de prioridad si se detecta mas de una clase a la vez):
  - toilet        -> "we are in the bathroom"
  - potted plant  -> "we are in the bederoom"
  - sink          -> "we are in the kitfchen"
  - ninguna       -> "we are in an emptyroom"
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RoomAnnouncerNode(Node):
    def __init__(self):
        super().__init__('room_announcer_node')

        # Ultimas clases detectadas recibidas. Se actualiza cada vez que
        # llega un mensaje, pero solo se imprime una vez por segundo
        # gracias al timer de abajo (independiente del framerate de YOLO).
        self.detected_classes = set()

        self.subscription = self.create_subscription(
            String,
            '/yolo_detected_classes',
            self.detections_callback,
            10
        )
        self.subscription  # evita warning de variable sin usar

        # Timer a 1 Hz: imprime el estado una vez por segundo
        self.timer = self.create_timer(1.0, self.print_room)

        self.get_logger().info('room_announcer_node iniciado, esperando detecciones...')

    def detections_callback(self, msg: String):
        if msg.data:
            self.detected_classes = set(msg.data.split(','))
        else:
            self.detected_classes = set()

    def print_room(self):
        if 'toilet' in self.detected_classes:
            print("we are in the bathroom")
        elif 'potted plant' in self.detected_classes:
            print("we are in the bederoom")
        elif 'sink' in self.detected_classes:
            print("we are in the kitfchen")
        else:
            print("we are in an emptyroom")


def main():
    rclpy.init()
    node = RoomAnnouncerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
