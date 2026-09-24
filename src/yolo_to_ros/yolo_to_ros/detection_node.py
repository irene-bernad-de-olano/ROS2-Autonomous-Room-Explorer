#!/usr/bin/env python
import cv2
import torch
from ultralytics import YOLO
import math
import json
from os import putenv

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from image_geometry import PinholeCameraModel
from std_msgs.msg import String  # NUEVO: para publicar las clases detectadas

from cv_bridge import CvBridge

# For AMD ROCm
# putenv("HSA_OVERRIDE_GFX_VERSION", "10.3.0")
# For NVIDIA CUDA
# torch.cuda.set_device(0)


class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.bridge = CvBridge()
        self.detections = self.create_publisher(Image, '/yolo_detections', 10)
        # NUEVO: topic con los nombres de clase detectados en el frame actual,
        # separados por comas (vacio si no hay detecciones)
        self.classes_pub = self.create_publisher(String, '/yolo_detected_classes', 10)
        # Consumed by room_explorer_node.py to classify rooms (object + confidence,
        # unlike /yolo_detected_classes which only carries bare class names).
        self.data_pub = self.create_publisher(String, '/yolo_detections_data', 10)
        self.subscription = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.subscription  # prevent unused variable warning
        self.model = YOLO('/home/rss/sf_ws/src/yolo_to_ros/yolo_to_ros/yolov8n.pt')  # standard YOLOv8 nano model

    def image_callback(self, frame):
        frame = self.bridge.imgmsg_to_cv2(frame, "bgr8")
        results = self.model(frame, stream=True)

        detected_classes = set()  # NUEVO: acumula las clases de este frame
        detections_data = []  # object + confidence, for room classification

        for r in results:
            boxes = r.boxes
            for box in boxes:
                # Pixel coordinates
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                # Put boxes in frame
                cv2.rectangle(frame, (x1, y1), (x2, y2), (100, 0, 255), 1)

                # Confidence
                confidence = math.ceil((box.conf[0] * 100)) / 100

                # Optional confidence output in console
                # print("Confidence --->", confidence)

                # Class name
                cls = int(box.cls[0])

                # Optional class name output in console
                # print("Class name -->", r.names[cls])

                detected_classes.add(r.names[cls])  # NUEVO

                detections_data.append({
                    "object": r.names[cls],
                    "confidence": float(confidence),
                })

                org = [x1, y1]
                font = cv2.FONT_HERSHEY_SIMPLEX
                fontScale = 1
                color = (100, 0, 255)
                thickness = 1
                cv2.putText(frame, f"{r.names[cls]} {confidence}", org, font, fontScale, color, thickness)

        self.detections.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))

        # NUEVO: publica las clases detectadas en este frame
        classes_msg = String()
        classes_msg.data = ','.join(sorted(detected_classes))
        self.classes_pub.publish(classes_msg)

        data_msg = String()
        data_msg.data = json.dumps(detections_data)
        self.data_pub.publish(data_msg)


def main():
    rclpy.init()
    depth_to_pose_node = DetectionNode()
    try:
        rclpy.spin(depth_to_pose_node)
    except KeyboardInterrupt:
        pass
    depth_to_pose_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
