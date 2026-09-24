#!/usr/bin/env python

import cv2
import torch
from ultralytics import YOLO
import math
from os import putenv

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import PoseStamped
from image_geometry import PinholeCameraModel
# import numpy as np
from std_msgs.msg import String

from cv_bridge import CvBridge

# For AMD ROCm
# putenv("HSA_OVERRIDE_GFX_VERSION", "10.3.0")
# For NVIDIA CUDA
# torch.cuda.set_device(0)

CLASS_LEFT = 1 
CLASS_RIGHT = 2 
CLASS_STOP = 3 
CLASS_UP = 6 
ACTIVE_CLASSES = {CLASS_LEFT, CLASS_RIGHT, CLASS_STOP, CLASS_UP}

class DetectionNode(Node):
    def __init__(self):
        super().__init__('comp_detection_node')
        self.bridge = CvBridge()
        self.detections = self.create_publisher(Image, '/yolo_detections', 10)
        self.detections_compressed = self.create_publisher(CompressedImage, '/yolo_detections/compressed', 10)
        self.subscription_compressed = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.subscription_compressed
        self.hand_sign_pub = self.create_publisher(
            String,
            '/hand_sign',
            10
        )
        # self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        # self.subscription  # prevent unused variable warning
        self.model = YOLO('/home/rss/sf_ws/src/yolo_to_ros/yolo_to_ros/yolov8n.pt')  # standard YOLOv8 nano model

    def image_callback(self, msg):

        # Convert ROS raw Image → OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # YOLO detection
        results = self.model(
            frame,
            imgsz=320,
            conf=0.5,
            stream=True
        )

        best_sign = None
        best_confidence = 0.0

        for r in results:
            boxes = r.boxes

            for box in boxes:

                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                confidence = float(box.conf[0])
                cls = int(box.cls[0])

                # Draw detection
                cv2.rectangle(
                    frame,
                    (x1, y1),
                    (x2, y2),
                    (100, 0, 255),
                    1
                )

                cv2.putText(
                    frame,
                    f"{r.names[cls]} {confidence:.2f}",
                    (x1, y1),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (100, 0, 255),
                    1
                )

                # Only use active hand-sign classes
                if cls in ACTIVE_CLASSES and confidence > best_confidence:
                    best_confidence = confidence

                    if cls == CLASS_LEFT:
                        best_sign = "LEFT"

                    elif cls == CLASS_RIGHT:
                        best_sign = "RIGHT"

                    elif cls == CLASS_STOP:
                        best_sign = "STOP"

                    elif cls == CLASS_UP:
                        best_sign = "UP"


        # Publish best detected sign
        if best_sign is not None:
            sign_msg = String()
            sign_msg.data = best_sign
            self.hand_sign_pub.publish(sign_msg)

        # -------------------------------------------------
        # 1. Publish RAW YOLO image
        # -------------------------------------------------
        raw_msg = self.bridge.cv2_to_imgmsg(
            frame,
            'bgr8'
        )

        raw_msg.header = msg.header

        self.detections.publish(raw_msg)

        # -------------------------------------------------
        # 2. Compress YOLO image
        # -------------------------------------------------
        success, encoded_image = cv2.imencode(
            '.jpg',
            frame,
            [cv2.IMWRITE_JPEG_QUALITY, 80]
        )

        if not success:
            self.get_logger().error(
                "Could not compress image"
            )
            return


        # -------------------------------------------------
        # 3. Create CompressedImage
        # -------------------------------------------------
        compressed_msg = CompressedImage()

        compressed_msg.header = msg.header
        compressed_msg.format = "jpeg"
        compressed_msg.data = encoded_image.tobytes()


        # -------------------------------------------------
        # 4. Publish compressed YOLO image
        # -------------------------------------------------
        self.detections_compressed.publish(
            compressed_msg
        )
                
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