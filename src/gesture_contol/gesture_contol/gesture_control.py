import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CompressedImage, Image
from ultralytics import YOLO

# ids de clase segun el data.yaml del dataset (Hand Gesture.v6i.yolov8-obb)
CLASS_LEFT = 1
CLASS_RIGHT = 2
CLASS_STOP = 3
CLASS_UP = 6

ACTIVE_CLASSES = {CLASS_LEFT, CLASS_RIGHT, CLASS_STOP, CLASS_UP}


class GestureControl(Node):
    def __init__(self, name='gesture_control'):
        super().__init__(name)

        model_path = self.declare_parameter(
            'model_path', '/home/rss/sf_ws/src/yolo_to_ros/yolo_to_ros/best.pt'
        ).value
        image_topic = self.declare_parameter(
            'image_topic', '/image_raw/compressed'
        ).value
        self.conf_threshold = self.declare_parameter('conf_threshold', 0.35).value
        self.linear_speed = self.declare_parameter('linear_speed', 0.15).value
        self.angular_speed = self.declare_parameter('angular_speed', 0.4).value
        self.decision_period = self.declare_parameter('decision_period', 2.0).value
        self.max_image_age = self.declare_parameter('max_image_age', 0.5).value

        self.get_logger().info(f'Cargando modelo YOLO: {model_path}')
        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        self.pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.detections_pub = self.create_publisher(Image, 'gesture_detections', 10)
        self.subscription = self.create_subscription(
            CompressedImage, image_topic, self.image_callback, 10
        )

        self._latest_detected_cls = None
        self._last_image_time = None
        self._current_cmd = CLASS_STOP

        # Decision and publishing run on separate clocks: the gesture is only
        # (re)read every decision_period seconds so a flickering detection
        # doesn't make the robot flip commands frame to frame, while cmd_vel
        # keeps publishing (and can still stop fast) at a normal control rate.
        self.decision_timer = self.create_timer(self.decision_period, self.decision_callback)
        self.publish_timer = self.create_timer(0.1, self.publish_callback)

    def _best_detection(self, results):
        obb = results.obb
        if obb is not None and len(obb) > 0:
            classes = obb.cls.tolist()
            confs = obb.conf.tolist()
        else:
            boxes = results.boxes
            if boxes is None or len(boxes) == 0:
                return None
            classes = boxes.cls.tolist()
            confs = boxes.conf.tolist()

        best_cls, best_conf = None, 0.0
        for cls, conf in zip(classes, confs):
            cls = int(cls)
            if cls in ACTIVE_CLASSES and conf >= self.conf_threshold and conf > best_conf:
                best_cls, best_conf = cls, conf
        return best_cls

    def _twist_for_current_cmd(self):
        # Left/right turn while moving forward, they don't rotate in place.
        twist = Twist()
        if self._current_cmd == CLASS_UP:
            twist.linear.x = self.linear_speed
        elif self._current_cmd == CLASS_LEFT:
            twist.linear.x = self.linear_speed
            twist.angular.z = self.angular_speed
        elif self._current_cmd == CLASS_RIGHT:
            twist.linear.x = self.linear_speed
            twist.angular.z = -self.angular_speed
        return twist

    def _image_is_fresh(self):
        if self._last_image_time is None:
            return False
        age = (self.get_clock().now() - self._last_image_time).nanoseconds * 1e-9
        return 0.0 <= age <= self.max_image_age

    def stop(self):
        self.pub.publish(Twist())

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        results = self.model(frame, verbose=False)[0]
        self._latest_detected_cls = self._best_detection(results)
        self._last_image_time = self.get_clock().now()

        annotated = results.plot()
        self.detections_pub.publish(self.bridge.cv2_to_imgmsg(annotated, 'bgr8'))

    def decision_callback(self):
        target = self._latest_detected_cls if self._image_is_fresh() else None
        target = target if target is not None else CLASS_STOP

        if target != self._current_cmd:
            self._current_cmd = target
            self.get_logger().info(f'Nuevo comando: {self._current_cmd}')

    def publish_callback(self):
        if not self._image_is_fresh():
            self.stop()
            return
        self.pub.publish(self._twist_for_current_cmd())


def main(args=None):
    # Keep ROS alive during Ctrl+C cleanup so we can request a stop.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = GestureControl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    finally:
        node.decision_timer.cancel()
        node.publish_timer.cancel()
        if rclpy.ok():
            node.stop()
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
