"""
Vision Node for Aquavision 2.0.

Subscribes to camera images, runs YOLOv11 inference on a timer,
and publishes gate detection data (error_x, error_y, width_ratio, gate_visible).
All inference is non-blocking via timer-based processing of the latest frame.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from aquavision_msgs.msg import GateData

try:
    import cv2
    import numpy as np
except ImportError as e:
    raise RuntimeError(
        "OpenCV/NumPy failed to import. Ensure numpy<2.0 is installed: "
        "pip install 'numpy<2'"
    ) from e
from cv_bridge import CvBridge
from typing import Optional

# Ultralytics import — must be installed on the target machine
from ultralytics import YOLO


class VisionNode(Node):
    """ROS 2 node for YOLO-based gate detection.

    Subscribes to raw camera images, runs inference on a timer (non-blocking),
    and publishes normalized gate error data to /vauv/gate_data.

    Parameters:
        model_path (str): Path to YOLO best.pt weights file.
        target_gate_class (str): Class name to detect ('green_gate' or 'red_gate').
        confidence_threshold (float): Minimum detection confidence.
        inference_rate (float): Inference loop frequency in Hz.
    """

    def __init__(self) -> None:
        super().__init__('vision_node')

        # ── Declare parameters ──────────────────────────────────────────
        self.declare_parameter('model_path', 'best.pt')
        self.declare_parameter('target_gate_class', 'green_gate')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('inference_rate', 10.0)  # Hz

        model_path: str = (
            self.get_parameter('model_path').get_parameter_value().string_value
        )
        self._target_class: str = (
            self.get_parameter('target_gate_class')
            .get_parameter_value()
            .string_value
        )
        self._conf_thresh: float = (
            self.get_parameter('confidence_threshold')
            .get_parameter_value()
            .double_value
        )
        inference_rate: float = (
            self.get_parameter('inference_rate')
            .get_parameter_value()
            .double_value
        )

        # ── Load YOLO model ─────────────────────────────────────────────
        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        try:
            self._model: YOLO = YOLO(model_path)
            self.get_logger().info('✅ YOLO model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to load YOLO model: {e}')
            raise

        # Build class name → index mapping
        self._class_names: dict = self._model.names  # {0: 'green_gate', ...}
        self._target_class_idx: Optional[int] = None
        for idx, name in self._class_names.items():
            if name == self._target_class:
                self._target_class_idx = idx
                break
        if self._target_class_idx is None:
            self.get_logger().warn(
                f'⚠️ Target class "{self._target_class}" not found in model. '
                f'Available: {list(self._class_names.values())}'
            )

        # ── CV Bridge ────────────────────────────────────────────────────
        self._bridge: CvBridge = CvBridge()

        # ── Latest frame buffer (written by subscriber, read by timer) ──
        self._latest_frame: Optional[np.ndarray] = None

        # ── Publisher ────────────────────────────────────────────────────
        self._gate_pub = self.create_publisher(GateData, '/vauv/gate_data', 10)

        # ── Subscriber ──────────────────────────────────────────────────
        self.create_subscription(
            Image, '/camera/image_raw', self._image_callback, 10
        )

        # ── Inference timer (non-blocking) ──────────────────────────────
        timer_period: float = 1.0 / max(inference_rate, 1.0)
        self._inference_timer = self.create_timer(
            timer_period, self._inference_callback
        )

        self.get_logger().info(
            f'👁️ Vision node started | class={self._target_class} | '
            f'rate={inference_rate}Hz | conf≥{self._conf_thresh}'
        )

    # ── Callbacks ────────────────────────────────────────────────────────

    def _image_callback(self, msg: Image) -> None:
        """Store the latest image frame. Does NOT run inference."""
        try:
            self._latest_frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'Failed to convert image: {e}')

    def _inference_callback(self) -> None:
        """Run YOLO inference on the latest buffered frame."""
        frame = self._latest_frame
        if frame is None:
            # No frame received yet — publish invisible
            self._publish_no_gate()
            return

        frame_h, frame_w = frame.shape[:2]

        try:
            results = self._model(frame, verbose=False)
        except Exception as e:
            self.get_logger().error(f'Inference error: {e}')
            self._publish_no_gate()
            return

        # ── Filter detections ────────────────────────────────────────
        best_conf: float = 0.0
        best_box: Optional[np.ndarray] = None

        for result in results:
            boxes = result.boxes
            if boxes is None or len(boxes) == 0:
                continue

            for i in range(len(boxes)):
                cls_id: int = int(boxes.cls[i].item())
                conf: float = float(boxes.conf[i].item())

                # Filter by target class
                if (self._target_class_idx is not None
                        and cls_id != self._target_class_idx):
                    continue

                # Filter by confidence
                if conf < self._conf_thresh:
                    continue

                # Keep highest confidence
                if conf > best_conf:
                    best_conf = conf
                    best_box = boxes.xyxy[i].cpu().numpy()

        # ── Publish result ───────────────────────────────────────────
        if best_box is not None:
            x1, y1, x2, y2 = best_box
            cx: float = (x1 + x2) / 2.0
            cy: float = (y1 + y2) / 2.0
            box_w: float = x2 - x1

            # Normalized errors: [-1.0, 1.0]
            # Positive error_x = gate is to the right of center
            # Positive error_y = gate is below center
            error_x: float = (cx - frame_w / 2.0) / (frame_w / 2.0)
            error_y: float = (cy - frame_h / 2.0) / (frame_h / 2.0)
            width_ratio: float = box_w / frame_w

            # Clamp
            error_x = max(-1.0, min(1.0, error_x))
            error_y = max(-1.0, min(1.0, error_y))
            width_ratio = max(0.0, min(1.0, width_ratio))

            msg = GateData()
            msg.error_x = float(error_x)
            msg.error_y = float(error_y)
            msg.width_ratio = float(width_ratio)
            msg.gate_visible = True
            self._gate_pub.publish(msg)
        else:
            self._publish_no_gate()

    def _publish_no_gate(self) -> None:
        """Publish a GateData message indicating no gate is visible."""
        msg = GateData()
        msg.error_x = 0.0
        msg.error_y = 0.0
        msg.width_ratio = 0.0
        msg.gate_visible = False
        self._gate_pub.publish(msg)


def main(args=None) -> None:
    """Entry point for the vision node."""
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
