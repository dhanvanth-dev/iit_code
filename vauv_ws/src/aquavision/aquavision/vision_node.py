"""
Vision Node for Aquavision 2.0.

Captures camera frames directly via OpenCV VideoCapture (GStreamer pipeline
on Jetson Orin Nano), runs YOLOv11 inference on a timer, publishes gate
detection data, and optionally displays a visualization window.

Direct capture eliminates ROS subscriber + cv_bridge latency.
"""

import rclpy
from rclpy.node import Node
from aquavision_msgs.msg import GateData

import cv2
import numpy as np
from typing import Optional
import logging

# Ultralytics import — must be installed on the target machine
from ultralytics import YOLO

# ── GPU optimization (skills.md §3.1, §7) ───────────────────────────────
try:
    import torch
    if torch.cuda.is_available():
        torch.backends.cudnn.benchmark = True
        _CUDA_AVAILABLE = True
    else:
        _CUDA_AVAILABLE = False
except ImportError:
    _CUDA_AVAILABLE = False

logger = logging.getLogger(__name__)


class VisionNode(Node):
    """ROS 2 node for YOLO-based gate detection with direct camera capture.

    Opens the camera directly via OpenCV (bypassing the ROS image topic
    pipeline) for minimal latency on Jetson. Optionally shows a live
    visualization window with bounding boxes and error overlay.

    Parameters:
        model_path (str): Path to YOLO best.pt weights file.
        target_gate_class (str): Class name to detect ('green_gate' or 'red_gate').
        confidence_threshold (float): Minimum detection confidence.
        inference_rate (float): Inference loop frequency in Hz.
        camera_id (int): /dev/video<N> device index for V4L2, or pipeline index.
        camera_width (int): Capture width in pixels.
        camera_height (int): Capture height in pixels.
        use_gstreamer (bool): Use GStreamer pipeline (recommended on Jetson).
        show_display (bool): Show OpenCV visualization window (disable for deployment).
    """

    def __init__(self) -> None:
        super().__init__('vision_node')

        # ── Declare parameters ──────────────────────────────────────────
        self.declare_parameter('model_path', 'best.pt')
        self.declare_parameter('target_gate_class', 'green_gate')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('inference_rate', 15.0)  # Hz (skills.md: ≥15 FPS)
        self.declare_parameter('use_fp16', True)
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('use_gstreamer', True)
        self.declare_parameter('show_display', True)

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
        self._use_fp16: bool = (
            self.get_parameter('use_fp16')
            .get_parameter_value()
            .bool_value
        ) and _CUDA_AVAILABLE
        camera_id: int = (
            self.get_parameter('camera_id')
            .get_parameter_value()
            .integer_value
        )
        cam_w: int = (
            self.get_parameter('camera_width')
            .get_parameter_value()
            .integer_value
        )
        cam_h: int = (
            self.get_parameter('camera_height')
            .get_parameter_value()
            .integer_value
        )
        use_gst: bool = (
            self.get_parameter('use_gstreamer')
            .get_parameter_value()
            .bool_value
        )
        self._show_display: bool = (
            self.get_parameter('show_display')
            .get_parameter_value()
            .bool_value
        )

        # ── Open camera directly ─────────────────────────────────────────
        if use_gst:
            # GStreamer pipeline optimized for Jetson Orin Nano (JetPack 6.1)
            # nvarguscamerasrc → CSI camera with hardware ISP
            # v4l2src → USB camera fallback
            gst_pipeline: str = (
                f'v4l2src device=/dev/video{camera_id} ! '
                f'video/x-raw, width={cam_w}, height={cam_h}, framerate=30/1 ! '
                f'videoconvert ! '
                f'video/x-raw, format=BGR ! '
                f'appsink drop=1'
            )
            self.get_logger().info(f'📷 GStreamer pipeline: {gst_pipeline}')
            self._cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        else:
            self.get_logger().info(f'📷 Opening /dev/video{camera_id} via V4L2')
            self._cap = cv2.VideoCapture(camera_id)
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam_w)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam_h)

        if not self._cap.isOpened():
            self.get_logger().error('❌ Failed to open camera')
            raise RuntimeError(f'Cannot open camera device {camera_id}')

        if self._use_fp16:
            self.get_logger().info(
                '⚡ FP16 half-precision inference ENABLED (CUDA available)'
            )
        elif _CUDA_AVAILABLE:
            self.get_logger().info('🔧 CUDA available but FP16 disabled by param')
        else:
            self.get_logger().warn(
                '⚠️ CUDA not available — falling back to CPU inference'
            )

        self.get_logger().info(
            f'✅ Camera opened: {cam_w}x{cam_h} '
            f'(gstreamer={use_gst})'
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

        # ── Publisher ────────────────────────────────────────────────────
        self._gate_pub = self.create_publisher(GateData, '/vauv/gate_data', 10)

        # ── Inference timer (non-blocking) ──────────────────────────────
        timer_period: float = 1.0 / max(inference_rate, 1.0)
        self._inference_timer = self.create_timer(
            timer_period, self._inference_callback
        )

        if self._show_display:
            self.get_logger().info(
                '🖥️ Visualization window ENABLED (set show_display:=false to disable)'
            )

        self.get_logger().info(
            f'👁️ Vision node started | class={self._target_class} | '
            f'rate={inference_rate}Hz | conf≥{self._conf_thresh}'
        )

    # ── Inference timer callback ─────────────────────────────────────────

    def _inference_callback(self) -> None:
        """Grab latest frame from camera, run YOLO, publish and optionally display."""
        ret, frame = self._cap.read()
        if not ret or frame is None:
            self._publish_no_gate()
            return

        frame_h, frame_w = frame.shape[:2]

        try:
            results = self._model(frame, verbose=False, half=self._use_fp16)
        except Exception as e:
            self.get_logger().error(f'Inference error: {e}')
            self._publish_no_gate()
            return

        # ── Filter detections ────────────────────────────────────────
        best_conf: float = 0.0
        best_box: Optional[np.ndarray] = None
        best_cls_name: str = ''

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
                    best_cls_name = self._class_names.get(cls_id, '?')

        # ── Compute errors and publish ───────────────────────────────
        error_x: float = 0.0
        error_y: float = 0.0
        width_ratio: float = 0.0
        gate_visible: bool = False

        if best_box is not None:
            x1, y1, x2, y2 = best_box
            cx: float = (x1 + x2) / 2.0
            cy: float = (y1 + y2) / 2.0
            box_w: float = x2 - x1

            # Normalized errors: [-1.0, 1.0]
            error_x = (cx - frame_w / 2.0) / (frame_w / 2.0)
            error_y = (cy - frame_h / 2.0) / (frame_h / 2.0)
            width_ratio = box_w / frame_w

            # Clamp
            error_x = max(-1.0, min(1.0, error_x))
            error_y = max(-1.0, min(1.0, error_y))
            width_ratio = max(0.0, min(1.0, width_ratio))
            gate_visible = True

            msg = GateData()
            msg.error_x = float(error_x)
            msg.error_y = float(error_y)
            msg.width_ratio = float(width_ratio)
            msg.gate_visible = True
            self._gate_pub.publish(msg)
        else:
            self._publish_no_gate()

        # ── Visualization ────────────────────────────────────────────
        if self._show_display:
            self._draw_overlay(
                frame, best_box, best_conf, best_cls_name,
                error_x, error_y, width_ratio, gate_visible
            )
            cv2.imshow('Aquavision – Gate Detection', frame)
            cv2.waitKey(1)  # Required for OpenCV window refresh

    # ── Visualization helpers ────────────────────────────────────────────

    def _draw_overlay(
        self,
        frame: np.ndarray,
        box: Optional[np.ndarray],
        conf: float,
        cls_name: str,
        error_x: float,
        error_y: float,
        width_ratio: float,
        gate_visible: bool,
    ) -> None:
        """Draw bounding box, crosshair, and telemetry overlay on frame."""
        frame_h, frame_w = frame.shape[:2]
        cx_frame = frame_w // 2
        cy_frame = frame_h // 2

        # ── Crosshair at frame center ────────────────────────────────
        cross_color = (0, 255, 0) if gate_visible else (0, 0, 255)
        cv2.line(frame, (cx_frame - 20, cy_frame), (cx_frame + 20, cy_frame),
                 cross_color, 2)
        cv2.line(frame, (cx_frame, cy_frame - 20), (cx_frame, cy_frame + 20),
                 cross_color, 2)

        if box is not None:
            x1, y1, x2, y2 = box.astype(int)

            # ── Bounding box ─────────────────────────────────────────
            box_color = (0, 255, 0)  # Green
            cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)

            # ── Gate center dot ──────────────────────────────────────
            gate_cx = (x1 + x2) // 2
            gate_cy = (y1 + y2) // 2
            cv2.circle(frame, (gate_cx, gate_cy), 6, (0, 0, 255), -1)

            # ── Line from frame center to gate center ────────────────
            cv2.line(frame, (cx_frame, cy_frame), (gate_cx, gate_cy),
                     (255, 255, 0), 1, cv2.LINE_AA)

            # ── Label ────────────────────────────────────────────────
            label = f'{cls_name} {conf:.2f}'
            (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
            cv2.rectangle(frame, (x1, y1 - lh - 8), (x1 + lw + 4, y1), box_color, -1)
            cv2.putText(frame, label, (x1 + 2, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA)

        # ── Telemetry HUD (top-left) ────────────────────────────────
        hud_lines = [
            f'Gate: {"VISIBLE" if gate_visible else "NOT FOUND"}',
            f'err_x: {error_x:+.3f}',
            f'err_y: {error_y:+.3f}',
            f'width: {width_ratio:.3f}',
        ]
        for i, line in enumerate(hud_lines):
            color = (0, 255, 0) if gate_visible else (0, 0, 255)
            cv2.putText(frame, line, (10, 25 + i * 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)

    def _publish_no_gate(self) -> None:
        """Publish a GateData message indicating no gate is visible."""
        msg = GateData()
        msg.error_x = 0.0
        msg.error_y = 0.0
        msg.width_ratio = 0.0
        msg.gate_visible = False
        self._gate_pub.publish(msg)

    def destroy_node(self) -> None:
        """Release camera and close display window on shutdown."""
        if self._cap is not None and self._cap.isOpened():
            self._cap.release()
        if self._show_display:
            cv2.destroyAllWindows()
        super().destroy_node()


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
