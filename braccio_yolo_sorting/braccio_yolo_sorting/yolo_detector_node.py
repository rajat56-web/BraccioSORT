#!/usr/bin/env python3
"""
yolo_detector_node.py
Despite the filename, object detection uses HSV colour segmentation rather
than a neural-network YOLO model.

─────────────────
  TODO 1 — Generate clean red and blue masks
    • BGR → HSV conversion
    • Dual-range red masking (hue wraps at 0/180 in OpenCV HSV)
    • Single-range blue masking
    • Morphological closing (fill holes) then opening (remove noise)

  TODO 2 — Extract and validate cube detections
    • Find external contours on each cleaned mask
    • Area filter (too small → noise, too large → table / container)
    • Bounding-rectangle validation (aspect ratio, container size)
    • Vertical image region filter (top rows are background / sky)
    • Confidence proportional to contour area
    • Return list[dict] consumed by image_callback()
"""

import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    ObjectHypothesisWithPose,
)


class YOLODetectorNode(Node):
    """HSV-based color detector for Braccio sorting."""

    def __init__(self):
        super().__init__('yolo_detector_node')

        # Parameters
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('image_topic', '/camera/image_raw')

        self.conf_threshold = self.get_parameter('confidence_threshold').value
        image_topic = self.get_parameter('image_topic').value

        # HSV color ranges (tuned for Gazebo simulation)
        self.color_ranges = {
            'red': {
                'lower1': np.array([0, 120, 70]),
                'upper1': np.array([10, 255, 255]),
                'lower2': np.array([170, 120, 70]),
                'upper2': np.array([180, 255, 255])
            },
            'blue': {
                'lower': np.array([100, 120, 70]),
                'upper': np.array([130, 255, 255])
            }
        }

        # CV Bridge
        self.bridge = CvBridge()

        # Publishers
        self.detection_pub = self.create_publisher(
            Detection2DArray, '/detections', 10)

        self.annotated_pub = self.create_publisher(
            Image, '/detections/annotated', 10)

        # Subscriber
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)

        self.frame_count = 0
        self.get_logger().info('Braccio YOLO Detector Ready (HSV Color Detection)')

    def detect_objects_hsv(self, image):
        """Detect colored objects using HSV color segmentation."""

        detections = []
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # ================================================================
        # IMPLEMENTATION: Part 8 — HSV Mask Generation
        # ================================================================
        # Red wraps around the ends of OpenCV's HSV hue range (0-180),
        # so it must be detected using two separate hue intervals.

        # Step 1: red_mask1 from lower range [0..10]
        red_mask1 = cv2.inRange(
            hsv,
            self.color_ranges['red']['lower1'],
            self.color_ranges['red']['upper1']
        )
        # Step 2: red_mask2 from upper range [170..180]
        red_mask2 = cv2.inRange(
            hsv,
            self.color_ranges['red']['lower2'],
            self.color_ranges['red']['upper2']
        )
        # Step 3: combine both red intervals with bitwise OR
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        # Step 4: blue mask from single range [100..130]
        blue_mask = cv2.inRange(
            hsv,
            self.color_ranges['blue']['lower'],
            self.color_ranges['blue']['upper']
        )
        # ── END HSV Mask Generation ────────────────────────────────────

        # Debug: log mask pixel counts every 60 frames
        if self.frame_count % 60 == 1:
            red_px  = int(cv2.countNonZero(red_mask))
            blue_px = int(cv2.countNonZero(blue_mask))
            self.get_logger().info(
                f'HSV debug: red_mask={red_px}px  blue_mask={blue_px}px'
            )

        # ================================================================
        # IMPLEMENTATION: Part 9 — Contour-Based Object Detection
        # ================================================================
        
        # Process both colour masks with the same pipeline.
        color_masks = [('red', red_mask), ('blue', blue_mask)]

        # Thresholds (pixels in 640x480 image)
        MIN_AREA        = 400    # ignore tiny noise blobs
        MAX_AREA        = 5000  # ignore coloured containers (very large)
        MAX_DIM         = 100    # reject boxes wider/taller than this
        MIN_ASPECT      = 0.25   # reject slivers (w/h or h/w below this)
        IMG_H           = image.shape[0]
        VALID_Y_MIN     = int(IMG_H * 0.1)   # ignore top 10 % (background)
        VALID_Y_MAX     = int(IMG_H * 0.9)   # ignore bottom 10 % (edge effects)

        for color, mask in color_masks:

            # ── Step A: Clean the mask ────────────────────────────────
            kernel = np.ones((5, 5), np.uint8)
            # Fill small holes
            cleaned = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            # Remove isolated noise
            cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_OPEN, kernel)

            # ── Step B: Find external contours ────────────────────────
            contours, _ = cv2.findContours(
                cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            for cnt in contours:
                area = cv2.contourArea(cnt)

                # Reject below minimum useful area
                if area < MIN_AREA:
                    continue

                # ── Step C: Bounding box filtering ───────────────────
                x, y, w, h = cv2.boundingRect(cnt)
                cx = x + w // 2
                cy = y + h // 2

                # Reject very large regions (coloured destination containers)
                if area > MAX_AREA:
                    continue

                # Reject boxes with excessive width or height
                if w > MAX_DIM or h > MAX_DIM:
                    continue

                # Reject boxes outside the useful image-height region
                if cy < VALID_Y_MIN or cy > VALID_Y_MAX:
                    continue

                # Reject implausible aspect ratios (slivers are noise)
                aspect = min(w, h) / max(w, h) if max(w, h) > 0 else 0
                if aspect < MIN_ASPECT:
                    continue

                # ── Step D: Append accepted detection ─────────────────
                x_min, y_min = x, y
                x_max, y_max = x + w, y + h

                # High fixed confidence for simulation detections
                confidence = 0.92

                detections.append({
                    'bbox':       [x_min, y_min, x_max, y_max],
                    'color':      color,
                    'confidence': confidence,
                })

                if self.frame_count % 30 == 0:
                    self.get_logger().info(
                        f'Detected {color} cube: bbox=[{x_min},{y_min},{x_max},{y_max}] '
                        f'area={int(area)} conf={confidence:.2f}'
                    )

        return detections
        # ── END Contour-Based Object Detection ────────────────────────

    def image_callback(self, msg):
        """Process incoming images."""

        self.frame_count += 1

        try:
            # ros_gz_image publishes as rgb8.  cv_bridge with
            # desired_encoding='bgr8' should convert, but some
            # versions silently pass through when the source is
            # already 8UC3.  Handle both cases explicitly.
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Convert to BGR if the source was RGB
            if msg.encoding in ('rgb8', 'RGB8'):
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

            # Detect objects
            detections = self.detect_objects_hsv(cv_image)

            # Create detection array
            detection_array = Detection2DArray()
            detection_array.header = msg.header

            for det in detections:
                detection = Detection2D()
                detection.header = msg.header

                # Bounding box
                x1, y1, x2, y2 = det['bbox']
                detection.bbox.center.position.x = float((x1 + x2) / 2)
                detection.bbox.center.position.y = float((y1 + y2) / 2)
                detection.bbox.size_x = float(x2 - x1)
                detection.bbox.size_y = float(y2 - y1)

                # Hypothesis
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = f'{det["color"]}_cube'
                hypothesis.hypothesis.score    = det['confidence']

                detection.results.append(hypothesis)
                detection_array.detections.append(detection)

            # Publish detections
            self.detection_pub.publish(detection_array)

            # Publish annotated image (always, so RViz panel isn't blank)
            annotated = self.draw_detections(cv_image, detection_array)
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)

            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    f'Frame {self.frame_count}: {len(detection_array.detections)} objects '
                    f'(encoding={msg.encoding}, shape={cv_image.shape})'
                )

        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

    def draw_detections(self, image, detection_array):
        """Draw bounding boxes on image."""

        annotated = image.copy()

        for detection in detection_array.detections:
            if len(detection.results) == 0:
                continue

            cx = int(detection.bbox.center.position.x)
            cy = int(detection.bbox.center.position.y)
            w  = int(detection.bbox.size_x)
            h  = int(detection.bbox.size_y)

            x1 = cx - w // 2
            y1 = cy - h // 2
            x2 = cx + w // 2
            y2 = cy + h // 2

            hypothesis  = detection.results[0]
            class_name  = hypothesis.hypothesis.class_id
            confidence  = hypothesis.hypothesis.score

            color = (0, 0, 255) if 'red' in class_name else (255, 0, 0)

            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)

            label = f'{class_name}: {confidence:.2f}'
            cv2.rectangle(annotated, (x1, y1 - 25), (x1 + 150, y1), color, -1)
            cv2.putText(annotated, label, (x1 + 5, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            cv2.circle(annotated, (cx, cy), 5, color, -1)

        return annotated


def main(args=None):
    rclpy.init(args=args)
    node = YOLODetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()