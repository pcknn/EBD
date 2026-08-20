#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from vision_msgs.msg import Detection2DArray

from foxglove_msgs.msg import (
    Color,
    ImageAnnotations,
    Point2,
    PointsAnnotation,
    TextAnnotation,
)


IMAGE_WIDTH = 1280.0
IMAGE_HEIGHT = 720.0


class DetectionAnnotationBridge(Node):

    def __init__(self):
        super().__init__("detection_annotation_bridge")

        self.publisher = self.create_publisher(
            ImageAnnotations,
            "/camera/annotations",
            10,
        )

        self.subscription = self.create_subscription(
            Detection2DArray,
            "/detections",
            self.detections_callback,
            10,
        )

        self.get_logger().info(
            "Listening to filtered /detections"
        )
        self.get_logger().info(
            "Publishing Foxglove overlays on /camera/annotations"
        )

    @staticmethod
    def color(r, g, b, a=1.0):
        c = Color()
        c.r = r
        c.g = g
        c.b = b
        c.a = a
        return c

    @staticmethod
    def point(x, y):
        p = Point2()
        p.x = float(x)
        p.y = float(y)
        return p

    def detections_callback(self, msg):
        annotations = ImageAnnotations()

        # Use the detection timestamp for annotation synchronization.
        annotations.timestamp = msg.header.stamp

        for detection in msg.detections:
            if not detection.results:
                continue

            hypothesis = detection.results[0].hypothesis

            label = hypothesis.class_id
            score = float(hypothesis.score)

            cx = float(detection.bbox.center.position.x)
            cy = float(detection.bbox.center.position.y)

            width = float(detection.bbox.size_x)
            height = float(detection.bbox.size_y)

            x1 = max(0.0, min(IMAGE_WIDTH, cx - width / 2.0))
            y1 = max(0.0, min(IMAGE_HEIGHT, cy - height / 2.0))
            x2 = max(0.0, min(IMAGE_WIDTH, cx + width / 2.0))
            y2 = max(0.0, min(IMAGE_HEIGHT, cy + height / 2.0))

            # Detection bounding box.
            box = PointsAnnotation()
            box.type = PointsAnnotation.LINE_LOOP

            box.points = [
                self.point(x1, y1),
                self.point(x2, y1),
                self.point(x2, y2),
                self.point(x1, y2),
            ]

            box.outline_color = self.color(
                0.1, 1.0, 0.1, 1.0
            )
            box.fill_color = self.color(
                0.0, 0.0, 0.0, 0.0
            )
            box.thickness = 3.0

            annotations.points.append(box)

            # Human-readable class/confidence label.
            text = TextAnnotation()

            text.position = self.point(
                x1,
                max(20.0, y1),
            )

            text.text = f"{label} {score:.0%}"
            text.font_size = 18.0

            text.text_color = self.color(
                1.0, 1.0, 1.0, 1.0
            )

            text.background_color = self.color(
                0.0, 0.0, 0.0, 0.75
            )

            annotations.texts.append(text)

        self.publisher.publish(annotations)


def main(args=None):
    rclpy.init(args=args)

    node = DetectionAnnotationBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
