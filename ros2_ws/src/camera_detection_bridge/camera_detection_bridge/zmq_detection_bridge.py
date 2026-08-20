#!/usr/bin/env python3

import json

import rclpy
from rclpy.node import Node
import zmq

from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)


ZMQ_ADDRESS = "tcp://127.0.0.1:5555"

IMAGE_WIDTH = 1280.0
IMAGE_HEIGHT = 720.0
FRAME_ID = "c922_camera_optical_frame"


class ZmqDetectionBridge(Node):

    def __init__(self):
        super().__init__("zmq_detection_bridge")

        # Classes used by the robot by default.
        self.declare_parameter(
            "allowed_classes",
            [
                "person",
                "car",
                "truck",
                "bus",
                "bicycle",
                "motorcycle",
                "dog",
            ],
        )

        self.declare_parameter("min_confidence", 0.50)
        self.declare_parameter("publish_raw", True)

        self.filtered_publisher = self.create_publisher(
            Detection2DArray,
            "/detections",
            10,
        )

        self.raw_publisher = self.create_publisher(
            Detection2DArray,
            "/detections_raw",
            10,
        )

        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.SUB)
        self.zmq_socket.setsockopt(zmq.SUBSCRIBE, b"")
        self.zmq_socket.connect(ZMQ_ADDRESS)

        self.timer = self.create_timer(0.005, self.poll_zmq)

        self.get_logger().info(
            f"Listening for Hailo detections on {ZMQ_ADDRESS}"
        )

        self.get_logger().info(
            "Filtered detections -> /detections"
        )

        self.get_logger().info(
            "Raw detections -> /detections_raw"
        )

        self.log_filter_settings()

    def log_filter_settings(self):
        allowed = self.get_parameter("allowed_classes").value
        threshold = self.get_parameter("min_confidence").value

        self.get_logger().info(
            f"Allowed classes: {', '.join(allowed)}"
        )
        self.get_logger().info(
            f"Minimum confidence: {threshold:.2f}"
        )

    @staticmethod
    def set_stamp_from_ms(header, timestamp_ms):
        timestamp_ms = int(timestamp_ms)

        header.stamp.sec = timestamp_ms // 1000
        header.stamp.nanosec = (timestamp_ms % 1000) * 1_000_000
        header.frame_id = FRAME_ID

    @staticmethod
    def make_detection(hailo_det, header):
        bbox = hailo_det.get("HailoBBox", {})

        xmin = float(bbox.get("xmin", 0.0))
        ymin = float(bbox.get("ymin", 0.0))
        width = float(bbox.get("width", 0.0))
        height = float(bbox.get("height", 0.0))

        detection = Detection2D()
        detection.header = header

        detection.bbox.center.position.x = (
            xmin + width / 2.0
        ) * IMAGE_WIDTH

        detection.bbox.center.position.y = (
            ymin + height / 2.0
        ) * IMAGE_HEIGHT

        detection.bbox.center.theta = 0.0
        detection.bbox.size_x = width * IMAGE_WIDTH
        detection.bbox.size_y = height * IMAGE_HEIGHT

        hypothesis = ObjectHypothesisWithPose()

        hypothesis.hypothesis.class_id = str(
            hailo_det.get("label", "unknown")
        )

        hypothesis.hypothesis.score = float(
            hailo_det.get("confidence", 0.0)
        )

        detection.results.append(hypothesis)
        detection.id = ""

        return detection

    def poll_zmq(self):
        while True:
            try:
                raw = self.zmq_socket.recv(flags=zmq.NOBLOCK)
            except zmq.Again:
                return

            try:
                data = json.loads(raw.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                self.get_logger().warning(
                    f"Invalid Hailo ZMQ JSON: {exc}"
                )
                continue

            self.publish_detection_messages(data)

    def publish_detection_messages(self, data):
        raw_output = Detection2DArray()
        filtered_output = Detection2DArray()

        timestamp_ms = data.get("timestamp (ms)")

        if timestamp_ms is None:
            stamp = self.get_clock().now().to_msg()

            raw_output.header.stamp = stamp
            raw_output.header.frame_id = FRAME_ID

            filtered_output.header.stamp = stamp
            filtered_output.header.frame_id = FRAME_ID
        else:
            self.set_stamp_from_ms(
                raw_output.header,
                timestamp_ms,
            )
            self.set_stamp_from_ms(
                filtered_output.header,
                timestamp_ms,
            )

        allowed_classes = {
            str(label).lower()
            for label in self.get_parameter(
                "allowed_classes"
            ).value
        }

        min_confidence = float(
            self.get_parameter("min_confidence").value
        )

        publish_raw = bool(
            self.get_parameter("publish_raw").value
        )

        subobjects = data.get(
            "HailoROI", {}
        ).get("SubObjects", [])

        for obj in subobjects:
            hailo_det = obj.get("HailoDetection")

            if hailo_det is None:
                continue

            label = str(
                hailo_det.get("label", "unknown")
            )

            confidence = float(
                hailo_det.get("confidence", 0.0)
            )

            detection = self.make_detection(
                hailo_det,
                raw_output.header,
            )

            raw_output.detections.append(detection)

            if (
                label.lower() in allowed_classes
                and confidence >= min_confidence
            ):
                filtered_output.detections.append(
                    detection
                )

        if publish_raw:
            self.raw_publisher.publish(raw_output)

        self.filtered_publisher.publish(filtered_output)

    def destroy_node(self):
        self.zmq_socket.close(0)
        self.zmq_context.term()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = ZmqDetectionBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
