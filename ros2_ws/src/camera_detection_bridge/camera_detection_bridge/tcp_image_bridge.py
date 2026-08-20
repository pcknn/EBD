#!/usr/bin/env python3

import socket
from copy import deepcopy

import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory

from camera_info_manager import CameraInfoManager
from sensor_msgs.msg import CameraInfo, CompressedImage


HOST = "127.0.0.1"
PORT = 5556

# Calibrated C922 optical frame used by the camera/LiDAR transform.
FRAME_ID = "c922_camera_optical_frame"

CAMERA_NAME = "c922_prototype_v1"

PACKAGE_SHARE_DIR = get_package_share_directory(
    "camera_detection_bridge"
)

CALIB_URL = (
    "file://"
    + PACKAGE_SHARE_DIR
    + "/config/c922_prototype_v1.yaml"
)


class TcpImageBridge(Node):

    def __init__(self):
        super().__init__("tcp_image_bridge")

        self.image_publisher = self.create_publisher(
            CompressedImage,
            "/camera/image/compressed",
            5,
        )

        self.camera_info_publisher = self.create_publisher(
            CameraInfo,
            "/camera/camera_info",
            5,
        )

        self.camera_info_manager = CameraInfoManager(
            node=self,
            cname=CAMERA_NAME,
            url=CALIB_URL,
        )

        self.camera_info_manager.loadCameraInfo()

        if not self.camera_info_manager.isCalibrated():
            raise RuntimeError(
                f"Camera calibration could not be loaded from {CALIB_URL}"
            )

        self.sock = None
        self.buffer = bytearray()

        self.timer = self.create_timer(0.005, self.poll)

        self.get_logger().info(
            f"Loaded calibration for {CAMERA_NAME}"
        )

        self.get_logger().info(
            f"Waiting for C922 MJPEG stream at tcp://{HOST}:{PORT}"
        )

        self.get_logger().info(
            "Publishing sensor_msgs/CompressedImage on "
            "/camera/image/compressed"
        )

        self.get_logger().info(
            "Publishing sensor_msgs/CameraInfo on "
            "/camera/camera_info"
        )

    def ensure_connected(self):
        if self.sock is not None:
            return True

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(0.05)
            sock.connect((HOST, PORT))
            sock.setblocking(False)

            self.sock = sock
            self.buffer.clear()

            self.get_logger().info(
                f"Connected to tcp://{HOST}:{PORT}"
            )
            return True

        except (ConnectionRefusedError, TimeoutError, OSError):
            try:
                sock.close()
            except Exception:
                pass
            return False

    def disconnect(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass

        self.sock = None
        self.buffer.clear()

    def publish_jpeg(self, jpeg):
        # One timestamp is generated for this received JPEG and reused
        # unchanged for both the image and its CameraInfo.
        stamp = self.get_clock().now().to_msg()

        image_msg = CompressedImage()
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = FRAME_ID
        image_msg.format = "jpeg"
        image_msg.data = bytes(jpeg)

        camera_info_msg = deepcopy(
            self.camera_info_manager.getCameraInfo()
        )
        camera_info_msg.header.stamp = stamp
        camera_info_msg.header.frame_id = FRAME_ID

        self.image_publisher.publish(image_msg)
        self.camera_info_publisher.publish(camera_info_msg)

    def extract_frames(self):
        while True:
            start = self.buffer.find(b"\xff\xd8")

            if start < 0:
                if len(self.buffer) > 2_000_000:
                    self.buffer.clear()
                return

            if start > 0:
                del self.buffer[:start]

            end = self.buffer.find(b"\xff\xd9", 2)

            if end < 0:
                return

            end += 2
            jpeg = self.buffer[:end]
            del self.buffer[:end]

            self.publish_jpeg(jpeg)

    def poll(self):
        if not self.ensure_connected():
            return

        try:
            while True:
                chunk = self.sock.recv(65536)

                if not chunk:
                    self.disconnect()
                    return

                self.buffer.extend(chunk)
                self.extract_frames()

        except BlockingIOError:
            pass
        except (ConnectionResetError, OSError):
            self.disconnect()

    def destroy_node(self):
        self.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = TcpImageBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
