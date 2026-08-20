from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Receives the C922 MJPEG stream from GStreamer over TCP.
            # Publishes:
            #   /camera/image/compressed
            #   /camera/camera_info
            Node(
                package="camera_detection_bridge",
                executable="tcp_image_bridge",
                name="tcp_image_bridge",
                output="screen",
            ),

            # Receives Hailo detection metadata over ZeroMQ.
            # Publishes:
            #   /detections
            #   /detections_raw
            Node(
                package="camera_detection_bridge",
                executable="zmq_detection_bridge",
                name="zmq_detection_bridge",
                output="screen",
            ),

            # Converts filtered detections into Foxglove image annotations.
            # Publishes:
            #   /camera/annotations
            Node(
                package="camera_detection_bridge",
                executable="detection_annotation_bridge",
                name="detection_annotation_bridge",
                output="screen",
            ),

            # Fixed physical relationship between the RPLIDAR C1 frame
            # and the calibrated C922 optical frame.
            #
            # Camera optical center relative to LiDAR:
            #   +100 mm forward
            #     3 mm right
            #   -100 mm vertical (below)
            #
            # Optical-frame orientation:
            #   +Z forward
            #   +X right
            #   +Y down
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="laser_to_c922_static_tf",
                arguments=[
                    "--x",
                    "0.100",
                    "--y",
                    "-0.003",
                    "--z",
                    "-0.100",
                    "--roll",
                    "-1.57079632679",
                    "--pitch",
                    "0.0",
                    "--yaw",
                    "-1.57079632679",
                    "--frame-id",
                    "laser",
                    "--child-frame-id",
                    "c922_camera_optical_frame",
                ],
                output="screen",
            ),
        ]
    )
