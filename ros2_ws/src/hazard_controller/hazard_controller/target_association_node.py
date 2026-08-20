import math
from collections import deque

import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from sensor_msgs.msg import CameraInfo, LaserScan
from std_msgs.msg import Bool, Float32, String
from vision_msgs.msg import Detection2DArray

from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_to_matrix(x, y, z, w):
    n = x * x + y * y + z * z + w * w

    if n < 1e-12:
        return np.eye(3)

    s = 2.0 / n

    xx = x * x * s
    yy = y * y * s
    zz = z * z * s
    xy = x * y * s
    xz = x * z * s
    yz = y * z * s
    wx = w * x * s
    wy = w * y * s
    wz = w * z * s

    return np.array([
        [1.0 - yy - zz, xy - wz,       xz + wy],
        [xy + wz,       1.0 - xx - zz, yz - wx],
        [xz - wy,       yz + wx,       1.0 - xx - yy],
    ], dtype=np.float64)


def angle_difference_deg(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


class TargetAssociationNode(Node):

    def __init__(self):
        super().__init__('target_association_node')

        # Keep the experimentally validated association window configurable.
        self.declare_parameter('association_window_deg', 2.0)
        self.declare_parameter('min_confidence', 0.50)
        self.declare_parameter('min_candidate_count', 3)
        self.declare_parameter('detection_timeout_sec', 0.50)
        self.declare_parameter('report_period_sec', 0.50)

        # Closing-speed estimator.
        # Positive = approaching.
        # Negative = moving away.
        self.declare_parameter('closing_window_sec', 0.80)
        self.declare_parameter('closing_min_samples', 5)
        self.declare_parameter('closing_min_span_sec', 0.35)

        # Prototype same-target continuity guard.
        #
        # These deliberately generous limits prevent obvious
        # same-class target switches from contaminating the
        # closing-speed regression.
        self.declare_parameter(
            'continuity_max_distance_jump_m',
            0.20,
        )
        self.declare_parameter(
            'continuity_max_bearing_jump_deg',
            10.0,
        )

        # Prototype TTC gate.
        # Ignore tiny positive closing-speed noise.
        self.declare_parameter(
            'ttc_min_closing_speed_mps',
            0.05,
        )

        self.K = None
        self.D = None

        self.latest_detections = None
        self.latest_detection_rx_ns = None

        self.last_report_ns = 0

        self.closing_history = deque()
        self.closing_tracked_class = None
        self.closing_tracked_distance_m = None
        self.closing_tracked_bearing_deg = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            qos_profile_sensor_data,
        )

        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            qos_profile_sensor_data,
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.target_valid_pub = self.create_publisher(
            Bool,
            '/target_valid',
            10,
        )

        self.target_class_pub = self.create_publisher(
            String,
            '/target_class',
            10,
        )

        self.target_distance_pub = self.create_publisher(
            Float32,
            '/target_distance',
            10,
        )

        self.target_bearing_pub = self.create_publisher(
            Float32,
            '/target_bearing',
            10,
        )

        self.closing_speed_pub = self.create_publisher(
            Float32,
            '/closing_speed',
            10,
        )

        self.time_to_collision_pub = self.create_publisher(
            Float32,
            '/time_to_collision',
            10,
        )

        self.get_logger().info(
            'Target association node started: '
            '/detections + /camera/camera_info + /scan + TF '
            '-> /target_valid + /target_class + '
            '/target_distance + /target_bearing'
        )

        self.get_logger().info(
            '/target_distance is meters; '
            '/target_bearing is camera horizontal bearing in degrees'
        )

    def camera_info_callback(self, msg):
        if msg.k[0] == 0.0:
            return

        self.K = np.array(
            msg.k,
            dtype=np.float64,
        ).reshape(3, 3)

        self.D = np.array(
            msg.d,
            dtype=np.float64,
        )

    def detection_callback(self, msg):
        # Preserve the most recent QUALIFYING detection through brief
        # detector dropouts. Empty or low-confidence frames do not
        # immediately erase a valid target; scan_callback expires the
        # cached detection using detection_timeout_sec.

        min_confidence = float(
            self.get_parameter(
                'min_confidence'
            ).value
        )

        qualifying_detections = []

        for detection in msg.detections:
            if not detection.results:
                continue

            best_result = max(
                detection.results,
                key=lambda item:
                    item.hypothesis.score,
            )

            if (
                float(best_result.hypothesis.score)
                < min_confidence
            ):
                continue

            qualifying_detections.append(
                detection
            )

        if not qualifying_detections:
            return

        cached_msg = Detection2DArray()
        cached_msg.header = msg.header
        cached_msg.detections = qualifying_detections

        self.latest_detections = cached_msg
        self.latest_detection_rx_ns = (
            self.get_clock().now().nanoseconds
        )

    def reset_closing_history(self):
        self.closing_history.clear()
        self.closing_tracked_class = None
        self.closing_tracked_distance_m = None
        self.closing_tracked_bearing_deg = None

    def get_scan_time_sec(self, scan):
        stamp_sec = (
            float(scan.header.stamp.sec)
            + float(scan.header.stamp.nanosec) * 1e-9
        )

        # Defensive fallback if a driver ever provides a zero stamp.
        if stamp_sec <= 0.0:
            stamp_sec = (
                self.get_clock().now().nanoseconds
                / 1e9
            )

        return stamp_sec

    def compute_closing_speed(self, target, scan):
        class_id = target['class_id']
        distance_m = float(target['distance_m'])
        bearing_deg = float(
            target['camera_bearing_deg']
        )
        stamp_sec = self.get_scan_time_sec(scan)

        # Do not calculate velocity across a class switch.
        if (
            self.closing_tracked_class is not None
            and class_id != self.closing_tracked_class
        ):
            self.reset_closing_history()

        # Detection2D.id is not populated by the current bridge,
        # so Prototype_V1 uses a small physical-continuity guard
        # for same-class target switches.
        if (
            self.closing_tracked_class == class_id
            and self.closing_tracked_distance_m is not None
            and self.closing_tracked_bearing_deg is not None
        ):
            distance_jump_m = abs(
                distance_m
                - self.closing_tracked_distance_m
            )

            bearing_jump_deg = abs(
                angle_difference_deg(
                    bearing_deg,
                    self.closing_tracked_bearing_deg,
                )
            )

            max_distance_jump_m = float(
                self.get_parameter(
                    'continuity_max_distance_jump_m'
                ).value
            )

            max_bearing_jump_deg = float(
                self.get_parameter(
                    'continuity_max_bearing_jump_deg'
                ).value
            )

            if (
                distance_jump_m > max_distance_jump_m
                or bearing_jump_deg > max_bearing_jump_deg
            ):
                self.reset_closing_history()

        self.closing_tracked_class = class_id
        self.closing_tracked_distance_m = distance_m
        self.closing_tracked_bearing_deg = bearing_deg

        # Protect the regression from non-monotonic timestamps.
        if (
            self.closing_history
            and stamp_sec <= self.closing_history[-1][0]
        ):
            self.reset_closing_history()
            self.closing_tracked_class = class_id
            self.closing_tracked_distance_m = distance_m
            self.closing_tracked_bearing_deg = bearing_deg

        self.closing_history.append(
            (stamp_sec, distance_m)
        )

        window_sec = float(
            self.get_parameter(
                'closing_window_sec'
            ).value
        )

        while (
            self.closing_history
            and
            stamp_sec - self.closing_history[0][0]
            > window_sec
        ):
            self.closing_history.popleft()

        min_samples = int(
            self.get_parameter(
                'closing_min_samples'
            ).value
        )

        if len(self.closing_history) < min_samples:
            return None

        samples = list(self.closing_history)

        span_sec = samples[-1][0] - samples[0][0]

        min_span_sec = float(
            self.get_parameter(
                'closing_min_span_sec'
            ).value
        )

        if span_sec < min_span_sec:
            return None

        times = [
            sample[0] - samples[0][0]
            for sample in samples
        ]

        distances = [
            sample[1]
            for sample in samples
        ]

        mean_time = sum(times) / len(times)
        mean_distance = (
            sum(distances) / len(distances)
        )

        numerator = sum(
            (time_value - mean_time)
            * (distance_value - mean_distance)
            for time_value, distance_value
            in zip(times, distances)
        )

        denominator = sum(
            (time_value - mean_time) ** 2
            for time_value in times
        )

        if denominator <= 1e-12:
            return None

        distance_slope_mps = (
            numerator / denominator
        )

        # Distance decreases while approaching,
        # therefore invert the distance slope:
        #
        # positive closing speed = approaching
        # negative closing speed = opening/receding
        return -float(distance_slope_mps)

    def publish_invalid(self):
        self.reset_closing_history()

        valid_msg = Bool()
        valid_msg.data = False
        self.target_valid_pub.publish(valid_msg)

        closing_msg = Float32()
        closing_msg.data = float('nan')
        self.closing_speed_pub.publish(closing_msg)

        ttc_msg = Float32()
        ttc_msg.data = float('nan')
        self.time_to_collision_pub.publish(ttc_msg)

    def scan_callback(self, scan):
        if self.K is None or self.D is None:
            self.publish_invalid()
            return

        if (
            self.latest_detections is None
            or self.latest_detection_rx_ns is None
        ):
            self.publish_invalid()
            return

        now_ns = self.get_clock().now().nanoseconds

        detection_timeout_sec = float(
            self.get_parameter(
                'detection_timeout_sec'
            ).value
        )

        detection_age_sec = (
            now_ns - self.latest_detection_rx_ns
        ) / 1e9

        if detection_age_sec > detection_timeout_sec:
            self.publish_invalid()
            return

        try:
            # Transform LiDAR points into the C922 optical frame.
            tf = self.tf_buffer.lookup_transform(
                'c922_camera_optical_frame',
                'laser',
                Time(),
            )

        except TransformException:
            self.publish_invalid()
            return

        q = tf.transform.rotation

        rotation = quaternion_to_matrix(
            q.x,
            q.y,
            q.z,
            q.w,
        )

        translation = np.array([
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
        ], dtype=np.float64)

        lidar_points = []

        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue

            if distance < scan.range_min:
                continue

            if distance > scan.range_max:
                continue

            laser_angle = (
                scan.angle_min
                + index * scan.angle_increment
            )

            point_laser = np.array([
                distance * math.cos(laser_angle),
                distance * math.sin(laser_angle),
                0.0,
            ])

            point_camera = (
                rotation @ point_laser
                + translation
            )

            # Optical +Z is forward.
            if point_camera[2] <= 0.0:
                continue

            projected_bearing_deg = math.degrees(
                math.atan2(
                    point_camera[0],
                    point_camera[2],
                )
            )

            lidar_points.append({
                'range_m': float(distance),
                'laser_angle_deg':
                    math.degrees(laser_angle),
                'projected_bearing_deg':
                    projected_bearing_deg,
            })

        if not lidar_points:
            self.publish_invalid()
            return

        window_deg = float(
            self.get_parameter(
                'association_window_deg'
            ).value
        )

        min_confidence = float(
            self.get_parameter(
                'min_confidence'
            ).value
        )

        min_candidate_count = int(
            self.get_parameter(
                'min_candidate_count'
            ).value
        )

        associations = []

        for detection in self.latest_detections.detections:
            if not detection.results:
                continue

            # Use the highest-scoring classification hypothesis.
            result = max(
                detection.results,
                key=lambda item:
                    item.hypothesis.score,
            )

            class_id = result.hypothesis.class_id
            confidence = float(
                result.hypothesis.score
            )

            if confidence < min_confidence:
                continue

            u = float(
                detection.bbox.center.position.x
            )

            v = float(
                detection.bbox.center.position.y
            )

            pixel = np.array(
                [[[u, v]]],
                dtype=np.float64,
            )

            normalized = cv2.undistortPoints(
                pixel,
                self.K,
                self.D,
            )[0, 0]

            camera_bearing_deg = math.degrees(
                math.atan2(
                    float(normalized[0]),
                    1.0,
                )
            )

            candidates = []

            for point in lidar_points:
                error_deg = angle_difference_deg(
                    point['projected_bearing_deg'],
                    camera_bearing_deg,
                )

                if abs(error_deg) <= window_deg:
                    candidates.append(point)

            if len(candidates) < min_candidate_count:
                continue

            ranges = np.array([
                point['range_m']
                for point in candidates
            ])

            projected_bearings = np.array([
                point['projected_bearing_deg']
                for point in candidates
            ])

            laser_angles = np.array([
                point['laser_angle_deg']
                for point in candidates
            ])

            associations.append({
                'class_id': class_id,
                'confidence': confidence,
                'camera_bearing_deg':
                    camera_bearing_deg,
                'distance_m':
                    float(np.median(ranges)),
                'projected_bearing_deg':
                    float(np.median(
                        projected_bearings
                    )),
                'laser_angle_deg':
                    float(np.median(
                        laser_angles
                    )),
                'candidate_count':
                    len(candidates),
            })

        if not associations:
            self.publish_invalid()
            return

        # Prototype_V1 target selection:
        # choose the closest successfully associated relevant target.
        target = min(
            associations,
            key=lambda item:
                item['distance_m'],
        )

        closing_speed = self.compute_closing_speed(
            target,
            scan,
        )

        ttc_sec = None

        if closing_speed is not None:
            min_closing_speed_mps = float(
                self.get_parameter(
                    'ttc_min_closing_speed_mps'
                ).value
            )

            if closing_speed > min_closing_speed_mps:
                ttc_sec = (
                    float(target['distance_m'])
                    / closing_speed
                )

        valid_msg = Bool()
        valid_msg.data = True

        class_msg = String()
        class_msg.data = target['class_id']

        distance_msg = Float32()
        distance_msg.data = target['distance_m']

        bearing_msg = Float32()
        bearing_msg.data = target[
            'camera_bearing_deg'
        ]

        closing_msg = Float32()

        if closing_speed is None:
            # Estimator is still filling its rolling window.
            closing_msg.data = float('nan')
        else:
            closing_msg.data = closing_speed

        ttc_msg = Float32()

        if ttc_sec is None:
            # Invalid, warming up, stationary, or receding.
            ttc_msg.data = float('nan')
        else:
            ttc_msg.data = ttc_sec

        self.target_valid_pub.publish(valid_msg)
        self.target_class_pub.publish(class_msg)
        self.target_distance_pub.publish(
            distance_msg
        )
        self.target_bearing_pub.publish(
            bearing_msg
        )

        self.closing_speed_pub.publish(
            closing_msg
        )

        self.time_to_collision_pub.publish(
            ttc_msg
        )

        report_period_sec = float(
            self.get_parameter(
                'report_period_sec'
            ).value
        )

        if (
            now_ns - self.last_report_ns
            < report_period_sec * 1e9
        ):
            return

        self.last_report_ns = now_ns

        self.get_logger().info(
            f'Target: {target["class_id"]} | '
            f'confidence={target["confidence"]:.3f} | '
            f'distance={target["distance_m"]:.3f} m | '
            f'camera_bearing='
            f'{target["camera_bearing_deg"]:+.2f} deg | '
            f'projected_lidar='
            f'{target["projected_bearing_deg"]:+.2f} deg | '
            f'raw_lidar_angle='
            f'{target["laser_angle_deg"]:+.2f} deg | '
            f'candidates={target["candidate_count"]}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = TargetAssociationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
