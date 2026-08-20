#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import UInt8


GREEN = 0
BLUE = 1
YELLOW = 2
RED = 3


class FusionHazardNode(Node):

    def __init__(self):
        super().__init__('fusion_hazard_node')

        # Prototype_V1 provisional indoor thresholds.
        self.declare_parameter(
            'hazard_yellow_distance_m',
            1.00,
        )
        self.declare_parameter(
            'hazard_red_distance_m',
            0.60,
        )
        self.declare_parameter(
            'hazard_min_closing_speed_mps',
            0.05,
        )
        self.declare_parameter(
            'hazard_yellow_ttc_sec',
            6.0,
        )
        self.declare_parameter(
            'hazard_red_ttc_sec',
            3.0,
        )

        self.target_valid = None
        self.target_distance_m = None
        self.closing_speed_mps = None

        self.last_hazard_level = None

        self.target_valid_sub = self.create_subscription(
            Bool,
            '/target_valid',
            self.target_valid_callback,
            10,
        )

        self.target_distance_sub = self.create_subscription(
            Float32,
            '/target_distance',
            self.target_distance_callback,
            10,
        )

        self.closing_speed_sub = self.create_subscription(
            Float32,
            '/closing_speed',
            self.closing_speed_callback,
            10,
        )

        self.ttc_sub = self.create_subscription(
            Float32,
            '/time_to_collision',
            self.ttc_callback,
            10,
        )

        self.hazard_pub = self.create_publisher(
            UInt8,
            '/hazard_level',
            10,
        )

        self.get_logger().info(
            'Fusion hazard node started: '
            '/target_valid + /target_distance + '
            '/closing_speed + /time_to_collision '
            '-> /hazard_level'
        )

        self.get_logger().info(
            'Hazard states: '
            '0=GREEN, 1=BLUE, 2=YELLOW, 3=RED'
        )

    def target_valid_callback(self, msg):
        self.target_valid = bool(msg.data)

    def target_distance_callback(self, msg):
        self.target_distance_m = float(msg.data)

    def closing_speed_callback(self, msg):
        self.closing_speed_mps = float(msg.data)

    def calculate_hazard_level(self, ttc_sec):

        # No relevant associated target.
        if self.target_valid is not True:
            return GREEN

        # A valid associated target should have a finite distance.
        # If the distance has not arrived yet, do not publish a
        # decision from incomplete data.
        if (
            self.target_distance_m is None
            or not math.isfinite(self.target_distance_m)
        ):
            return None

        yellow_distance_m = float(
            self.get_parameter(
                'hazard_yellow_distance_m'
            ).value
        )

        red_distance_m = float(
            self.get_parameter(
                'hazard_red_distance_m'
            ).value
        )

        min_closing_speed_mps = float(
            self.get_parameter(
                'hazard_min_closing_speed_mps'
            ).value
        )

        yellow_ttc_sec = float(
            self.get_parameter(
                'hazard_yellow_ttc_sec'
            ).value
        )

        red_ttc_sec = float(
            self.get_parameter(
                'hazard_red_ttc_sec'
            ).value
        )

        closing_is_meaningful = (
            self.closing_speed_mps is not None
            and math.isfinite(self.closing_speed_mps)
            and
            self.closing_speed_mps
            > min_closing_speed_mps
        )

        ttc_is_valid = math.isfinite(ttc_sec)

        # Evaluate most urgent state first.
        if self.target_distance_m <= red_distance_m:
            return RED

        if (
            closing_is_meaningful
            and ttc_is_valid
            and ttc_sec <= red_ttc_sec
        ):
            return RED

        if self.target_distance_m <= yellow_distance_m:
            return YELLOW

        if (
            closing_is_meaningful
            and ttc_is_valid
            and ttc_sec <= yellow_ttc_sec
        ):
            return YELLOW

        return BLUE

    def ttc_callback(self, msg):
        ttc_sec = float(msg.data)

        hazard_level = self.calculate_hazard_level(
            ttc_sec
        )

        if hazard_level is None:
            return

        output = UInt8()
        output.data = hazard_level

        self.hazard_pub.publish(output)

        # Log only state changes so normal 10 Hz publishing
        # does not flood the terminal.
        if hazard_level != self.last_hazard_level:

            names = {
                GREEN: 'GREEN',
                BLUE: 'BLUE',
                YELLOW: 'YELLOW',
                RED: 'RED',
            }

            distance_text = (
                'unknown'
                if self.target_distance_m is None
                else f'{self.target_distance_m:.3f} m'
            )

            closing_text = (
                'unknown'
                if self.closing_speed_mps is None
                or not math.isfinite(
                    self.closing_speed_mps
                )
                else
                f'{self.closing_speed_mps:+.3f} m/s'
            )

            ttc_text = (
                'NaN'
                if not math.isfinite(ttc_sec)
                else f'{ttc_sec:.3f} s'
            )

            self.get_logger().info(
                f'Hazard -> '
                f'{names[hazard_level]} '
                f'({hazard_level}) | '
                f'distance={distance_text} | '
                f'closing={closing_text} | '
                f'TTC={ttc_text}'
            )

            self.last_hazard_level = hazard_level


def main(args=None):
    rclpy.init(args=args)

    node = FusionHazardNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
