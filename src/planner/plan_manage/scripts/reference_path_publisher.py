#!/usr/bin/env python3
"""Publish one configured 3D reference path after odometry is available."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)


def parse_waypoints(values):
    if values is None or len(values) < 6 or len(values) % 3 != 0:
        raise ValueError("waypoints must contain at least two x,y,z triples")
    numbers = [float(value) for value in values]
    if not all(math.isfinite(value) for value in numbers):
        raise ValueError("waypoints must contain only finite values")
    return [tuple(numbers[index : index + 3]) for index in range(0, len(numbers), 3)]


def build_path_message(waypoints, frame_id, stamp):
    message = Path()
    message.header.frame_id = frame_id
    message.header.stamp = stamp
    for x, y, z in waypoints:
        pose = PoseStamped()
        message.poses.append(pose)
        pose.header = message.header
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
    return message


class ReferencePathPublisher(Node):
    def __init__(self):
        super().__init__("reference_path_publisher")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("waypoints", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("publish_delay_sec", 0.5)

        self.frame_id = self.get_parameter("frame_id").value
        self.waypoints = parse_waypoints(self.get_parameter("waypoints").value)
        self.publish_delay_sec = float(self.get_parameter("publish_delay_sec").value)
        if not self.frame_id:
            raise ValueError("frame_id must not be empty")
        if not math.isfinite(self.publish_delay_sec) or self.publish_delay_sec < 0.0:
            raise ValueError("publish_delay_sec must be finite and non-negative")

        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_publisher = self.create_publisher(Path, "initial_path", path_qos)
        self.odom_subscription = self.create_subscription(
            Odometry, "body_pose", self.odom_callback, qos_profile_sensor_data
        )
        self.first_odom_time = None
        self.published = False
        self.timer = self.create_timer(0.05, self.try_publish)
        self.get_logger().info(
            f"Waiting to publish {len(self.waypoints)} reference-path points"
        )

    def odom_callback(self, _message):
        if self.first_odom_time is None:
            self.first_odom_time = self.get_clock().now()

    def try_publish(self):
        if self.published or self.first_odom_time is None:
            return
        if self.path_publisher.get_subscription_count() == 0:
            return
        elapsed = (self.get_clock().now() - self.first_odom_time).nanoseconds / 1e9
        if elapsed < self.publish_delay_sec:
            return

        message = build_path_message(
            self.waypoints, self.frame_id, self.get_clock().now().to_msg()
        )
        self.path_publisher.publish(message)
        self.published = True
        self.timer.cancel()
        self.get_logger().info(
            f"Published reference path on initial_path with {len(message.poses)} points"
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ReferencePathPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ValueError) as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            rclpy.logging.get_logger("reference_path_publisher").error(str(error))
        if isinstance(error, ValueError):
            raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
