"""Verify that the demo path waits for odometry and publishes only once."""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)


@pytest.mark.launch_test
def generate_test_description():
    publisher = launch_ros.actions.Node(
        package="scan_planner",
        executable="reference_path_publisher.py",
        name="reference_path_publisher",
        parameters=[
            {
                "frame_id": "world",
                "publish_delay_sec": 0.1,
                "waypoints": [0.0, 0.0, 0.0, 1.0, 0.0, 0.5],
            }
        ],
        output="screen",
    )
    return (
        launch.LaunchDescription(
            [
                publisher,
                launch_testing.actions.ReadyToTest(),
                launch.actions.TimerAction(
                    period=5.0,
                    actions=[launch.actions.Shutdown(reason="publisher test complete")],
                ),
            ]
        ),
        {"publisher": publisher},
    )


class TestReferencePathPublisher(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)

    @classmethod
    def tearDownClass(cls):
        cls.context.shutdown()

    def test_waits_for_odom_and_publishes_once(self, proc_info, publisher):
        proc_info.assertWaitForStartup(process=publisher, timeout=10)
        node = rclpy.create_node(
            "reference_path_publisher_test", context=self.context
        )
        received = []
        path_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        subscription = node.create_subscription(
            Path, "initial_path", received.append, path_qos
        )
        odom_publisher = node.create_publisher(
            Odometry, "body_pose", qos_profile_sensor_data
        )
        executor = SingleThreadedExecutor(context=self.context)
        executor.add_node(node)
        try:
            deadline = time.monotonic() + 0.4
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
            self.assertEqual(received, [])

            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline and not received:
                odom_publisher.publish(Odometry())
                executor.spin_once(timeout_sec=0.05)
            self.assertEqual(len(received), 1)
            self.assertEqual(len(received[0].poses), 2)
            self.assertEqual(received[0].header.frame_id, "world")
            self.assertAlmostEqual(received[0].poses[-1].pose.position.x, 1.0)
            self.assertAlmostEqual(received[0].poses[-1].pose.position.z, 0.5)

            deadline = time.monotonic() + 0.5
            while time.monotonic() < deadline:
                odom_publisher.publish(Odometry())
                executor.spin_once(timeout_sec=0.05)
            self.assertEqual(len(received), 1)
        finally:
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_subscription(subscription)
            node.destroy_publisher(odom_publisher)
            node.destroy_node()
