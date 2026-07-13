# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

"""Shared file for the ``autonomy_datasets.launch.py`` integration tests.

``DatasetNodeTestBase`` launches the dataset node exactly as ``ros2 launch autonomy_datasets
autonomy_datasets.launch.py dataset:=<dataset>`` would (visualization disabled so it runs
headless), and offers helpers to inspect the topics it advertises and publishes. The node's
``wait_for_ack`` mechanism only starts publishing once every publisher has a subscriber, so
subscribing to all advertised topics also drives playback forward.
"""

import os
import signal
import subprocess
import unittest

import rclpy
from perception_msgs.msg import EgoData, ObjectList
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_msgs.msg import TFMessage

# Name/namespace the launch file assigns to the dataset node by default.
NODE_NAME = "datasets"
NODE_NAMESPACE = "/"

# Publishers every ROS node advertises; not part of the dataset output.
INFRA_TOPICS = {"/parameter_events", "/rosout"}

# Message type strings (as reported by the graph) mapped to their python classes.
TYPE_MAP = {
    "rosgraph_msgs/msg/Clock": Clock,
    "tf2_msgs/msg/TFMessage": TFMessage,
    "perception_msgs/msg/EgoData": EgoData,
    "perception_msgs/msg/ObjectList": ObjectList,
    "sensor_msgs/msg/Image": Image,
    "sensor_msgs/msg/CameraInfo": CameraInfo,
    "sensor_msgs/msg/PointCloud2": PointCloud2,
}

DISCOVERY_TIMEOUT_S = 180.0
MESSAGE_TIMEOUT_S = 180.0
SHUTDOWN_TIMEOUT_S = 20.0


class DatasetNodeTestBase(unittest.TestCase):
    """Shared machinery for launching the dataset node and inspecting its published topics."""

    __test__ = False

    DATASET: str = ""

    def setUp(self):
        """Skip when the dataset is unavailable, then create the subscriber node."""
        if not self.DATASET:
            self.skipTest("abstract base class")
        # Skip gracefully when the raw dataset is not mounted (e.g. CI without data access),
        # so the rest of the pipeline still passes. DATASETS_PATH mirrors the launch default.
        self.datasets_path = os.environ.get("DATASETS_PATH", "/datasets")
        dataset_dir = os.path.join(self.datasets_path, self.DATASET)
        if not os.path.isdir(dataset_dir) or not os.listdir(dataset_dir):
            self.skipTest(f"Dataset '{self.DATASET}' not available at '{dataset_dir}'")
        self._processes = []
        rclpy.init()
        self.node = rclpy.create_node("published_topics_test_subscriber")

    def tearDown(self):
        """Tear down the subscriber node and terminate any launched nodes."""
        if getattr(self, "node", None) is not None:
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for proc in getattr(self, "_processes", []):
            self._terminate_launch(proc)

    def _launch(self, log_path=None, config="", **overrides):
        """Launch the dataset node headless and return the process.

        ``overrides`` replace individual launch arguments; ``config`` selects a parameter file;
        ``log_path`` captures the node's stdout/stderr for later inspection.
        """
        args = {
            "rviz": "no",
            "write_rosbag": "false",
            "overwrite_rosbag": "true",
            "publish_samples": "true",
            "wait_for_ack": "true",
            "target_frame_rate": "0.0",
            "loop": "false",
            "use_sim_time": "true",
        }
        args.update(overrides)
        cmd = [
            "ros2",
            "launch",
            "autonomy_datasets",
            "autonomy_datasets.launch.py",
            f"dataset:={self.DATASET}",
            f"datasets_path:={self.datasets_path}",
        ]
        if config:
            cmd.append(f"config:={config}")
        cmd += [f"{name}:={value}" for name, value in args.items()]

        log_file = open(log_path, "wb") if log_path else None
        proc = subprocess.Popen(
            cmd,
            stdout=log_file,
            stderr=subprocess.STDOUT if log_file else None,
            # own process group so the whole launch tree can be signalled on teardown
            start_new_session=True,
        )
        if log_file is not None:
            log_file.close()  # the child keeps its own copy of the file descriptor
        self._processes.append(proc)
        return proc

    def _terminate_launch(self, proc):
        """Send SIGINT to the launch process group, escalating to SIGKILL if needed."""
        if proc.poll() is not None:
            return
        pgid = os.getpgid(proc.pid)
        os.killpg(pgid, signal.SIGINT)
        try:
            proc.wait(timeout=SHUTDOWN_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            os.killpg(pgid, signal.SIGKILL)
            proc.wait(timeout=SHUTDOWN_TIMEOUT_S)

    def _spin_until(self, predicate, timeout_s):
        """Spin the test node until ``predicate()`` is true or the timeout elapses."""
        end = self.node.get_clock().now().nanoseconds + int(timeout_s * 1e9)
        while rclpy.ok() and self.node.get_clock().now().nanoseconds < end:
            if predicate():
                return True
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return predicate()

    def _discover_advertised_topics(self):
        """Return {topic: type_str} advertised by the dataset node, excluding infra topics."""
        advertised = {}
        try:
            names_and_types = self.node.get_publisher_names_and_types_by_node(NODE_NAME, NODE_NAMESPACE)
        except _rclpy.NodeNameNonExistentError:
            # Node has not finished starting up yet; report no topics so callers keep polling.
            return advertised
        for name, types in names_and_types:
            if name in INFRA_TOPICS:
                continue
            advertised[name] = types[0]
        return advertised

    def _assert_topics_published(self, expected_topics, capture_first=()):
        """Assert the node advertises exactly ``expected_topics`` and publishes on each.

        Subscribes to every advertised topic (which also releases the node's ``wait_for_ack``
        gate). For topics listed in ``capture_first`` the first received message is returned in
        a ``{topic: message}`` dict.
        """
        # 1. Wait for the dataset node to advertise all expected publishers.
        self.assertTrue(
            self._spin_until(
                lambda: set(self._discover_advertised_topics()) >= set(expected_topics),
                DISCOVERY_TIMEOUT_S,
            ),
            msg=(
                f"Dataset node did not advertise the expected topics for '{self.DATASET}' in time.\n"
                f"Expected: {sorted(expected_topics)}\n"
                f"Advertised: {sorted(self._discover_advertised_topics())}"
            ),
        )

        advertised = self._discover_advertised_topics()

        # 2. The set of advertised topics must equal the requested set.
        self.assertEqual(
            set(advertised),
            set(expected_topics),
            msg="Advertised topics do not match the requested topics",
        )

        # 3. The advertised message type must match the expected type for each topic.
        for topic, expected_type in expected_topics.items():
            self.assertIn(advertised[topic], TYPE_MAP, f"Unexpected message type for '{topic}'")
            self.assertIs(
                TYPE_MAP[advertised[topic]],
                expected_type,
                f"Topic '{topic}' has type '{advertised[topic]}', expected {expected_type.__name__}",
            )

        # 4. Subscribe to every advertised topic, recording receipt and the first message.
        received = set()
        first_messages = {}
        subscriptions = []
        # Match the publisher QoS (RELIABLE / VOLATILE / KEEP_LAST depth 1).
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        def make_callback(topic):
            def callback(msg):
                received.add(topic)
                if topic in capture_first and topic not in first_messages:
                    first_messages[topic] = msg

            return callback

        for topic, type_str in advertised.items():
            subscriptions.append(self.node.create_subscription(TYPE_MAP[type_str], topic, make_callback(topic), qos))

        try:
            # 5. Wait until a message has been received on every requested topic.
            all_received = self._spin_until(lambda: received >= set(expected_topics), MESSAGE_TIMEOUT_S)
            missing = set(expected_topics) - received
            self.assertTrue(all_received, msg=f"No message received on topics: {sorted(missing)}")
        finally:
            for sub in subscriptions:
                self.node.destroy_subscription(sub)

        return first_messages
