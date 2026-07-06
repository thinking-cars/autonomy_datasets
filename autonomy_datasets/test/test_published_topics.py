# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

"""Integration tests verifying that ``autonomy_datasets.launch.py`` publishes its topics.

Each test launches the node exactly as ``ros2 launch autonomy_datasets
autonomy_datasets.launch.py dataset:=<dataset>`` would, but with visualization and rosbag
writing disabled so it can run headless. A helper node discovers the topics advertised by
the ``datasets`` node, subscribes to all of them and checks that:

* the set of advertised topics equals the set expected from the enabled ``publish_*``
  parameters (the *requested* topics), and
* at least one message is actually received on every one of those topics.
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


def _camera_topics(num_cameras):
    """Return {topic: type} for camera_01..camera_<num_cameras> image and info topics."""
    topics = {}
    for i in range(1, num_cameras + 1):
        topics[f"/camera_{i:02d}/image_raw"] = Image
        topics[f"/camera_{i:02d}/camera_info"] = CameraInfo
    return topics


# Topics each dataset is expected to publish when every publish_* parameter is enabled
_BASE_TOPICS = {
    "/clock": Clock,
    "/tf": TFMessage,
    "/tf_static": TFMessage,
    "/ego_data": EgoData,
}
EXPECTED_TOPICS_BY_DATASET = {
    "nvidia_physicalai_av_dataset": {
        **_BASE_TOPICS,
        "/object_list/lidar_01": ObjectList,
        "/lidar_01/point_cloud": PointCloud2,
        "/radar_01/point_cloud": PointCloud2,
        **_camera_topics(7),
    },
    "nuscenes": {
        **_BASE_TOPICS,
        "/object_list/lidar_01": ObjectList,
        "/object_list/camera_01": ObjectList,
        "/lidar_01/point_cloud": PointCloud2,
        **_camera_topics(6),
    },
    "waymo_open_dataset": {
        **_BASE_TOPICS,
        "/object_list/lidar_01": ObjectList,
        "/object_list/camera_01": ObjectList,
        "/object_list/camera_all": ObjectList,
        "/lidar_01/point_cloud": PointCloud2,
        **_camera_topics(5),
    },
}

DISCOVERY_TIMEOUT_S = 180.0
MESSAGE_TIMEOUT_S = 180.0
SHUTDOWN_TIMEOUT_S = 20.0


class PublishedTopicsTestBase(unittest.TestCase):
    """Shared logic for the per-dataset published-topics tests.

    Subclasses must set :attr:`DATASET` to a dataset name and :attr:`EXPECTED_TOPICS` to the
    ``{topic: message_class}`` mapping that dataset is expected to publish. ``__test__`` is
    ``False`` so pytest does not collect this base directly; a skip guard in ``setUp`` protects
    against other collectors.
    """

    # Tell pytest not to collect this abstract base directly.
    __test__ = False

    DATASET: str = ""
    EXPECTED_TOPICS: dict = {}

    def setUp(self):
        """Launch the dataset node headless and create the subscriber node."""
        if not self.DATASET:
            self.skipTest("PublishedTopicsTestBase is an abstract base class")
        self.launch_process = subprocess.Popen(
            [
                "ros2",
                "launch",
                "autonomy_datasets",
                "autonomy_datasets.launch.py",
                f"dataset:={self.DATASET}",
                "rviz:=no",
                "write_rosbag:=false",
                "overwrite_rosbag:=true",
                "publish_samples:=true",
                "wait_for_ack:=true",
                "target_frame_rate:=0.0",
                "loop:=false",
                "use_sim_time:=true",
            ],
            # own process group so the whole launch tree can be signalled on teardown
            start_new_session=True,
        )
        rclpy.init()
        self.node = rclpy.create_node("published_topics_test_subscriber")

    def tearDown(self):
        """Tear down the subscriber node and terminate the launched node."""
        if getattr(self, "node", None) is not None:
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self._terminate_launch()

    def _terminate_launch(self):
        """Send SIGINT to the launch process group, escalating to SIGKILL if needed."""
        proc = getattr(self, "launch_process", None)
        if proc is None or proc.poll() is not None:
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

    def test_requested_topics_are_published(self):
        """The advertised topics match the request and each one delivers a message."""
        # 1. Wait for the dataset node to advertise all expected publishers.
        self.assertTrue(
            self._spin_until(
                lambda: set(self._discover_advertised_topics()) >= set(self.EXPECTED_TOPICS),
                DISCOVERY_TIMEOUT_S,
            ),
            msg=(
                f"Dataset node did not advertise the expected topics for '{self.DATASET}' in time.\n"
                f"Expected: {sorted(self.EXPECTED_TOPICS)}\n"
                f"Advertised: {sorted(self._discover_advertised_topics())}"
            ),
        )

        advertised = self._discover_advertised_topics()

        # 2. The set of advertised topics must equal the requested set (no more, no less).
        self.assertEqual(
            set(advertised),
            set(self.EXPECTED_TOPICS),
            msg="Advertised topics do not match the requested topics",
        )

        # 3. The advertised message type must match the expected type for each topic.
        for topic, expected_type in self.EXPECTED_TOPICS.items():
            self.assertIn(advertised[topic], TYPE_MAP, f"Unexpected message type for '{topic}'")
            self.assertIs(
                TYPE_MAP[advertised[topic]],
                expected_type,
                f"Topic '{topic}' has type '{advertised[topic]}', expected {expected_type.__name__}",
            )

        # 4. Subscribe to every advertised topic. This also satisfies the node's
        #    wait_for_ack check so it starts publishing.
        received = set()
        subscriptions = []
        # Match the publisher QoS (RELIABLE / VOLATILE / KEEP_LAST depth 1).
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        for topic, type_str in advertised.items():
            subscriptions.append(
                self.node.create_subscription(
                    TYPE_MAP[type_str],
                    topic,
                    lambda _msg, t=topic: received.add(t),
                    qos,
                )
            )

        try:
            # 5. Wait until a message has been received on every requested topic.
            all_received = self._spin_until(
                lambda: received >= set(self.EXPECTED_TOPICS),
                MESSAGE_TIMEOUT_S,
            )
            missing = set(self.EXPECTED_TOPICS) - received
            self.assertTrue(
                all_received,
                msg=f"No message received on topics: {sorted(missing)}",
            )
        finally:
            for sub in subscriptions:
                self.node.destroy_subscription(sub)


class TestNvidiaPhysicalAiAvDataset(PublishedTopicsTestBase):
    """Published-topics test for the nvidia_physicalai_av_dataset dataset."""

    __test__ = True
    DATASET = "nvidia_physicalai_av_dataset"
    EXPECTED_TOPICS = EXPECTED_TOPICS_BY_DATASET["nvidia_physicalai_av_dataset"]


class TestNuscenes(PublishedTopicsTestBase):
    """Published-topics test for the nuscenes dataset."""

    __test__ = True
    DATASET = "nuscenes"
    EXPECTED_TOPICS = EXPECTED_TOPICS_BY_DATASET["nuscenes"]


class TestWaymoOpenDataset(PublishedTopicsTestBase):
    """Published-topics test for the waymo_open_dataset dataset."""

    __test__ = True
    DATASET = "waymo_open_dataset"
    EXPECTED_TOPICS = EXPECTED_TOPICS_BY_DATASET["waymo_open_dataset"]
