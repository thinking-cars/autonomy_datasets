# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for selecting the published topics by patterns of topic names.

The parameters ``include_topics`` and ``exclude_topics`` select the topics of every dataset, both
those registered by a dataset adapter and those replayed from rosbags without one.
"""

import os
import shutil
import tempfile
import unittest

from autonomy_datasets.datasets.dataset import DatasetAdapter
from autonomy_datasets.datasets.rosbag.rosbag import create_rosbag_writer, RosbagReplayAdapter
from autonomy_datasets.datasets.topic_filter import TopicFilter
from rclpy.serialization import serialize_message
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, PointCloud2

TOPICS = [
    "/clock",
    "/tf",
    "/tf_static",
    "ego_data",
    "camera_01/image_raw",
    "camera_01/camera_info",
    "camera_02/image_raw",
    "camera_02/camera_info",
    "lidar_01/point_cloud",
    "lidar_02/point_cloud",
    "radar_01/point_cloud",
    "object_list/lidar_01",
    "object_list/lidar_01/meta_info",
    "object_list/camera_01",
    "object_list/camera_01/meta_info",
]


def selected(topic_filter: TopicFilter) -> set:
    """Return the topics of TOPICS the filter selects."""
    return {topic for topic in TOPICS if topic_filter(topic)}


class TestTopicFilter(unittest.TestCase):
    """Topics are selected by include and exclude patterns of their names."""

    def test_includes_all_topics_by_default(self):
        """Without patterns, all topics are selected."""
        self.assertEqual(selected(TopicFilter()), set(TOPICS))

    def test_wildcard_matches_slashes(self):
        """'*' matches any characters including '/', but patterns are anchored at the start of a topic."""
        self.assertEqual(
            selected(TopicFilter(include=["camera_*"])),
            {"camera_01/image_raw", "camera_01/camera_info", "camera_02/image_raw", "camera_02/camera_info"},
        )
        self.assertEqual(selected(TopicFilter(include=["*/image_raw"])), {"camera_01/image_raw", "camera_02/image_raw"})
        self.assertEqual(
            selected(TopicFilter(include=["lidar_0[2-9]", "radar_*"])), {"lidar_02/point_cloud", "radar_01/point_cloud"}
        )

    def test_pattern_matches_namespace(self):
        """A pattern also matches the topics below the namespace it names."""
        self.assertEqual(
            selected(TopicFilter(include=["object_list/lidar_01"])), {"object_list/lidar_01", "object_list/lidar_01/meta_info"}
        )
        self.assertEqual(selected(TopicFilter(include=["camera_01"])), {"camera_01/image_raw", "camera_01/camera_info"})
        self.assertFalse(TopicFilter(include=["camera_0"])("camera_01/image_raw"))

    def test_exclude_overrides_include(self):
        """A topic matching both an include and an exclude pattern is not selected."""
        topic_filter = TopicFilter(include=["*"], exclude=["camera_*", "object_list/*/meta_info", "lidar_02"])
        self.assertEqual(
            selected(topic_filter),
            set(TOPICS)
            - {
                "camera_01/image_raw",
                "camera_01/camera_info",
                "camera_02/image_raw",
                "camera_02/camera_info",
                "object_list/lidar_01/meta_info",
                "object_list/camera_01/meta_info",
                "lidar_02/point_cloud",
            },
        )

    def test_ignores_leading_slashes_and_empty_patterns(self):
        """Absolute and relative topic names are matched alike; empty patterns select nothing."""
        self.assertTrue(TopicFilter(include=["tf"])("/tf"))
        self.assertTrue(TopicFilter(include=["/ego_data"])("ego_data"))
        self.assertFalse(TopicFilter(include=["tf"])("/tf_static"))
        self.assertEqual(selected(TopicFilter(include=["*"], exclude=[""])), set(TOPICS))
        self.assertEqual(selected(TopicFilter(include=[""])), set())


class StubAdapter(DatasetAdapter):
    """Adapter providing a camera, a lidar and an object list."""

    def __init__(self, data_publishers, topic_filter=None):
        """Register the topics of the stub's sensors that the topic filter selects."""
        super().__init__(data_publishers, topic_filter=topic_filter)
        self.publish_camera_images = self.register_topics(["camera_01/image_raw", "camera_01/camera_info"])
        self.publish_lidar_pointclouds = self.register_topic("lidar_01/point_cloud")
        self.publish_lidar_object_lists = self.register_object_list("object_list/lidar_01")

    def generate_samples(self):
        """Generate no sample."""
        return iter(())


class TestAdapterTopicRegistration(unittest.TestCase):
    """Dataset adapters register only the topics selected by the topic filter."""

    def test_registers_all_topics_without_filter(self):
        """Without a topic filter, all topics of the adapter are registered."""
        data_publishers = {"/clock": None}
        adapter = StubAdapter(data_publishers)
        self.assertEqual(
            set(data_publishers),
            {
                "/clock",
                "camera_01/image_raw",
                "camera_01/camera_info",
                "lidar_01/point_cloud",
                "object_list/lidar_01",
                "object_list/lidar_01/meta_info",
            },
        )
        self.assertTrue(adapter.publish_camera_images and adapter.publish_lidar_pointclouds)

    def test_registers_selected_topics(self):
        """Only the selected topics are registered, and a group of topics is loaded if any of them is."""
        data_publishers = {"/clock": None}
        adapter = StubAdapter(data_publishers, TopicFilter(exclude=["lidar_*", "camera_01/camera_info", "*/meta_info"]))
        self.assertEqual(set(data_publishers), {"/clock", "camera_01/image_raw", "object_list/lidar_01"})
        self.assertTrue(adapter.publishes("camera_01/image_raw"))
        self.assertFalse(adapter.publishes("camera_01/camera_info"))
        # a group of topics is loaded if any of its topics is published
        self.assertTrue(adapter.publish_camera_images)
        self.assertFalse(adapter.publish_lidar_pointclouds)
        self.assertTrue(adapter.publish_lidar_object_lists)


class TestReplayTopicFilter(unittest.TestCase):
    """Without a dataset adapter, the stored topics selected by the topic filter are replayed."""

    def setUp(self):
        """Write a rosbag storing a camera, a lidar and a radar topic."""
        self.bag_root_dir = tempfile.mkdtemp(prefix="topic_filter_")
        self.addCleanup(shutil.rmtree, self.bag_root_dir)
        self.bag_path = os.path.join(self.bag_root_dir, "nuscenes_mini_val_00001_scene-0001")
        topics = {
            "/clock": "rosgraph_msgs/msg/Clock",
            "camera_01/image_raw": "sensor_msgs/msg/Image",
            "lidar_01/point_cloud": "sensor_msgs/msg/PointCloud2",
            "radar_01/point_cloud": "sensor_msgs/msg/PointCloud2",
        }
        writer = create_rosbag_writer(self.bag_path, topics, "", scene_id="scene-0001")
        for index in range(3):
            clock = Clock()
            clock.clock.sec = index
            timestamp = index * 1_000_000_000
            writer.write("/clock", serialize_message(clock), timestamp)
            writer.write("camera_01/image_raw", serialize_message(Image()), timestamp)
            writer.write("lidar_01/point_cloud", serialize_message(PointCloud2()), timestamp)
            writer.write("radar_01/point_cloud", serialize_message(PointCloud2()), timestamp)
        writer.close()

    def test_replays_selected_stored_topics(self):
        """Only the stored topics selected by the topic filter are requested and replayed."""
        data_publishers = {"/clock": None}
        adapter = RosbagReplayAdapter(
            [self.bag_path],
            data_publishers,
            restore_map=False,
            request_stored_topics=True,
            topic_filter=TopicFilter(exclude=["camera_*", "radar_*"]),
        )
        self.assertEqual(set(data_publishers), {"/clock", "lidar_01/point_cloud"})
        samples = [sample for _, sample in adapter.generate_samples()]
        self.assertEqual(len(samples), 3)
        self.assertTrue(all(set(sample) == {"scene_id", "/clock", "lidar_01/point_cloud"} for sample in samples))


if __name__ == "__main__":
    unittest.main()
