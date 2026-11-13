# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for detecting and skipping corrupted rosbags during replay.

A rosbag whose writing was interrupted lacks its metadata or ends in a truncated MCAP file, which
the MCAP storage plugin reads without raising an error until it silently ends early. Corrupted
rosbags are therefore detected up front and skipped, so that they do not break the replay of the
remaining ones.
"""

import os
import shutil
import tempfile
import unittest

from ament_index_python import get_package_share_directory
from autonomy_datasets.datasets.rosbag.rosbag import create_rosbag_writer, read_complete_rosbag_metadata, RosbagReplayAdapter
from rclpy.serialization import serialize_message
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2

NUM_SAMPLES = 50
TOPICS = {"/clock": "rosgraph_msgs/msg/Clock", "pointcloud": "sensor_msgs/msg/PointCloud2"}


class TestRosbagIntegrity(unittest.TestCase):
    """Corrupted rosbags are rejected and skipped during replay, complete ones are replayed."""

    def setUp(self):
        """Create a temporary directory for the rosbags of a test."""
        self.bag_root_dir = tempfile.mkdtemp(prefix="rosbag_integrity_")
        self.addCleanup(shutil.rmtree, self.bag_root_dir)

    def _write_rosbag(self, name: str) -> str:
        """Write a complete rosbag of NUM_SAMPLES samples and return its path."""
        bag_path = os.path.join(self.bag_root_dir, name)
        # compressed like the rosbags the node generates, which makes corrupted data detectable
        storage_config_uri = os.path.join(get_package_share_directory("autonomy_datasets"), "config", "mcap_storage_config.yaml")
        writer = create_rosbag_writer(bag_path, TOPICS, storage_config_uri)
        for index in range(NUM_SAMPLES):
            clock = Clock()
            clock.clock.sec = index
            pointcloud = PointCloud2()
            pointcloud.data = bytes(10_000)
            writer.write("/clock", serialize_message(clock), index * 1_000_000_000)
            writer.write("pointcloud", serialize_message(pointcloud), index * 1_000_000_000)
        writer.close()
        return bag_path

    def test_accepts_complete_rosbag(self):
        """A rosbag that has been closed is complete."""
        bag_path = self._write_rosbag("complete")
        self.assertEqual(read_complete_rosbag_metadata(bag_path).message_count, 2 * NUM_SAMPLES)

    def test_rejects_rosbag_without_metadata(self):
        """A rosbag without metadata, which is only written on closing it, is incomplete."""
        bag_path = self._write_rosbag("without_metadata")
        os.remove(os.path.join(bag_path, "metadata.yaml"))
        with self.assertRaises(ValueError):
            read_complete_rosbag_metadata(bag_path)

    def test_rejects_truncated_rosbag(self):
        """A truncated MCAP file, which can still be opened, is detected by its missing trailing magic bytes."""
        bag_path = self._write_rosbag("truncated")
        mcap_path = os.path.join(bag_path, "truncated_0.mcap")
        os.truncate(mcap_path, os.path.getsize(mcap_path) // 2)
        with self.assertRaisesRegex(ValueError, "truncated"):
            read_complete_rosbag_metadata(bag_path)

    def test_replay_skips_corrupted_rosbags(self):
        """Corrupted rosbags are skipped instead of breaking the replay of the remaining ones."""
        first_bag_path = self._write_rosbag("bag_1")
        truncated_bag_path = self._write_rosbag("bag_2")
        truncated_mcap_path = os.path.join(truncated_bag_path, "bag_2_0.mcap")
        os.truncate(truncated_mcap_path, os.path.getsize(truncated_mcap_path) // 2)
        corrupted_bag_path = self._write_rosbag("bag_3")
        corrupted_mcap_path = os.path.join(corrupted_bag_path, "bag_3_0.mcap")
        with open(corrupted_mcap_path, "r+b") as bag_file:
            bag_file.seek(os.path.getsize(corrupted_mcap_path) // 3)
            bag_file.write(b"\x5a" * 1_000)
        last_bag_path = self._write_rosbag("bag_4")

        adapter = RosbagReplayAdapter(
            [first_bag_path, truncated_bag_path, corrupted_bag_path, last_bag_path],
            {"/clock": None, "pointcloud": None},
            restore_map=False,
        )
        self.assertEqual(adapter.rosbag_paths, [first_bag_path, corrupted_bag_path, last_bag_path])

        samples_per_scene = {}
        for _, sample in adapter.generate_samples():
            samples_per_scene[sample["scene_id"]] = samples_per_scene.get(sample["scene_id"], 0) + 1
        # the rosbag corrupted within its data is replayed up to the corrupted data only
        self.assertLess(samples_per_scene.get("bag_3", 0), NUM_SAMPLES)
        # complete rosbags are replayed with all of their samples, including the last one
        self.assertEqual(samples_per_scene["bag_1"], NUM_SAMPLES)
        self.assertEqual(samples_per_scene["bag_4"], NUM_SAMPLES)


if __name__ == "__main__":
    unittest.main()
