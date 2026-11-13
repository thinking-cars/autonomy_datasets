# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for replaying rosbags with the scene IDs and topics they were generated with.

A rosbag stores the ID of the scene it was generated from in its metadata, so that replaying it
publishes the same scene ID as its generation did, instead of the name of the rosbag directory.
"""

import os
import shutil
import tempfile
import unittest

from autonomy_datasets.datasets.rosbag.rosbag import create_rosbag_writer, RosbagReplayAdapter
from rclpy.serialization import serialize_message
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2

NUM_SAMPLES = 3
TOPICS = {"/clock": "rosgraph_msgs/msg/Clock", "lidar_01/point_cloud": "sensor_msgs/msg/PointCloud2"}


class TestRosbagSceneIds(unittest.TestCase):
    """Replayed samples carry the scene ID and topics of the generated samples."""

    def setUp(self):
        """Create a temporary directory for the rosbags of a test."""
        self.bag_root_dir = tempfile.mkdtemp(prefix="rosbag_scene_ids_")
        self.addCleanup(shutil.rmtree, self.bag_root_dir)

    def _write_rosbag(self, name: str, scene_id: str) -> str:
        """Write a rosbag of NUM_SAMPLES samples of the given scene and return its path."""
        bag_path = os.path.join(self.bag_root_dir, name)
        writer = create_rosbag_writer(bag_path, TOPICS, "", scene_id=scene_id)
        for index in range(NUM_SAMPLES):
            clock = Clock()
            clock.clock.sec = index
            writer.write("/clock", serialize_message(clock), index * 1_000_000_000)
            writer.write("lidar_01/point_cloud", serialize_message(PointCloud2()), index * 1_000_000_000)
        writer.close()
        return bag_path

    @staticmethod
    def _replayed_scene_ids(adapter: RosbagReplayAdapter) -> list[str]:
        """Return the scene ID of each sample replayed by the adapter."""
        return [sample["scene_id"] for _, sample in adapter.generate_samples()]

    def test_replays_stored_scene_id(self):
        """The scene ID stored in the rosbag's metadata is replayed, even if it differs from the rosbag's name."""
        bag_path = self._write_rosbag("nuscenes_mini_val_00001_renamed", scene_id="scene-0103")
        adapter = RosbagReplayAdapter([bag_path], dict.fromkeys(TOPICS), restore_map=False)
        self.assertEqual(self._replayed_scene_ids(adapter), ["scene-0103"] * NUM_SAMPLES)

    def test_requests_stored_topics(self):
        """Without topics requested by a dataset adapter, all topics stored in the rosbags are requested."""
        bag_path = self._write_rosbag("nuscenes_mini_val_00001_scene-0103", scene_id="scene-0103")
        data_publishers = {"/clock": None, "/tf": None}
        adapter = RosbagReplayAdapter([bag_path], data_publishers, restore_map=False, request_stored_topics=True)
        self.assertEqual(set(data_publishers), {"/clock", "/tf", "lidar_01/point_cloud"})
        samples = [sample for _, sample in adapter.generate_samples()]
        self.assertEqual(len(samples), NUM_SAMPLES)
        self.assertTrue(all("lidar_01/point_cloud" in sample for sample in samples))


if __name__ == "__main__":
    unittest.main()
