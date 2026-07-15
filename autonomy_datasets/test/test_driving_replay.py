# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

"""Tests for replaying existing DrivIng rosbags before native data."""

import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import autonomy_datasets.autonomy_datasets as node_module


class _FakeDrivIngAdapter:
    instances = []

    def __init__(self, **kwargs):
        self.start_scene_indices = kwargs["start_scene_indices"]
        self.generate_calls = 0
        self.__class__.instances.append(self)

    def generate_samples(self):
        self.generate_calls += 1
        return iter(())


class _FakeRosbagReplayAdapter:
    instances = []

    def __init__(self, rosbag_paths, data_publishers):
        self.rosbag_paths = rosbag_paths
        self.__class__.instances.append(self)

    def generate_samples(self):
        return iter(())


class TestDrivIngReplay(unittest.TestCase):
    """Verify that DrivIng continues with native data after bag replay."""

    def setUp(self):
        """Reset fake adapter instance tracking."""
        _FakeDrivIngAdapter.instances = []
        _FakeRosbagReplayAdapter.instances = []

    def test_replay_keeps_adapter_and_write_configuration(self):
        """Keep the download adapter alive without changing write_rosbag."""
        logger = SimpleNamespace(info=MagicMock(), warn=MagicMock(), fatal=MagicMock())
        node = SimpleNamespace(
            dataset="driving",
            dataset_path="/datasets",
            dataset_split="all",
            continue_from_latest=False,
            write_rosbag=True,
            overwrite_rosbag=False,
            loop=False,
            wait_for_ack=False,
            publish_samples=False,
            data_publishers={},
            driving_publish_ego_data=True,
            driving_publish_camera_images=True,
            driving_publish_lidar_pointclouds=True,
            driving_publish_lidar_object_lists=True,
            driving_auto_download=True,
            driving_download_workers=8,
            driving_rosbag_duration_seconds=20.0,
            get_logger=lambda: logger,
            _start_key_listener=MagicMock(),
            _stop_key_listener=MagicMock(),
            _close_rosbag_writer=MagicMock(),
        )

        with (
            patch.object(
                node_module,
                "rosbag_paths_by_sequence",
                return_value={"night": ["night_1", "night_2"], "day": ["day_1"], "dusk": []},
            ),
            patch.object(node_module, "completed_driving_rosbags", side_effect=lambda paths, duration: paths),
            patch.object(node_module, "DrivIngAdapter", _FakeDrivIngAdapter),
            patch.object(node_module, "RosbagReplayAdapter", _FakeRosbagReplayAdapter),
        ):
            node_module.AutonomyDatasets.publish_data(node)

        self.assertTrue(node.write_rosbag)
        self.assertEqual(len(_FakeDrivIngAdapter.instances), 1)
        self.assertEqual(_FakeDrivIngAdapter.instances[0].generate_calls, 1)
        self.assertEqual(
            _FakeDrivIngAdapter.instances[0].start_scene_indices,
            {"night": 2, "day": 1, "dusk": 0},
        )
        self.assertEqual(len(_FakeRosbagReplayAdapter.instances), 1)
        self.assertEqual(_FakeRosbagReplayAdapter.instances[0].rosbag_paths, ["night_1", "night_2", "day_1"])


if __name__ == "__main__":
    unittest.main()
