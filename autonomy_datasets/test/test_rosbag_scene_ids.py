# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for the scene IDs stored in rosbags.

A rosbag stores the ID of the scene it was generated from in its metadata, so that replaying it
publishes the same scene ID as its generation did, instead of the name of the rosbag directory, and
so that continuing the rosbag generation skips the scenes that are already stored.
"""

import array
import os
import shutil
import tempfile
import unittest

import rosbag2_py
from autonomy_datasets.datasets.dataset import DatasetAdapter
from autonomy_datasets.datasets.rosbag.rosbag import (
    create_rosbag_writer,
    format_rosbag_counts,
    generation_parameter_warnings,
    get_stored_scene_ids,
    read_generation_parameters,
    RosbagReplayAdapter,
    SKIPPED_WITHOUT_SCENE_ID,
)
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

    def test_stores_generation_parameters(self):
        """The parameters the samples were generated with are stored in the rosbag's metadata and read back unchanged."""
        parameters = {
            "dataset": "zenseact_open_dataset",
            "exclude_topics": ["camera_*"],
            "zod_image_scale": 0.25,
            "nvidia_filter_countries": None,
            "ranges": array.array("d", [1.0, 2.5]),
        }
        bag_path = os.path.join(self.bag_root_dir, "zenseact_open_dataset_frames_mini_val_00001_000001")
        writer = create_rosbag_writer(bag_path, TOPICS, "", scene_id="000001", generation_parameters=parameters)
        writer.write("/clock", serialize_message(Clock()), 0)
        writer.close()

        adapter = RosbagReplayAdapter([bag_path], dict.fromkeys(TOPICS), restore_map=False)
        stored = read_generation_parameters(adapter.rosbag_infos[bag_path].custom_data)
        self.assertEqual(stored, {**parameters, "ranges": [1.0, 2.5]})

    def test_rosbags_without_generation_parameters(self):
        """Rosbags generated before the generation parameters were stored hold none."""
        bag_path = self._write_rosbag("nuscenes_mini_val_00001_scene-0103", scene_id="scene-0103")
        adapter = RosbagReplayAdapter([bag_path], dict.fromkeys(TOPICS), restore_map=False)
        self.assertIsNone(read_generation_parameters(adapter.rosbag_infos[bag_path].custom_data))


class TestGenerationParameterWarnings(unittest.TestCase):
    """Continuing the rosbag generation warns about stored rosbags that were generated with other parameters."""

    CURRENT = {"dataset": "zenseact_open_dataset", "exclude_topics": ["camera_*"], "zod_image_scale": 1.0}

    def setUp(self):
        """Create a temporary directory for the rosbags of a test."""
        self.bag_root_dir = tempfile.mkdtemp(prefix="rosbag_generation_parameters_")
        self.addCleanup(shutil.rmtree, self.bag_root_dir)

    def _write_rosbag(self, name: str, generation_parameters=None) -> str:
        """Write a rosbag of one sample with the given generation parameters, or none."""
        bag_path = os.path.join(self.bag_root_dir, name)
        writer = create_rosbag_writer(bag_path, TOPICS, "", scene_id=name, generation_parameters=generation_parameters)
        writer.write("/clock", serialize_message(Clock()), 0)
        writer.close()
        return bag_path

    def test_no_warning_for_rosbags_generated_alike(self):
        """Rosbags generated with the current parameters cause no warning, also if arrays are stored as lists."""
        bags = [
            self._write_rosbag("bag_1", self.CURRENT),
            self._write_rosbag("bag_2", {**self.CURRENT, "exclude_topics": ("camera_*",)}),
        ]
        self.assertEqual(generation_parameter_warnings(bags, self.CURRENT), [])

    def test_warns_about_differing_parameters(self):
        """Each differing parameter is reported with its current value and the values stored in how many rosbags."""
        bags = [
            self._write_rosbag("bag_1", self.CURRENT),
            self._write_rosbag("bag_2", {**self.CURRENT, "zod_image_scale": 0.25}),
            self._write_rosbag("bag_3", {**self.CURRENT, "zod_image_scale": 0.25, "exclude_topics": []}),
        ]
        warnings = generation_parameter_warnings(bags, self.CURRENT)
        self.assertEqual(len(warnings), 1)
        self.assertIn("2 of 3 stored rosbag(s) were generated with other parameters", warnings[0])
        self.assertIn("'exclude_topics' is [\"camera_*\"] now, but [] in 1", warnings[0])
        self.assertIn("'zod_image_scale' is 1.0 now, but 0.25 in 2", warnings[0])
        self.assertIn("overwrite_rosbag:=true", warnings[0])

    def test_warns_about_parameters_missing_in_stored_rosbags(self):
        """A parameter added since the rosbags were generated is reported as unset in them."""
        stored = {name: value for name, value in self.CURRENT.items() if name != "zod_image_scale"}
        warnings = generation_parameter_warnings([self._write_rosbag("bag_1", stored)], self.CURRENT)
        self.assertIn("'zod_image_scale' is 1.0 now, but <unset> in 1", warnings[0])

    def test_warns_about_rosbags_without_generation_parameters(self):
        """Rosbags generated before the generation parameters were stored are reported apart."""
        bags = [self._write_rosbag("bag_1", self.CURRENT), self._write_rosbag("bag_2")]
        warnings = generation_parameter_warnings(bags, self.CURRENT)
        self.assertEqual(len(warnings), 1)
        self.assertIn("1 of 2 stored rosbag(s) do not hold the parameters they were generated with", warnings[0])


class _SceneAdapter(DatasetAdapter):
    """Adapter yielding one sample of each of its scenes that is not stored yet."""

    def __init__(self, scene_ids, stored_scene_ids):
        super().__init__(data_publishers={}, stored_scene_ids=stored_scene_ids)
        self.scene_ids = scene_ids

    def generate_samples(self):
        for index, scene_id in enumerate(scene_id for scene_id in self.scene_ids if not self.is_scene_stored(scene_id)):
            yield index, {"scene_id": scene_id}


class TestStoredSceneIds(unittest.TestCase):
    """Continuing the rosbag generation skips the scenes whose rosbags are stored, identified by their scene ID."""

    def setUp(self):
        """Create a temporary directory for the rosbags of a test."""
        self.bag_root_dir = tempfile.mkdtemp(prefix="rosbag_stored_scene_ids_")
        self.addCleanup(shutil.rmtree, self.bag_root_dir)

    def _write_rosbag(self, name: str, scene_id=None) -> str:
        """Write a rosbag of one sample, storing the scene ID in its metadata unless it is None."""
        bag_path = os.path.join(self.bag_root_dir, name)
        if scene_id is not None:
            writer = create_rosbag_writer(bag_path, TOPICS, "", scene_id=scene_id)
        else:
            # rosbags generated by older versions of the node lack the scene ID in their metadata
            writer = rosbag2_py.SequentialWriter()
            writer.open(
                rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap"),
                rosbag2_py.ConverterOptions(input_serialization_format="", output_serialization_format=""),
            )
            writer.create_topic(rosbag2_py.TopicMetadata(id=0, name="/clock", type=TOPICS["/clock"], serialization_format="cdr"))
        writer.write("/clock", serialize_message(Clock()), 0)
        writer.close()
        return bag_path

    def test_reads_scene_ids_from_metadata(self):
        """The scene ID is read from a rosbag's metadata, even if its name holds another one."""
        bags = [
            self._write_rosbag("nvidia_physicalai_av_dataset_all_00001_clip-b", scene_id="clip-b"),
            self._write_rosbag("nvidia_physicalai_av_dataset_all_00002_renamed", scene_id="clip-d"),
        ]
        self.assertEqual(get_stored_scene_ids(bags, "nvidia_physicalai_av_dataset", "all"), {"clip-b", "clip-d"})

    def test_falls_back_to_scene_id_in_name(self):
        """The scene ID of a rosbag without it in its metadata is taken from its name, which may contain underscores."""
        bags = [self._write_rosbag("tum_traffic_r02_00003_r02_s01_00002")]
        self.assertEqual(get_stored_scene_ids(bags, "tum_traffic", "r02"), {"r02_s01_00002"})

    def test_ignores_rosbags_of_unknown_scenes(self):
        """A rosbag whose scene cannot be determined does not prevent the other scenes from being skipped."""
        bags = [self._write_rosbag("unrelated_name"), self._write_rosbag("nuscenes_mini_val_00001_x", scene_id="token-1")]
        self.assertEqual(get_stored_scene_ids(bags, "nuscenes", "mini_val"), {"token-1"})

    def test_adapter_skips_stored_scenes_regardless_of_their_position(self):
        """After a configuration change reorders or filters the scenes, only the scenes not stored yet are generated."""
        stored = get_stored_scene_ids(
            [
                self._write_rosbag("nvidia_physicalai_av_dataset_all_00001_clip-a", scene_id="clip-a"),
                self._write_rosbag("nvidia_physicalai_av_dataset_all_00002_clip-c", scene_id="clip-c"),
            ],
            "nvidia_physicalai_av_dataset",
            "all",
        )
        adapter = _SceneAdapter(["clip-b", "clip-c", "clip-d", "clip-a"], stored_scene_ids=stored)
        self.assertEqual([sample["scene_id"] for _, sample in adapter.generate_samples()], ["clip-b", "clip-d"])
        self.assertFalse(_SceneAdapter([], stored_scene_ids=None).is_scene_stored("clip-a"))

    def test_replay_counts_skipped_rosbags(self):
        """Rosbags without a stored scene ID are skipped, and counted apart from the replayed ones."""
        bags = [
            self._write_rosbag("nuscenes_mini_val_00001_token-1", scene_id="token-1"),
            self._write_rosbag("nuscenes_mini_val_00002_token-2"),
        ]
        adapter = RosbagReplayAdapter(bags, {"/clock": None}, restore_map=False)
        self.assertEqual(adapter.rosbag_paths, bags[:1])
        self.assertEqual(adapter.skipped_rosbags, {bags[1]: SKIPPED_WITHOUT_SCENE_ID})
        self.assertEqual(adapter._rosbag_counts(), "1 of 2 rosbag(s); 1 skipped: 1 without stored scene ID")


class TestRosbagCounts(unittest.TestCase):
    """The replay reports how many of the rosbags found are replayed, and why the others are skipped."""

    def test_without_skipped_rosbags(self):
        """Without skipped rosbags, only the number of replayed rosbags is reported."""
        self.assertEqual(format_rosbag_counts(10, 10, []), "10 rosbag(s)")

    def test_with_skipped_and_ended_early_rosbags(self):
        """Skipped rosbags are counted per reason, and rosbags ended early by corrupted data are counted apart."""
        self.assertEqual(
            format_rosbag_counts(7, 10, ["corrupted", "without stored scene ID", "corrupted"], ended_early_count=1),
            "7 of 10 rosbag(s); 3 skipped: 2 corrupted, 1 without stored scene ID; 1 ended early due to corrupted data",
        )


if __name__ == "__main__":
    unittest.main()
