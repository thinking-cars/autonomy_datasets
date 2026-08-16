# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

"""Integration tests verifying the rosbag record/replay round-trip of the dataset node.

A first run (``write_rosbag:=true``, ``overwrite_rosbag:=true``) records the generated samples
to a rosbag; a second run replays them from that rosbag instead of regenerating from the raw
data. The tests confirm which path each run takes and that the replayed data matches what was
recorded.
"""

import glob
import os
import shutil
import tempfile
import time

import rosbag2_py
import yaml
from dataset_test_base import DatasetNodeTestBase
from perception_msgs.msg import EgoData, ObjectList
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

# Timeout for a recording run to generate all samples and close its rosbag.
RECORD_TIMEOUT_S = 300.0


class RosbagRoundtripTestBase(DatasetNodeTestBase):
    """Verify the record-then-replay rosbag round-trip.

    Subclasses set :attr:`DATASET`, :attr:`ROUNDTRIP_CONFIG` (a parameter file that keeps
    generation fast, i.e. no image/point-cloud decode) and :attr:`ROUNDTRIP_TOPICS` (the topics
    that configuration publishes).
    """

    ROUNDTRIP_CONFIG: str = ""
    ROUNDTRIP_TOPICS: dict = {}
    #: Whether the dataset stores a map next to each rosbag (only datasets that provide one).
    ROUNDTRIP_EXPECTS_MAP: bool = False

    def _write_temp_config(self):
        """Write ``ROUNDTRIP_CONFIG`` to a temp file and schedule its removal."""
        fd, path = tempfile.mkstemp(suffix=".yml", prefix=f"{self.DATASET}_roundtrip_")
        with os.fdopen(fd, "w") as config_file:
            config_file.write(self.ROUNDTRIP_CONFIG)
        self.addCleanup(os.remove, path)
        return path

    def _temp_log(self):
        """Return a temp file path for capturing a node's output, scheduled for removal."""
        fd, path = tempfile.mkstemp(suffix=".log", prefix=f"{self.DATASET}_roundtrip_")
        os.close(fd)
        self.addCleanup(os.remove, path)
        return path

    @staticmethod
    def _read(path):
        """Read a captured log file as text."""
        with open(path, "r", errors="replace") as log_file:
            return log_file.read()

    def _wait_for_log(self, proc, log_path, marker, timeout_s):
        """Wait until ``marker`` appears in the process log, the process ends, or timeout."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if marker in self._read(log_path):
                return True
            if proc.poll() is not None:  # process ended (e.g. crashed) without the marker
                break
            time.sleep(0.5)
        return marker in self._read(log_path)

    def _find_bags(self, bags_dir):
        """Return recorded rosbag directories for this dataset, sorted by name.

        The node stores rosbags in a subfolder per dataset adapter version, so bags of every
        version below ``bags_dir`` are reported.
        """
        return sorted(path for path in glob.glob(os.path.join(bags_dir, "*", f"{self.DATASET}_*")) if os.path.isdir(path))

    @staticmethod
    def _first_clock_in_bag(bag_path):
        """Return (sec, nanosec) of the first ``/clock`` message in a rosbag, or None."""
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap"),
            rosbag2_py.ConverterOptions(input_serialization_format="", output_serialization_format=""),
        )
        try:
            while reader.has_next():
                topic, data, _ = reader.read_next()
                if topic == "/clock":
                    clock = deserialize_message(data, Clock)
                    return clock.clock.sec, clock.clock.nanosec
        finally:
            reader.close()
        return None

    @staticmethod
    def _read_stored_map(bag_path):
        """Return the map contents stored next to a rosbag, or None if none is stored."""
        metadata_path = os.path.join(bag_path, "map.yaml")
        if not os.path.isfile(metadata_path):
            return None
        with open(metadata_path) as metadata_file:
            metadata = yaml.safe_load(metadata_file) or {}
        map_path = os.path.join(bag_path, metadata.get("map_contents_file", "map.osm"))
        if not os.path.isfile(map_path):
            return None
        with open(map_path) as map_file:
            return map_file.read()

    @staticmethod
    def _first_map_log_line(output):
        """Return the first map parameter update logged by a node run, or None."""
        marker = "Updated map parameters"
        for line in output.splitlines():
            if marker in line:
                return line[line.index(marker) :]
        return None

    def test_records_then_replays_rosbag(self):
        """First run records to a rosbag; second run replays it instead of regenerating."""
        config = self._write_temp_config()
        bags_dir = os.path.join(self.datasets_path, self.DATASET, "bags")
        # Bags mutate the (mounted) dataset; remove whatever this test produces afterwards.
        self.addCleanup(lambda: [shutil.rmtree(bag, ignore_errors=True) for bag in self._find_bags(bags_dir)])

        # --- Run 1: generate samples from raw data and record them to a rosbag. ---
        record_log = self._temp_log()
        record_proc = self._launch(
            log_path=record_log,
            config=config,
            write_rosbag="true",
            overwrite_rosbag="true",
            publish_samples="false",
            wait_for_ack="false",
        )
        # The node shuts itself down once generation is complete; wait for the completion log
        # (printed once the rosbag has been written and closed), after which the process exits.
        finished = self._wait_for_log(record_proc, record_log, "Finished publishing all samples", RECORD_TIMEOUT_S)
        self._terminate_launch(record_proc)
        self.assertTrue(finished, msg=f"Recording run did not finish in time:\n{self._read(record_log)}")

        record_output = self._read(record_log)
        self.assertNotIn(
            "replaying instead of generating",
            record_output,
            msg="First run must generate samples from raw data, not replay a rosbag",
        )

        bags = self._find_bags(bags_dir)
        self.assertTrue(bags, msg=f"First run recorded no rosbag in '{bags_dir}'")
        recorded_first_clock = self._first_clock_in_bag(bags[0])
        self.assertIsNotNone(recorded_first_clock, msg="Recorded rosbag contains no /clock messages")

        # The scene's map is stored next to the rosbag, as it is no topic but a set of parameters.
        recorded_map_params = None
        if self.ROUNDTRIP_EXPECTS_MAP:
            stored_map = self._read_stored_map(bags[0])
            self.assertTrue(stored_map, msg=f"First run stored no map next to the rosbag '{bags[0]}'")
            recorded_map_params = self._first_map_log_line(record_output)
            self.assertIsNotNone(recorded_map_params, msg="First run set no map parameters")
            self.assertIn(
                f"map_contents size={len(stored_map)}",
                recorded_map_params,
                msg="Map stored next to the rosbag differs from the map set as parameter",
            )

        # --- Run 2: replay the recorded rosbag instead of regenerating from raw data. ---
        replay_log = self._temp_log()
        self._launch(
            log_path=replay_log,
            config=config,
            write_rosbag="false",
            overwrite_rosbag="false",
            publish_samples="true",
            wait_for_ack="true",
        )
        first_messages = self._assert_topics_published(self.ROUNDTRIP_TOPICS, capture_first=("/clock",))

        replay_output = self._read(replay_log)
        self.assertIn(
            "replaying instead of generating",
            replay_output,
            msg="Second run must replay the recorded rosbag, not regenerate from raw data",
        )

        # The map parameters must be restored from the stored map, not from the raw dataset.
        if self.ROUNDTRIP_EXPECTS_MAP:
            self.assertEqual(
                self._first_map_log_line(replay_output),
                recorded_map_params,
                msg="Map parameters restored on replay do not match the ones set while recording",
            )

        # --- The replayed data must match what was recorded. ---
        replayed_clock = first_messages.get("/clock")
        self.assertIsNotNone(replayed_clock, msg="No /clock message received during replay")
        self.assertEqual(
            (replayed_clock.clock.sec, replayed_clock.clock.nanosec),
            recorded_first_clock,
            msg="First /clock replayed does not match the recorded rosbag",
        )


# nuscenes is fully local and fast to generate with image/point-cloud decoding disabled, which
# makes it the reference for the record/replay round-trip (the mechanism is dataset-agnostic).
NUSCENES_ROUNDTRIP_CONFIG = """\
/**/*:
  ros__parameters:
    dataset: nuscenes
    dataset_split: mini_val
    publish_ego_data: true
    publish_camera_images: false
    publish_lidar_pointclouds: false
    publish_radar_pointclouds: false
    publish_lidar_object_lists: true
    publish_camera_01_object_lists: true
    publish_lanelet2_map: true
"""


class TestNuscenesRosbagRoundtrip(RosbagRoundtripTestBase):
    """Record-then-replay round-trip test for the nuscenes dataset."""

    __test__ = True
    DATASET = "nuscenes"
    ROUNDTRIP_CONFIG = NUSCENES_ROUNDTRIP_CONFIG
    ROUNDTRIP_EXPECTS_MAP = True
    ROUNDTRIP_TOPICS = {
        "/clock": Clock,
        "/tf": TFMessage,
        "/tf_static": TFMessage,
        "/ego_data": EgoData,
        "/object_list/lidar_01": ObjectList,
        "/object_list/camera_01": ObjectList,
    }
