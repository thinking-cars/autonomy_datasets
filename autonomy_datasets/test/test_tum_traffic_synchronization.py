# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for the synchronization of the sensors of the TUM Traffic adapter.

The tests run offline: the adapter is set up with a recording of stand-in frames, whose sensors are
triggered at offsets beyond the synchronization tolerance, instead of reading the dataset.
"""

import unittest
from pathlib import Path

from autonomy_datasets.datasets.tum_traffic.tum_traffic import _Recording, _SensorFrames, TumTrafficAdapter

LIDAR_01 = "s110_lidar_ouster_south"
LIDAR_02 = "s110_lidar_ouster_north"
CAMERA_01 = "s110_camera_basler_south1_8mm"
CAMERA_02 = "s110_camera_basler_south2_8mm"

TOLERANCE_NS = 10_000_000
FRAME_PERIOD_NS = 100_000_000
FRAME_COUNT = 10


def _sensor(sensor_id, modality, offset_ns):
    """Return the frames of a sensor triggered with an offset to the other sensors."""
    timestamps = [index * FRAME_PERIOD_NS + offset_ns for index in range(FRAME_COUNT)]
    return _SensorFrames(sensor_id, modality, {timestamp: Path(f"{sensor_id}_{timestamp}") for timestamp in timestamps})


class TestReferenceSensor(unittest.TestCase):
    """Only a published sensor drives the playback of a recording, so that its frames are kept."""

    def setUp(self):
        """Create a recording whose sensors are triggered 30 ms apart, beyond the tolerance."""
        self.recording = _Recording(
            name="r02_s01",
            path=Path("r02_s01"),
            sensors={
                LIDAR_01: _sensor(LIDAR_01, "lidar", 0),
                LIDAR_02: _sensor(LIDAR_02, "lidar", 30_000_000),
                CAMERA_01: _sensor(CAMERA_01, "camera", 60_000_000),
                CAMERA_02: _sensor(CAMERA_02, "camera", 90_000_000),
            },
            labels={},
            calibration_files={},
        )

    def _adapter(self, published_sensors, publish_lidar_object_lists=False):
        """Return an adapter publishing the topics of some sensors of the recording."""
        adapter = TumTrafficAdapter.__new__(TumTrafficAdapter)
        adapter.lidar_topics = {LIDAR_01: "lidar_01", LIDAR_02: "lidar_02"}
        adapter.camera_topics = {CAMERA_01: "camera_01", CAMERA_02: "camera_02"}
        adapter.reference_lidar = LIDAR_01
        adapter.published_sensors = set(published_sensors)
        adapter.publish_lidar_pointclouds = any(sensor_id in adapter.lidar_topics for sensor_id in published_sensors)
        adapter.publish_camera_images = any(sensor_id in adapter.camera_topics for sensor_id in published_sensors)
        adapter.publish_lidar_object_lists = publish_lidar_object_lists
        adapter.sync_tolerance_ns = TOLERANCE_NS
        return adapter

    def _synchronized_timestamps(self, adapter):
        """Return the timestamps of the samples of the recording, and the sensors they hold."""
        reference_sensor = adapter._reference_sensor(self.recording)
        frames = adapter._synchronized_frames(self.recording, reference_sensor)
        return reference_sensor, [timestamp for timestamp, _ in frames], {tuple(sensor_frames) for _, sensor_frames in frames}

    def test_selected_lidar_drives_playback(self):
        """With only lidar_02 published, every frame of it forms a sample, instead of none matching lidar_01."""
        reference_sensor, timestamps, sensors = self._synchronized_timestamps(self._adapter({LIDAR_02}))
        self.assertEqual(reference_sensor, LIDAR_02)
        self.assertEqual(timestamps, self.recording.sensors[LIDAR_02].timestamps)
        self.assertEqual(sensors, {(LIDAR_02,)})

    def test_selected_camera_drives_playback(self):
        """With only camera_02 published, every frame of it forms a sample, instead of none matching a lidar."""
        reference_sensor, timestamps, _ = self._synchronized_timestamps(self._adapter({CAMERA_02}))
        self.assertEqual(reference_sensor, CAMERA_02)
        self.assertEqual(timestamps, self.recording.sensors[CAMERA_02].timestamps)

    def test_prefers_published_lidar_over_published_camera(self):
        """Of the published sensors, a lidar drives the playback rather than a camera."""
        reference_sensor, _, _ = self._synchronized_timestamps(self._adapter({CAMERA_01, LIDAR_02}))
        self.assertEqual(reference_sensor, LIDAR_02)

    def test_reference_lidar_drives_playback_of_its_object_list(self):
        """The object list of the reference lidar keeps it driving the playback, even without its point cloud."""
        adapter = self._adapter(set(), publish_lidar_object_lists=True)
        reference_sensor, timestamps, _ = self._synchronized_timestamps(adapter)
        self.assertEqual(reference_sensor, LIDAR_01)
        self.assertEqual(timestamps, self.recording.sensors[LIDAR_01].timestamps)

    def test_reference_lidar_preferred_for_object_list(self):
        """With the object list published, the reference lidar drives the playback rather than another published sensor."""
        adapter = self._adapter({CAMERA_02}, publish_lidar_object_lists=True)
        self.assertEqual(adapter._reference_sensor(self.recording), LIDAR_01)

    def test_no_published_sensor(self):
        """A recording holding none of the published sensors has no reference sensor and is skipped."""
        del self.recording.sensors[CAMERA_02]
        self.assertIsNone(self._adapter({CAMERA_02})._reference_sensor(self.recording))


if __name__ == "__main__":
    unittest.main()
