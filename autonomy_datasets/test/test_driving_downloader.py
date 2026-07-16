# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for the bounded parallel DrivIng archive reader."""

import io
import shutil
import tarfile
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import autonomy_datasets.datasets.driving.driving as driving
import numpy as np
from builtin_interfaces.msg import Time


class TestDrivIngDownloader(unittest.TestCase):
    """Verify bounded parallel download and sequence progress behavior."""

    def test_slow_first_chunk_prefetches_second_worker_batch(self):
        """Prefetch a second worker batch while an earlier chunk downloads."""
        workers = 4
        total = 20
        state_lock = threading.Lock()
        release_first = threading.Event()
        state = {"active": 0, "max_active": 0, "started": [], "ready_before_release": 0}
        files = [
            {
                "filename": f"DrivIng.tar.gz.{index:03d}",
                "id": index,
                "md5": "unused",
                "chunk_number": index + 1,
                "chunk_total": total,
            }
            for index in range(total)
        ]

        def fake_download(file_data, directory):
            index = file_data["id"]
            with state_lock:
                state["active"] += 1
                state["max_active"] = max(state["max_active"], state["active"])
                state["started"].append(index)
            if index == 0:
                release_first.wait(5)
            else:
                time.sleep(0.01)
            (directory / file_data["filename"]).write_bytes(bytes([index]) * 128)
            with state_lock:
                state["active"] -= 1

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)

            def release_slow_chunk():
                time.sleep(0.2)
                state["ready_before_release"] = len(list(root.glob("DrivIng.tar.gz.*")))
                release_first.set()

            release_thread = threading.Thread(target=release_slow_chunk)
            release_thread.start()
            with patch.object(driving, "_download_file", fake_download):
                with driving._DownloadingChunkReader(files, root, max_workers=workers) as reader:
                    actual = reader.read()
                    self.assertEqual(reader.backlog_chunks, workers * 2)
            release_thread.join()

            expected = b"".join(bytes([index]) * 128 for index in range(total))
            self.assertEqual(actual, expected)
            self.assertEqual(state["max_active"], workers)
            self.assertGreaterEqual(state["ready_before_release"], workers * 2 - 1)
            self.assertFalse(list(root.glob("DrivIng.tar.gz.*")))

    def test_split_progress_totals(self):
        """Report progress relative to the selected sequence."""
        for split, total in driving._SEQUENCE_LAST_CHUNK.items():
            with self.subTest(split=split):
                files = [{} for _ in range(total)]
                driving._annotate_chunk_positions(files)
                self.assertEqual(driving._chunk_position(files[0]), f"1/{total}")
                self.assertEqual(driving._chunk_position(files[-1]), f"{total}/{total}")

    def test_failed_download_is_reported_while_waiting_for_sequence(self):
        """Raise a download failure instead of waiting indefinitely."""

        class UnsetEvent:
            """Minimal event stub that never becomes ready."""

            @staticmethod
            def is_set():
                return False

            @staticmethod
            def wait(timeout=None):
                return False

        adapter = object.__new__(driving.DrivIngAdapter)
        adapter._sequence_ready = {"night": UnsetEvent()}
        adapter._download_error = OSError("network unavailable")

        with self.assertRaisesRegex(RuntimeError, "download or extraction failed"):
            adapter._wait_for_sequence("night")

    def test_ready_sequence_remains_available_after_later_download_failure(self):
        """Allow an extracted sequence to remain usable if a later sequence fails."""
        ready = threading.Event()
        ready.set()
        adapter = object.__new__(driving.DrivIngAdapter)
        adapter._sequence_ready = {"night": ready}
        adapter._download_error = OSError("later sequence failed")

        adapter._wait_for_sequence("night")

    def test_transient_download_failure_is_retried(self):
        """Retry a chunk after a transient network failure."""
        file_data = {"filename": "DrivIng.tar.gz.000", "chunk_number": 1, "chunk_total": 1}
        with tempfile.TemporaryDirectory() as temporary:
            with patch.object(
                driving,
                "_download_file_once",
                side_effect=[driving.URLError("temporary failure"), None],
            ) as download_once:
                with patch.object(driving.time, "sleep"):
                    driving._download_file(file_data, Path(temporary))
        self.assertEqual(download_once.call_count, 2)

    def test_invalid_options_are_rejected(self):
        """Reject unsupported splits and nonpositive worker counts."""
        with self.assertRaisesRegex(ValueError, "Unsupported DrivIng split"):
            driving.DrivIngAdapter({}, "/unused", "invalid", auto_download=False)
        with self.assertRaisesRegex(ValueError, "download_workers must be at least 1"):
            driving.DrivIngAdapter({}, "/unused", "night", download_workers=0, auto_download=False)
        with self.assertRaisesRegex(ValueError, "rosbag_duration_seconds must be greater than 0"):
            driving.DrivIngAdapter({}, "/unused", "night", rosbag_duration_seconds=0, auto_download=False)

    def test_rows_are_divided_into_fixed_duration_rosbag_scenes(self):
        """Create deterministic scene indices and omit empty time buckets."""
        rows = [
            {"timestamp_nanoseconds": str(timestamp)}
            for timestamp in (1_000_000_000, 20_999_999_999, 21_000_000_000, 61_000_000_000)
        ]
        self.assertEqual(driving._scene_indices(rows, 20.0), [0, 0, 1, 2])

    def test_resume_starts_at_next_rosbag_scene(self):
        """Skip completed fixed-duration scenes rather than a native sequence."""
        rows = [
            {"timestamp_nanoseconds": str(timestamp), "vehicle_state": f"{timestamp}.json"}
            for timestamp in (1_000_000_000, 11_000_000_000, 21_000_000_000, 41_000_000_000)
        ]
        adapter = object.__new__(driving.DrivIngAdapter)
        adapter.split = "night"
        adapter.publish_ego_data = False
        adapter.publish_camera_images = False
        adapter.publish_lidar_pointclouds = False
        adapter.publish_lidar_object_lists = False
        adapter.rosbag_duration_seconds = 20.0
        adapter.start_scene_indices = {"night": 1}

        with tempfile.TemporaryDirectory() as temporary:
            adapter.dataset_root_dir = Path(temporary)
            (adapter.dataset_root_dir / "night").mkdir()
            with (
                patch.object(adapter, "_wait_for_sequence"),
                patch.object(driving, "_read_timesync", return_value=rows),
                patch.object(driving, "_load_calibration", return_value={}),
                patch.object(driving, "_static_tf", return_value=[]),
                patch.object(driving, "_load_json_sensor", return_value={}),
                patch.object(driving, "_ego_messages", return_value=(None, None, {})),
            ):
                samples = list(adapter.generate_samples())

        self.assertEqual([sample["scene_id"] for _, sample in samples], ["night_00002", "night_00003"])

    def test_only_complete_rosbags_with_matching_duration_are_resumed(self):
        """Reject partial bags and bags created with another scene duration."""
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            complete = root / "driving_night_00001"
            partial = root / "driving_night_00002"
            incompatible = root / "driving_night_00003"
            for path in (complete, partial, incompatible):
                path.mkdir()
            driving.mark_rosbag_complete(str(complete), 20.0)
            driving.mark_rosbag_complete(str(incompatible), 30.0)

            actual = driving.completed_rosbags(
                [str(complete), str(partial), str(incompatible)],
                20.0,
            )

        self.assertEqual(actual, [str(complete)])

    def test_rosbags_are_discovered_by_native_sequence(self):
        """Share native-sequence bags between individual and all split selection."""
        with tempfile.TemporaryDirectory() as temporary:
            bag_root = Path(temporary) / "bags"
            bag_root.mkdir()
            for name in (
                "driving_night_00002",
                "driving_day_00001",
                "driving_night_00001",
                "driving_all_00001",
                "driving_day_00002_day_00002",
            ):
                (bag_root / name).mkdir()

            all_bags = driving.rosbag_paths_by_sequence(temporary, "all")
            day_bags = driving.rosbag_paths_by_sequence(temporary, "day")

        self.assertEqual(
            [Path(path).name for path in all_bags["night"]],
            ["driving_night_00001", "driving_night_00002"],
        )
        self.assertEqual([Path(path).name for path in day_bags["day"]], ["driving_day_00001"])
        self.assertEqual(driving.rosbag_identity("dusk_00042"), ("dusk", 42))

    def test_requested_sequence_is_released_before_later_sequences(self):
        """Stop after extracting the selected sequence from the shared stream."""
        archive_bytes = io.BytesIO()
        with tarfile.open(fileobj=archive_bytes, mode="w:gz") as archive:
            for name, content in (("night/sample.txt", b"night"), ("day/sample.txt", b"day")):
                member = tarfile.TarInfo(name)
                member.size = len(content)
                archive.addfile(member, io.BytesIO(content))
        payload = archive_bytes.getvalue()

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source_dir = root / "source"
            source_dir.mkdir()
            split_at = len(payload) // 2
            chunks = [payload[:split_at], payload[split_at:]]
            files = []
            for index, content in enumerate(chunks):
                filename = f"DrivIng.tar.gz.{index:03d}"
                source = source_dir / filename
                source.write_bytes(content)
                files.append({"filename": filename, "source": source})
            driving._annotate_chunk_positions(files)

            def copy_chunk(file_data, download_dir):
                shutil.copyfile(file_data["source"], download_dir / file_data["filename"])

            ready = []
            with patch.object(driving, "_dataverse_archive_files", return_value=files):
                with patch.object(driving, "_download_file", side_effect=copy_chunk):
                    driving._download_and_extract(
                        root / "dataset",
                        on_sequence_ready=ready.append,
                        download_workers=2,
                        requested_sequences={"night"},
                    )

            self.assertEqual(ready, ["night"])
            self.assertEqual((root / "dataset/night/sample.txt").read_bytes(), b"night")
            self.assertFalse((root / "dataset/day/sample.txt").exists())


class TestDrivIngReleasedSchema(unittest.TestCase):
    """Test the schema in the published native archive."""

    def test_vehicle_state_accepts_released_and_conversion_tool_keys(self):
        """Accept both published vehicle-state key layouts."""
        self.assertEqual(driving._state_value({"long_abs": 1.25}, "long_abs"), 1.25)
        self.assertEqual(driving._state_value({"ins_long_abs": 2.5}, "long_abs"), 2.5)

    def test_released_camera_directory_takes_precedence(self):
        """Prefer the camera directory names used by the published archive."""
        with tempfile.TemporaryDirectory() as temporary:
            sequence = Path(temporary)
            (sequence / "front_left_camera").mkdir()
            path = driving._sensor_path(sequence, "front_left_camera", "frame.jpg")
            self.assertEqual(path, sequence / "front_left_camera/frame.jpg")

    def test_static_transform_uses_serializable_zero_timestamp(self):
        """Use a valid zero timestamp for static transforms."""
        transform = driving._matrix_transform("base_link", "sensor", np.eye(4))
        self.assertIsNotNone(transform.header.stamp)
        self.assertEqual(transform.header.stamp.sec, 0)
        self.assertEqual(transform.header.stamp.nanosec, 0)

    def test_incomplete_synchronized_rows_are_skipped(self):
        """Discard rows missing an enabled sensor."""
        rows = [
            {"timestamp_nanoseconds": "1", "camera": "one.jpg"},
            {"timestamp_nanoseconds": "2", "camera": ""},
            {"timestamp_nanoseconds": "3", "camera": "three.jpg"},
        ]
        complete = driving._complete_sync_rows(rows, ["timestamp_nanoseconds", "camera"])
        self.assertEqual([row["timestamp_nanoseconds"] for row in complete], ["1", "3"])

    def test_native_relative_position_and_north_referenced_yaw_become_ros_enu(self):
        """Convert native north/east pose data to ROS ENU coordinates."""
        calibration = {"adma": np.eye(4), "dimensions": [4.0, 2.0, 1.5]}
        first = {
            "long_abs": 11.0,
            "lat_abs": 48.0,
            "height_msl": 400.0,
            "pos_rel_x": 100.0,
            "pos_rel_y": 200.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        }
        _, first_tf, origin = driving._ego_messages(first, calibration, None, Time())
        second = dict(first, pos_rel_x=101.0, pos_rel_y=202.0)
        _, second_tf, _ = driving._ego_messages(second, calibration, origin, Time())
        first_transform = first_tf.transforms[0].transform
        second_transform = second_tf.transforms[0].transform
        self.assertAlmostEqual(first_transform.translation.x, 0.0)
        self.assertAlmostEqual(first_transform.translation.y, 0.0)
        self.assertAlmostEqual(second_transform.translation.x, 2.0)
        self.assertAlmostEqual(second_transform.translation.y, 1.0)
        self.assertAlmostEqual(first_transform.rotation.z, np.sqrt(0.5))
        self.assertAlmostEqual(first_transform.rotation.w, np.sqrt(0.5))

    def test_lidar_timestamp_remains_float64(self):
        """Preserve native per-point timestamp precision."""
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "cloud.npz"
            timestamp = np.array([1750883585.800001, 1750883585.850002], dtype=np.float64)
            np.savez(
                path,
                x=np.array([1.0, 2.0], dtype=np.float32),
                y=np.array([3.0, 4.0], dtype=np.float32),
                z=np.array([5.0, 6.0], dtype=np.float32),
                intensity=np.array([7, 8], dtype=np.uint8),
                timestamp=timestamp,
            )
            cloud = driving._lidar_message(path, Time())
            dtype = np.dtype(
                [
                    ("x", "<f4"),
                    ("y", "<f4"),
                    ("z", "<f4"),
                    ("intensity", "<f4"),
                    ("timestamp", "<f8"),
                ]
            )
            decoded = np.frombuffer(cloud.data, dtype=dtype)
            self.assertEqual(cloud.point_step, 24)
            np.testing.assert_array_equal(decoded["timestamp"], timestamp)


if __name__ == "__main__":
    unittest.main()
