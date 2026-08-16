# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

"""Native-file adapter for the DrivIng multimodal driving dataset."""

import csv
import hashlib
import json
import os
import queue
import re
import tarfile
import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor
from io import RawIOBase
from pathlib import Path
from typing import Any, Dict, Iterator, List, Optional, Tuple
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import Request, urlopen

import cv2
import numpy as np
import perception_msgs_utils as pmu
from autonomy_datasets.datasets.dataset import DatasetAdapter
from autonomy_datasets.datasets.utils import timestamp_micros_to_clock
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from perception_msgs.msg import EGO, EgoData, HEXAMOTION, Object, ObjectClassification, ObjectList, ObjectReferencePoint
from rclpy.logging import get_logger
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

LOGGER = get_logger("autonomy_datasets.driving")

_CAMERAS = [
    "front_left_camera",
    "front_right_camera",
    "left_camera",
    "right_camera",
    "back_left_camera",
    "back_right_camera",
]
_CLASS_MAPPING = {
    "Car": ObjectClassification.CAR,
    "Van": ObjectClassification.CAR,
    "Bus": ObjectClassification.BUS,
    "Truck": ObjectClassification.UTILITY,
    "OtherVehicle": ObjectClassification.UTILITY,
    "Trailer": ObjectClassification.UTILITY,
    "Cyclist": ObjectClassification.BICYCLE,
    "E-Scooter": ObjectClassification.MICRO,
    "Motorcycle": ObjectClassification.MOTORCYCLE,
    "Pedestrian": ObjectClassification.PEDESTRIAN,
    "OtherPedestrian": ObjectClassification.VRU,
    "Animal": ObjectClassification.ANIMAL,
    "Other": ObjectClassification.UNKNOWN,
}
_DATAVERSE_URL = "https://dataverse.harvard.edu"
_PERSISTENT_ID = "doi:10.7910/DVN/VBZKDY"
_ARCHIVE_PREFIX = "DrivIng.tar.gz."
# The split gzip stream is ordered night -> day -> dusk. Boundary chunks contain
# data from both adjacent sequences.
_ARCHIVE_CHUNKS = 207
_SEQUENCE_LAST_CHUNK = {"night": 60, "day": 146, "dusk": _ARCHIVE_CHUNKS}
_SEQUENCES = tuple(_SEQUENCE_LAST_CHUNK)
_DOWNLOAD_ATTEMPTS = 5
_HTTP_TIMEOUT_SECONDS = 60
_ROSBAG_COMPLETE_MARKER = ".driving_complete"

# The native vehicle state holds no velocity, so it is differentiated from consecutive vehicle
# poses. Frames recorded at 10 Hz; a gap larger than this is not differentiated anymore [s]
_MAX_VELOCITY_TIME_GAP_SECONDS = 0.5

# Speed below which the ego vehicle is reported to be at standstill [m/s]
_STANDSTILL_VELOCITY = 0.1


class DrivIngAdapter(DatasetAdapter):
    """Converts native DrivIng files to normalized ROS 2 messages."""

    VERSION = "1.0.0"
    RELEASE_NOTES = {
        "0.1.0": "Initial integration into Autonomy.Datasets",
        "1.0.0": "Create version subfolders, fill EgoData velocity and standstill flag",
    }

    def __init__(
        self,
        data_publishers: Dict[str, Any],
        dataset_root_dir: str,
        split: str,
        publish_ego_data: bool = True,
        publish_camera_images: bool = True,
        publish_lidar_pointclouds: bool = True,
        publish_lidar_object_lists: bool = True,
        auto_download: bool = True,
        download_workers: int = 8,
        rosbag_duration_seconds: float = 20.0,
        start_scene_index: Optional[int] = None,
    ) -> None:
        """Initialize the adapter and start downloading missing data when enabled."""
        super().__init__(data_publishers=data_publishers)
        if split not in (*_SEQUENCES, "all"):
            raise ValueError(f"Unsupported DrivIng split '{split}'; expected one of: all, {', '.join(_SEQUENCES)}")
        if download_workers < 1:
            raise ValueError("DrivIng download_workers must be at least 1")
        if rosbag_duration_seconds <= 0:
            raise ValueError("DrivIng rosbag_duration_seconds must be greater than 0")
        self.split = split
        self.rosbag_duration_seconds = rosbag_duration_seconds
        requested_root = Path(dataset_root_dir)
        requested_sequences = list(_SEQUENCES) if split == "all" else [split]
        self.dataset_root_dir = requested_root
        self._sequence_ready = {sequence: threading.Event() for sequence in _SEQUENCES}
        self._ready_sequences = queue.Queue()
        self._download_error = None
        self._download_thread = None
        download_in_progress = (self.dataset_root_dir / ".driving_download").is_dir()
        for sequence, event in self._sequence_ready.items():
            marker_exists = (self.dataset_root_dir / f".{sequence}_complete").is_file()
            manually_installed = (self.dataset_root_dir / sequence).is_dir() and not download_in_progress
            if marker_exists or manually_installed:
                event.set()
        data_available = all(self._sequence_ready[sequence].is_set() for sequence in requested_sequences)
        if not data_available and auto_download:
            self._download_thread = threading.Thread(
                target=self._download_in_background,
                args=(self.dataset_root_dir, download_workers),
                name="driving-download",
                daemon=True,
            )
            self._download_thread.start()
        elif not data_available:
            raise FileNotFoundError(f"DrivIng data directory not found: {self.dataset_root_dir}")
        self.publish_ego_data = publish_ego_data
        self.publish_camera_images = publish_camera_images
        self.publish_lidar_pointclouds = publish_lidar_pointclouds
        self.publish_lidar_object_lists = publish_lidar_object_lists
        self.start_scene_index = start_scene_index
        if publish_ego_data:
            data_publishers["ego_data"] = None
        if publish_lidar_pointclouds:
            data_publishers["lidar_01/point_cloud"] = None
        if publish_lidar_object_lists:
            data_publishers["object_list/lidar_01"] = None
        if publish_camera_images:
            for index in range(1, len(_CAMERAS) + 1):
                data_publishers[f"camera_{index:02d}/image_raw"] = None
                data_publishers[f"camera_{index:02d}/camera_info"] = None

    def generate_samples(self) -> Iterator[Tuple[int, Dict[str, Any]]]:
        """Yield synchronized native DrivIng frames for each selected sequence."""
        sequences = [self.split] if self.split != "all" else self._sequences_as_available()
        example_index = 0
        scene_index = 0
        for sequence in sequences:
            self._wait_for_sequence(sequence)
            sequence_dir = self.dataset_root_dir / sequence
            if not sequence_dir.is_dir():
                raise FileNotFoundError(f"DrivIng sequence directory not found: {sequence_dir}")
            sync_rows = _read_timesync(sequence_dir / "timesync_info.csv")
            required_sensors = ["timestamp_nanoseconds", "vehicle_state"]
            if self.publish_lidar_pointclouds:
                required_sensors.append("middle_lidar")
            if self.publish_camera_images:
                required_sensors.extend(_CAMERAS)
            complete_rows = _complete_sync_rows(sync_rows, required_sensors)
            scene_indices = _scene_indices(complete_rows, self.rosbag_duration_seconds)
            skipped_rows = len(sync_rows) - len(complete_rows)
            if skipped_rows:
                LOGGER.info(
                    f"DrivIng sequence '{sequence}': {skipped_rows} frames are missing data "
                    f"from one or more enabled sensors and will be skipped; "
                    f"{len(complete_rows)} frames are available."
                )
            calibration = _load_calibration(sequence_dir / "calibration.json")
            labels = _load_labels(sequence_dir / "annotations.json") if self.publish_lidar_object_lists else {}
            static_tf = _static_tf(calibration)
            origin = None
            last_scene_id = None
            previous_pose = None
            previous_timestamp = None
            for row, sequence_scene_index in zip(complete_rows, scene_indices):
                scene_id = f"{sequence}-{sequence_scene_index + 1:05d}"
                if scene_id != last_scene_id:
                    last_scene_id = scene_id
                    scene_index += 1
                if scene_index <= self.start_scene_index:
                    # LOGGER.info(f"Skipping already stored scene {scene_index}: {scene_id}")
                    continue
                timestamp = int(row["timestamp_nanoseconds"])
                stamp = timestamp_micros_to_clock(timestamp // 1000).clock
                state = _load_json_sensor(sequence_dir, "vehicle_state", row.get("vehicle_state"))
                global_from_vehicle, origin = _vehicle_pose(state, calibration, origin)
                # The native vehicle state holds no velocity; derive it from consecutive poses
                velocity = None
                if previous_pose is not None:
                    time_gap = (timestamp - previous_timestamp) / 1e9
                    if 0 < time_gap <= _MAX_VELOCITY_TIME_GAP_SECONDS:
                        velocity = (global_from_vehicle[:3, 3] - previous_pose[:3, 3]) / time_gap
                previous_pose = global_from_vehicle
                previous_timestamp = timestamp
                ego_data, dynamic_tf = _ego_messages(global_from_vehicle, calibration, stamp, velocity)
                sample: Dict[str, Any] = {
                    "scene_id": scene_id,
                    "/clock": timestamp_micros_to_clock(timestamp // 1000),
                    "/tf_static": TFMessage(transforms=static_tf),
                }
                sample["/tf"] = dynamic_tf
                if self.publish_ego_data:
                    sample["ego_data"] = ego_data
                if self.publish_lidar_pointclouds:
                    lidar_path = _sensor_path(sequence_dir, "middle_lidar", row.get("middle_lidar"))
                    sample["lidar_01/point_cloud"] = _lidar_message(lidar_path, stamp)
                if self.publish_lidar_object_lists:
                    sample["object_list/lidar_01"] = _object_list(labels.get(str(timestamp), []), stamp, scene_id)
                if self.publish_camera_images:
                    for index, camera in enumerate(_CAMERAS, 1):
                        image_path = _sensor_path(sequence_dir, camera, row.get(camera))
                        frame_id = f"camera_{index:02d}"
                        sample[f"camera_{index:02d}/image_raw"] = _image_message(image_path, stamp, frame_id)
                        sample[f"camera_{index:02d}/camera_info"] = _camera_info(calibration["cameras"][camera], stamp, frame_id)
                example_index += 1
                yield example_index, sample

    def _download_in_background(self, dataset_root: Path, download_workers: int) -> None:
        """Download data without delaying rosbag replay or RViz startup."""
        try:
            _download_and_extract(
                dataset_root,
                on_sequence_ready=self._mark_sequence_ready,
                download_workers=download_workers,
                requested_sequences=None if self.split == "all" else {self.split},
            )
        except Exception as error:
            self._download_error = error
            LOGGER.error(f"DrivIng download or extraction failed: {error}")

    def _mark_sequence_ready(self, sequence: str) -> None:
        """Record a completely extracted sequence and wake any waiter."""
        if sequence in self._sequence_ready:
            (self.dataset_root_dir / f".{sequence}_complete").touch()
            LOGGER.info(f"DrivIng sequence '{sequence}' is ready.")
            self._sequence_ready[sequence].set()
            self._ready_sequences.put(sequence)

    def _wait_for_sequence(self, sequence: str) -> None:
        """Wait only for the requested sequence, not for the complete dataset archive."""
        event = self._sequence_ready[sequence]
        if event.is_set():
            return
        LOGGER.info(f"Waiting for DrivIng sequence '{sequence}' to become available.")
        while not event.wait(timeout=1.0):
            if self._download_error is not None:
                raise RuntimeError("DrivIng download or extraction failed") from self._download_error

    def _sequences_as_available(self) -> Iterator[str]:
        """Yield complete sequences in extraction order when the ``all`` split is selected."""
        pending = set(self._sequence_ready)
        while pending:
            try:
                ready = self._ready_sequences.get_nowait()
            except queue.Empty:
                ready = next((sequence for sequence in pending if self._sequence_ready[sequence].is_set()), None)
                if ready is None:
                    try:
                        ready = self._ready_sequences.get(timeout=1.0)
                    except queue.Empty:
                        if self._download_error is not None:
                            raise RuntimeError("DrivIng download or extraction failed") from self._download_error
                        continue
            if ready in pending:
                pending.remove(ready)
                yield ready


def _read_timesync(path: Path) -> List[Dict[str, str]]:
    with path.open(newline="") as file:
        rows = list(csv.reader(file))
    sensor_rows = rows[1:]
    frame_count = len(rows[0]) - 1
    keys = [row[0] for row in sensor_rows]
    return [dict(zip(keys, (row[index] for row in sensor_rows))) for index in range(1, frame_count + 1)]


def _complete_sync_rows(rows: List[Dict[str, str]], required_sensors: List[str]) -> List[Dict[str, str]]:
    """Keep only frames containing every sensor enabled for publication."""
    return [row for row in rows if all(row.get(sensor) for sensor in required_sensors)]


def _scene_indices(rows: List[Dict[str, str]], duration_seconds: float) -> List[int]:
    """Assign synchronized rows to consecutive fixed-duration ROS scenes."""
    if not rows:
        return []
    duration_nanoseconds = round(duration_seconds * 1_000_000_000)
    first_timestamp = int(rows[0]["timestamp_nanoseconds"])
    bucket_to_scene = {}
    indices = []
    for row in rows:
        bucket = (int(row["timestamp_nanoseconds"]) - first_timestamp) // duration_nanoseconds
        if bucket not in bucket_to_scene:
            bucket_to_scene[bucket] = len(bucket_to_scene)
        indices.append(bucket_to_scene[bucket])
    return indices


def sequences_for_split(split: str) -> Tuple[str, ...]:
    """Return native sequences represented by a DrivIng split."""
    return _SEQUENCES if split == "all" else (split,)


def rosbag_paths_by_sequence(dataset_path: str, split: str) -> Dict[str, List[str]]:
    """Find canonical DrivIng rosbag paths grouped by native sequence."""
    bag_root = Path(dataset_path) / "bags"
    result = {sequence: [] for sequence in sequences_for_split(split)}
    if not bag_root.is_dir():
        return result
    for sequence in result:
        pattern = re.compile(rf"^driving_{sequence}_(\d+)$")
        indexed_paths = []
        for path in bag_root.iterdir():
            match = pattern.fullmatch(path.name)
            if path.is_dir() and match:
                indexed_paths.append((int(match.group(1)), str(path)))
        result[sequence] = [path for _, path in sorted(indexed_paths)]
    return result


def completed_rosbags(rosbag_paths: List[str], duration_seconds: float) -> List[str]:
    """Return the contiguous prefix completed with the configured duration."""
    completed = []
    for expected_index, rosbag_path in enumerate(rosbag_paths, start=1):
        try:
            actual_index = int(Path(rosbag_path).name.rsplit("_", 1)[1])
        except (IndexError, ValueError):
            break
        if actual_index != expected_index:
            break
        marker = Path(rosbag_path) / _ROSBAG_COMPLETE_MARKER
        try:
            marker_duration = float(marker.read_text().strip())
        except (OSError, ValueError):
            break
        if marker_duration != duration_seconds:
            break
        completed.append(rosbag_path)
    return completed


def mark_rosbag_complete(rosbag_path: str, duration_seconds: float) -> None:
    """Mark a fully written DrivIng rosbag segment as resumable."""
    (Path(rosbag_path) / _ROSBAG_COMPLETE_MARKER).write_text(f"{duration_seconds}\n")


def rosbag_identity(scene_id: str) -> Tuple[str, int]:
    """Return the native sequence and local segment index from a DrivIng scene ID."""
    match = re.fullmatch(r"(night|day|dusk)_(\d+)", scene_id)
    if match is None:
        raise ValueError(f"Invalid DrivIng scene ID: {scene_id}")
    return match.group(1), int(match.group(2))


def _download_and_extract(
    dataset_root: Path, on_sequence_ready=None, download_workers: int = 8, requested_sequences=None
) -> None:
    """Download chunks on demand and extract each native sequence as soon as it is complete."""
    data_dir = dataset_root
    data_dir.mkdir(parents=True, exist_ok=True)
    download_dir = data_dir / ".driving_download"
    download_dir.mkdir(exist_ok=True)
    LOGGER.info(f"DrivIng data was not found in '{data_dir}'; downloading it from Harvard Dataverse.")
    files = _dataverse_archive_files()
    if requested_sequences and len(requested_sequences) == 1 and len(files) == _ARCHIVE_CHUNKS:
        requested_sequence = next(iter(requested_sequences))
        required_chunks = _SEQUENCE_LAST_CHUNK[requested_sequence]
        files = files[:required_chunks]
        LOGGER.info(f"DrivIng sequence '{requested_sequence}' requires archive chunks 1-{required_chunks}/{_ARCHIVE_CHUNKS}.")
        _annotate_chunk_positions(files)
    LOGGER.info("Downloading and extracting DrivIng data concurrently.")
    current_sequence = None
    stopped_after_requested_sequence = False
    with _DownloadingChunkReader(files, download_dir, max_workers=download_workers) as chunk_reader:
        with tarfile.open(fileobj=chunk_reader, mode="r|gz") as archive:
            for member in archive:
                target = (data_dir / member.name).resolve()
                if target != data_dir.resolve() and data_dir.resolve() not in target.parents:
                    raise ValueError(f"Unsafe path in DrivIng archive: {member.name}")
                sequence = member.name.split("/", 1)[0]
                if sequence in {"day", "dusk", "night"}:
                    if current_sequence is not None and sequence != current_sequence:
                        LOGGER.info(
                            f"Finished extracting DrivIng sequence '{current_sequence}' at archive "
                            f"chunk {chunk_reader.current_chunk_position}."
                        )
                        if on_sequence_ready is not None:
                            on_sequence_ready(current_sequence)
                        if requested_sequences == {current_sequence}:
                            LOGGER.info(f"Download complete for selected DrivIng sequence '{current_sequence}'.")
                            stopped_after_requested_sequence = True
                            break
                    current_sequence = sequence
                archive.extract(member, data_dir, filter="data")
    if current_sequence is not None and not stopped_after_requested_sequence and on_sequence_ready is not None:
        on_sequence_ready(current_sequence)
    for file_data in files:
        chunk_path = download_dir / file_data["filename"]
        if chunk_path.exists():
            chunk_path.unlink()
    if not any(download_dir.iterdir()):
        download_dir.rmdir()
    expected_sequences = requested_sequences or {"day", "dusk", "night"}
    missing_sequences = [sequence for sequence in expected_sequences if not (data_dir / sequence).is_dir()]
    if missing_sequences:
        raise RuntimeError(f"DrivIng archive extraction did not create expected sequences: {missing_sequences}")


def _raise_for_bot_challenge(response: Any, request_description: str) -> None:
    """Reject a bot challenge that Harvard Dataverse returns in place of the requested data.

    Args:
        response: Response returned by :func:`urlopen`.
        request_description: What was requested, used in the error message.

    Raises:
        RuntimeError: If the response is a bot challenge instead of the requested data.
    """
    waf_action = response.headers.get("x-amzn-waf-action")
    if waf_action is None:
        return
    raise RuntimeError(
        f"Harvard Dataverse answered the request for the DrivIng {request_description} with a bot "
        f"challenge instead of data (HTTP {response.status}, x-amzn-waf-action: {waf_action}), "
        "which this download cannot solve. Retry later, or download the archive chunks manually "
        f"from {_DATAVERSE_URL}/dataset.xhtml?persistentId={_PERSISTENT_ID} and extract the "
        "sequences into the DrivIng dataset directory."
    )


def _dataverse_archive_files() -> List[Dict[str, Any]]:
    url = f"{_DATAVERSE_URL}/api/datasets/:persistentId/?persistentId={quote(_PERSISTENT_ID)}"
    with urlopen(
        Request(url, headers={"User-Agent": "autonomy_datasets"}),
        timeout=_HTTP_TIMEOUT_SECONDS,
    ) as response:
        _raise_for_bot_challenge(response, "dataset metadata")
        status = response.status
        payload = response.read()
    try:
        metadata = json.loads(payload)
    except json.JSONDecodeError as error:
        raise RuntimeError(
            f"Harvard Dataverse returned no valid DrivIng dataset metadata (HTTP {status}, " f"{len(payload)} bytes): {error}"
        ) from error
    files = []
    for entry in metadata["data"]["latestVersion"]["files"]:
        data_file = entry["dataFile"]
        if data_file["filename"].startswith(_ARCHIVE_PREFIX):
            files.append(
                {
                    "filename": data_file["filename"],
                    "id": data_file["id"],
                    "md5": data_file["md5"],
                    "filesize": data_file.get("filesize"),
                }
            )
    if not files:
        raise RuntimeError("No DrivIng archive chunks found in Harvard Dataverse metadata")
    files = sorted(files, key=lambda file_data: file_data["filename"])
    _annotate_chunk_positions(files)
    return files


def _annotate_chunk_positions(files: List[Dict[str, Any]]) -> None:
    """Number chunks relative to the active download rather than the full archive."""
    for index, file_data in enumerate(files, start=1):
        file_data["chunk_number"] = index
        file_data["chunk_total"] = len(files)


def _download_file(file_data: Dict[str, Any], download_dir: Path) -> None:
    for attempt in range(1, _DOWNLOAD_ATTEMPTS + 1):
        try:
            _download_file_once(file_data, download_dir)
            return
        except (HTTPError, URLError, TimeoutError, OSError, RuntimeError) as error:
            if attempt == _DOWNLOAD_ATTEMPTS:
                raise
            delay = min(2 ** (attempt - 1), 30)
            LOGGER.warn(
                f"DrivIng archive chunk {_chunk_position(file_data)} failed: {error}. "
                f"Retrying in {delay} seconds ({attempt}/{_DOWNLOAD_ATTEMPTS})."
            )
            time.sleep(delay)


def _download_file_once(file_data: Dict[str, Any], download_dir: Path) -> None:
    destination = download_dir / file_data["filename"]
    chunk_position = _chunk_position(file_data)
    if destination.is_file() and _md5(destination) == file_data["md5"]:
        LOGGER.info(f"Using existing DrivIng archive chunk {chunk_position}: {destination.name}")
        return
    partial = destination.with_suffix(destination.suffix + ".part")
    expected_size = file_data.get("filesize")
    if partial.exists() and expected_size and partial.stat().st_size >= expected_size:
        if partial.stat().st_size == expected_size and _md5(partial) == file_data["md5"]:
            os.replace(partial, destination)
            LOGGER.info(f"Using completed DrivIng archive chunk {chunk_position}: {destination.name}")
            return
        partial.unlink()
    offset = partial.stat().st_size if partial.exists() else 0
    request = Request(
        f"{_DATAVERSE_URL}/api/access/datafile/{file_data['id']}",
        headers={"User-Agent": "autonomy_datasets"},
    )
    if offset:
        request.add_header("Range", f"bytes={offset}-")
    LOGGER.info(f"Downloading DrivIng archive chunk {chunk_position}: {destination.name}")
    with urlopen(request, timeout=_HTTP_TIMEOUT_SECONDS) as response:
        _raise_for_bot_challenge(response, f"archive chunk {chunk_position}")
        mode = "ab" if offset and response.status == 206 else "wb"
        remaining = int(response.headers.get("Content-Length", 0))
        total = (offset if mode == "ab" else 0) + remaining
        downloaded = offset if mode == "ab" else 0
        report_step = max(total // 20, 64 * 1024 * 1024) if total else 512 * 1024 * 1024
        next_report = downloaded + report_step
        with partial.open(mode) as output:
            while chunk := response.read(1024 * 1024):
                output.write(chunk)
                downloaded += len(chunk)
                if downloaded >= next_report:
                    _log_progress(f"  Chunk {chunk_position} ({destination.name})", downloaded, total)
                    next_report += report_step
        _log_progress(f"  Chunk {chunk_position} ({destination.name})", downloaded, total)
    if _md5(partial) != file_data["md5"]:
        partial.unlink(missing_ok=True)
        raise RuntimeError(f"Checksum mismatch for downloaded DrivIng archive chunk: {destination.name}")
    os.replace(partial, destination)


def _chunk_position(file_data: Dict[str, Any]) -> str:
    """Return a human-readable one-based chunk position from Dataverse metadata."""
    if "chunk_number" in file_data and "chunk_total" in file_data:
        return f"{file_data['chunk_number']}/{file_data['chunk_total']}"
    return "?/?"


def _md5(path: Path) -> str:
    digest = hashlib.md5()
    with path.open("rb") as file:
        while chunk := file.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def _log_progress(label: str, completed: int, total: int) -> None:
    """Log a stable, single-line byte progress update."""
    completed_gib = completed / 1024**3
    if total:
        LOGGER.info(f"{label}: {completed_gib:.2f} / {total / 1024**3:.2f} GiB ({completed / total:.0%})")
    else:
        LOGGER.info(f"{label}: {completed_gib:.2f} GiB")


class _DownloadingChunkReader(RawIOBase):
    """Read chunks in order using a continuously fed, disk-bounded download pool."""

    def __init__(self, files: List[Dict[str, Any]], download_dir: Path, max_workers: int = 8) -> None:
        self.files = files
        self.download_dir = download_dir
        self.max_workers = min(max_workers, len(files))
        # Queue a second worker batch to reduce idle time behind a slow early chunk,
        # while bounding downloaded/in-progress data to twice the parallelism.
        self.backlog_chunks = min(self.max_workers * 2, len(files))
        self.executor = ThreadPoolExecutor(max_workers=self.max_workers, thread_name_prefix="driving-chunk")
        self.futures: Dict[int, Future] = {}
        self.index = 0
        self.next_to_schedule = 0
        self.current = None
        self.current_chunk_position = "?/?"
        for _ in range(self.backlog_chunks):
            self._schedule_next()
        self._open_next()

    def readable(self) -> bool:
        return True

    def close(self) -> None:
        if self.current is not None:
            self.current.close()
            self.current_path.unlink(missing_ok=True)
            self.current = None
        self.executor.shutdown(wait=False, cancel_futures=True)
        super().close()

    def readinto(self, buffer) -> int:
        while self.current is not None:
            size = self.current.readinto(buffer)
            if size:
                return size
            self.current.close()
            self.current_path.unlink(missing_ok=True)
            self._schedule_next()
            self._open_next()
        return 0

    def _open_next(self) -> None:
        if self.index >= len(self.files):
            self.current = None
            self.executor.shutdown(wait=True)
            return
        file_data = self.files[self.index]
        self.futures.pop(self.index).result()
        self.current_path = self.download_dir / file_data["filename"]
        self.current = self.current_path.open("rb")
        self.current_chunk_position = _chunk_position(file_data)
        self.index += 1

    def _schedule_next(self) -> None:
        if self.next_to_schedule < len(self.files):
            index = self.next_to_schedule
            self.futures[index] = self.executor.submit(_download_file, self.files[index], self.download_dir)
            self.next_to_schedule += 1


def _load_calibration(path: Path) -> Dict[str, Any]:
    with path.open() as file:
        raw = json.load(file)
    cameras = {}
    for name in _CAMERAS:
        value = raw[name]
        intrinsics, extrinsics = value["intrinsics"], value["extrinsics"]
        if "DistortionCoefficients" in intrinsics:
            distortion = intrinsics["DistortionCoefficients"]
        else:
            radial = intrinsics.get("RadialDistortion", [])
            tangential = intrinsics.get("TangentialDistortion", [])
            distortion = radial[:2] + tangential + radial[2:]
        cameras[name] = {
            "K": np.asarray(intrinsics["IntrinsicMatrix"], dtype=float).reshape(3, 3).T,
            "D": np.asarray(distortion, dtype=float),
            "size": tuple(reversed(intrinsics["ImageSize"])),
            # cTv maps vehicle coordinates into the camera. TF needs the
            # camera pose in the vehicle frame, vTc.
            "transform": np.linalg.inv(np.asarray(extrinsics["cTv"], dtype=float)),
        }
    return {
        "cameras": cameras,
        "lidar": np.asarray(raw["middle_lidar"]["extrinsics"]["vTl"], dtype=float),
        "adma": np.asarray(raw["adma"]["extrinsics"]["vTa"], dtype=float),
        "dimensions": raw.get("state", {}).get("dimension", [0.0, 0.0, 0.0]),
    }


def _load_labels(path: Path) -> Dict[str, List[Dict[str, Any]]]:
    with path.open() as file:
        tracks = json.load(file)["tracks"]
    labels: Dict[str, List[Dict[str, Any]]] = {}
    for track in tracks:
        for index, timestamp in enumerate(track["timestamps"]):
            labels.setdefault(str(timestamp), []).append(
                {
                    "id": track["track_id"],
                    "type": track["object_type"],
                    "position": track["positions"][index],
                    "orientation": track["orientations"][index],
                    "dimensions": track["dimensions"][0] if len(track["dimensions"]) == 1 else track["dimensions"][index],
                }
            )
    return labels


def _sensor_path(sequence_dir: Path, sensor: str, filename: str | None) -> Path:
    if not filename:
        raise FileNotFoundError(f"No synchronized {sensor} file in {sequence_dir}")
    directory = sequence_dir / sensor
    # The released archive uses short camera directory names, while the upstream
    # README documents vehicle_<camera>. Accept both layouts.
    if sensor in _CAMERAS and not directory.is_dir():
        directory = sequence_dir / f"vehicle_{sensor}"
    return directory / filename


def _load_json_sensor(sequence_dir: Path, sensor: str, filename: str | None) -> Dict[str, Any]:
    with _sensor_path(sequence_dir, sensor, filename).open() as file:
        return json.load(file)


def _state_value(state: Dict[str, Any], name: str) -> float:
    """Read native state values across the released and conversion-tool schemas."""
    for key in (name, f"ins_{name}"):
        if key in state:
            return float(state[key])
    raise KeyError(f"DrivIng vehicle state contains neither '{name}' nor 'ins_{name}'")


def _matrix_transform(parent: str, child: str, matrix: np.ndarray, stamp=None) -> TransformStamped:
    rotation = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    header = Header(frame_id=parent)
    if stamp is not None:
        header.stamp = stamp
    return TransformStamped(
        header=header,
        child_frame_id=child,
        transform=Transform(
            translation=Vector3(x=float(matrix[0, 3]), y=float(matrix[1, 3]), z=float(matrix[2, 3])),
            rotation=Quaternion(x=float(rotation[0]), y=float(rotation[1]), z=float(rotation[2]), w=float(rotation[3])),
        ),
    )


def _static_tf(calibration: Dict[str, Any]) -> List[TransformStamped]:
    transforms = [_matrix_transform("base_link", "lidar_01", calibration["lidar"])]
    transforms.extend(
        _matrix_transform("base_link", f"camera_{index:02d}", calibration["cameras"][camera]["transform"])
        for index, camera in enumerate(_CAMERAS, 1)
    )
    return transforms


def _vehicle_pose(state: Dict[str, Any], calibration: Dict[str, Any], origin):
    """Return the map-frame vehicle pose and the local origin the sequence is referenced to."""
    longitude = _state_value(state, "long_abs")
    latitude = _state_value(state, "lat_abs")
    altitude = _state_value(state, "height_msl")
    relative_position_available = "pos_rel_x" in state and "pos_rel_y" in state
    if origin is None:
        origin = {
            "longitude": longitude,
            "latitude": latitude,
            "altitude": altitude,
            "pos_rel_x": float(state.get("pos_rel_x", 0.0)),
            "pos_rel_y": float(state.get("pos_rel_y", 0.0)),
        }
    if relative_position_available:
        # The released state stores its local position as north/east. ROS map is ENU.
        east = float(state["pos_rel_y"]) - origin["pos_rel_y"]
        north = float(state["pos_rel_x"]) - origin["pos_rel_x"]
    else:
        east = (longitude - origin["longitude"]) * 111320.0 * np.cos(np.deg2rad(origin["latitude"]))
        north = (latitude - origin["latitude"]) * 110540.0
    roll = _state_value(state, "roll")
    pitch = _state_value(state, "pitch")
    yaw = _state_value(state, "yaw")
    global_from_adma = np.eye(4)
    # Convert north-referenced dataset yaw to ROS ENU yaw, whose zero points east.
    global_from_adma[:3, :3] = Rotation.from_euler("xyz", [roll, pitch, 90.0 + yaw], degrees=True).as_matrix()
    global_from_adma[:3, 3] = [east, north, altitude - origin["altitude"]]
    global_from_vehicle = global_from_adma @ np.linalg.inv(calibration["adma"])
    return global_from_vehicle, origin


def _ego_messages(
    global_from_vehicle: np.ndarray,
    calibration: Dict[str, Any],
    stamp,
    velocity: Optional[np.ndarray] = None,
):
    """Build the EgoData and TF messages of a frame from its map-frame vehicle pose.

    Args:
        global_from_vehicle: 4x4 transformation matrix (map <- vehicle).
        calibration: Calibration of the sequence, providing the vehicle dimensions.
        stamp: ROS Time message.
        velocity: Optional (3,) map-frame velocity vector [vx, vy, vz] from finite differencing.
    """
    ego = EgoData(header=Header(frame_id="map", stamp=stamp))
    pmu.initialize_state(ego.state, EGO.MODEL_ID)
    ego.state.reference_point = ObjectReferencePoint(value=ObjectReferencePoint.REAR_AXLE_GROUND)
    ego.state.continuous_state[EGO.X] = float(global_from_vehicle[0, 3])
    ego.state.continuous_state[EGO.Y] = float(global_from_vehicle[1, 3])
    ego.state.continuous_state[EGO.Z] = float(global_from_vehicle[2, 3])
    roll, pitch, yaw = Rotation.from_matrix(global_from_vehicle[:3, :3]).as_euler("xyz")
    ego.state.continuous_state[EGO.ROLL] = float(roll)
    ego.state.continuous_state[EGO.PITCH] = float(pitch)
    ego.state.continuous_state[EGO.YAW] = float(yaw)
    if velocity is not None:
        # Transform the map-frame velocity into the vehicle frame
        cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
        ego.state.continuous_state[EGO.VEL_LON] = float(cos_yaw * velocity[0] + sin_yaw * velocity[1])
        ego.state.continuous_state[EGO.VEL_LAT] = float(-sin_yaw * velocity[0] + cos_yaw * velocity[1])
        ego.state.discrete_state[EGO.STANDSTILL] = int(np.linalg.norm(velocity[:2]) < _STANDSTILL_VELOCITY)
    ego.length, ego.width, ego.height = (float(value) for value in calibration["dimensions"])
    return ego, TFMessage(transforms=[_matrix_transform("map", "base_link", global_from_vehicle, stamp)])


def _lidar_message(path: Path, stamp) -> PointCloud2:
    data = np.load(path)
    points = np.empty(
        len(data["x"]),
        dtype=[
            ("x", "<f4"),
            ("y", "<f4"),
            ("z", "<f4"),
            ("intensity", "<f4"),
            ("timestamp", "<f8"),
        ],
    )
    for name in ("x", "y", "z", "intensity", "timestamp"):
        points[name] = data[name]
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="timestamp", offset=16, datatype=PointField.FLOAT64, count=1),
    ]
    return create_cloud(Header(frame_id="lidar_01", stamp=stamp), fields, points)


def _object_list(labels: List[Dict[str, Any]], stamp, scene_id: str) -> ObjectList:
    message = ObjectList(header=Header(frame_id="lidar_01", stamp=stamp))
    for label in labels:
        obj = Object(id=int(label["id"]), existence_probability=1.0)
        pmu.initialize_state(obj.state, HEXAMOTION.MODEL_ID)
        obj.state.continuous_state[HEXAMOTION.X] = float(label["position"][0])
        obj.state.continuous_state[HEXAMOTION.Y] = float(label["position"][1])
        obj.state.continuous_state[HEXAMOTION.Z] = float(label["position"][2])
        obj.state.continuous_state[HEXAMOTION.YAW] = float(label["orientation"])
        length, width, height = label["dimensions"]
        obj.state.continuous_state[HEXAMOTION.LENGTH] = float(length)
        obj.state.continuous_state[HEXAMOTION.WIDTH] = float(width)
        obj.state.continuous_state[HEXAMOTION.HEIGHT] = float(height)
        obj.state.classifications = [
            ObjectClassification(
                type=_CLASS_MAPPING.get(label["type"], ObjectClassification.UNKNOWN),
                probability=1.0,
            )
        ]
        if hasattr(obj, "meta_info"):
            obj.meta_info.extend([f"scene_id:{scene_id}", f"original_class:{label['type']}"])
        message.objects.append(obj)
    return message


def _image_message(path: Path, stamp, frame_id: str) -> Image:
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"Failed to read image: {path}")
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    return Image(
        header=Header(frame_id=frame_id, stamp=stamp),
        height=image.shape[0],
        width=image.shape[1],
        encoding="rgb8",
        step=image.shape[1] * 3,
        data=image.tobytes(),
    )


def _camera_info(camera: Dict[str, Any], stamp, frame_id: str) -> CameraInfo:
    width, height = camera["size"]
    k = camera["K"]
    message = CameraInfo(header=Header(frame_id=frame_id, stamp=stamp), width=width, height=height)
    message.k = k.flatten().tolist()
    message.r = np.eye(3).flatten().tolist()
    message.p = [k[0, 0], k[0, 1], k[0, 2], 0.0, k[1, 0], k[1, 1], k[1, 2], 0.0, k[2, 0], k[2, 1], k[2, 2], 0.0]
    message.d = camera["D"].tolist()
    message.distortion_model = "equidistant" if len(camera["D"]) == 4 else "plumb_bob"
    return message
