# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

"""Native-file adapter for the TUM Traffic Dataset (TUMTraf).

The `TUM Traffic Dataset <https://innovation-mobility.com/en/project-providentia/a9-dataset/>`_
is recorded by roadside sensors mounted on gantry bridges of the Providentia++ test field near
Munich, Germany. It is therefore an *infrastructure* dataset: it has no ego vehicle, so no
``ego_data`` is published, and the sensors are static relative to the sensor station, which
is published as ``base_link``.

The dataset is published as one archive per release and subset (e.g. ``a9_dataset_r02_s04.zip``,
see the `release history <https://innovation-mobility.com/en/project-providentia/a9-dataset/>`_).
The releases share a common file layout, but differ in the sensors they contain, in the spelling
of their directory names, and in the format of their labels and calibration::

    <recording>/
        images|_images/<sensor_id>/<seconds>_<nanoseconds>_<sensor_id>.jpg
        point_clouds|_points_clouds/<sensor_id>/<seconds>_<nanoseconds>_<sensor_id>.pcd
        labels_point_clouds|_labels/<sensor_id>/<seconds>_<nanoseconds>_<sensor_id>.json
        _calibration/<sensor_id>.json

Rather than hard-coding every release, this adapter discovers the recordings below the dataset
directory, derives the sensors and frame timestamps from the file names, and maps the sensors
onto the canonical ``camera_XX`` / ``lidar_XX`` topics.

Two label formats hold real 3D cuboids and are converted into an object list: the OpenLABEL
format (``R02`` and newer) and the native pre-OpenLABEL format of the lidar subsets of ``R00``
(``r00_s03``, ``r00_s04``), which annotates a cuboid directly by its location, dimensions and
yaw (see the dev kit's own loader in ``src/utils/vis_utils.py`` for reference; only yaw is used,
as roll and pitch are not populated meaningfully in this format).

The earliest camera-only releases (the image subsets of ``R00``/``R01``) instead annotate a 3D
box projected into the image, i.e. only its 2D pixel-plane silhouette, with the actual 3D pose
that produced it not released. That data cannot be turned into a real 3D or 2D detection without
guessing (assuming an object's dimensions and a ground plane to resolve the missing depth), so
it is intentionally not converted; recordings with only this label format still publish their
raw camera images, calibration and transforms, just no ``object_list``.

All calibration is taken from the dataset itself: from the ``_calibration`` directory of a
recording if it ships one, otherwise from the ``coordinate_systems`` and ``streams`` sections of
its OpenLABEL label files.

The dataset requires registration and cannot be downloaded automatically. Archives that are
placed in the dataset directory are extracted on the first run.
"""

import json
import re
import zipfile
from bisect import bisect_left
from pathlib import Path, PurePosixPath
from typing import Any, Dict, Iterator, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import perception_msgs_utils as pmu
from autonomy_datasets.datasets.dataset import DatasetAdapter
from autonomy_datasets.datasets.utils import timestamp_micros_to_clock
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from perception_msgs.msg import HEXAMOTION, Object, ObjectClassification, ObjectList
from pypcd4 import PointCloud
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

# Directory names the releases use per modality. The naming is inconsistent across releases
# (e.g. "point_clouds" in R02 vs. "_points" in R00), so all known spellings are accepted.
_IMAGE_DIR_NAMES = ("_images", "images")
_POINT_CLOUD_DIR_NAMES = ("_points_clouds", "_point_clouds", "point_clouds", "_points", "points")
_LABEL_DIR_NAMES = ("_labels_point_clouds", "labels_point_clouds", "_labels_images", "_labels", "labels")
_CALIBRATION_DIR_NAMES = ("_calibration", "calibration")
_DATA_DIR_NAMES = frozenset(_IMAGE_DIR_NAMES + _POINT_CLOUD_DIR_NAMES + _LABEL_DIR_NAMES + _CALIBRATION_DIR_NAMES)

_IMAGE_SUFFIXES = (".png", ".jpg", ".jpeg")
_POINT_CLOUD_SUFFIXES = (".pcd",)
_LABEL_SUFFIXES = (".json",)

# Sensor data is named "<seconds>_<nanoseconds>_<sensor_id>"; a few releases omit the sensor ID
# and identify the sensor by the containing directory instead.
_FRAME_FILE_PATTERN = re.compile(r"^(?P<seconds>\d{9,11})_(?P<nanoseconds>\d{9})(?:_(?P<sensor>.+))?$")

# Station prefixes are written with and without leading zeros (e.g. "s40" and "s040").
_STATION_PREFIX_PATTERN = re.compile(r"^s(\d+)_")

# Calibration key holding the transformation from a sensor into a station base frame.
_SENSOR_TO_BASE_KEY_PATTERN = re.compile(r"^transformation_matrix_.+_to_.*base$")

# Canonical topic order of the known sensors. Sensors are numbered in this order, so that
# camera_01 and lidar_01 are the sensors the object lists are annotated in. Unknown sensors are
# appended in alphabetical order.
_CAMERA_ORDER = (
    "s110_camera_basler_south1_8mm",
    "s110_camera_basler_south2_8mm",
    "s110_camera_basler_east_8mm",
    "s110_camera_basler_north_8mm",
    "s040_camera_basler_north_16mm",
    "s040_camera_basler_north_50mm",
    "s050_camera_basler_south_16mm",
    "s050_camera_basler_south_50mm",
    "vehicle_camera_basler_16mm",
)
_LIDAR_ORDER = (
    "s110_lidar_ouster_south",
    "s110_lidar_ouster_south_and_vehicle_lidar_robosense_registered",
    "s110_lidar_ouster_north",
    "vehicle_lidar_robosense",
)

# Mapping from dataset class names to ROS ObjectClassification types. Every converted label
# format writes the classes in upper case, so keys are matched upper case. TRUCK, TRAILER and
# VAN are deprecated in perception_msgs and map to UTILITY / CAR.
_CLASS_MAPPING: Dict[str, int] = {
    "CAR": ObjectClassification.CAR,
    "TRUCK": ObjectClassification.UTILITY,
    "TRAILER": ObjectClassification.UTILITY,
    "VAN": ObjectClassification.CAR,
    "MOTORCYCLE": ObjectClassification.MOTORCYCLE,
    "BUS": ObjectClassification.BUS,
    "PEDESTRIAN": ObjectClassification.PEDESTRIAN,
    "BICYCLE": ObjectClassification.BICYCLE,
    "EMERGENCY_VEHICLE": ObjectClassification.UTILITY,
    "OTHER": ObjectClassification.UNKNOWN,
}

# Maximum recursion depth used to look for recordings below the dataset directory.
_MAX_DISCOVERY_DEPTH = 4

# Directory the node stores its generated rosbags in, see autonomy_datasets.datasets.rosbag.
_ROSBAG_DIR_NAME = "bags"

_PRINTED_MESSAGES: set = set()


class TumTrafficAdapter(DatasetAdapter):
    """Converts native TUM Traffic Dataset files to normalized ROS 2 messages."""

    VERSION = "1.0.0"
    RELEASE_NOTES = {"1.0.0": "Initial integration into Autonomy.Datasets"}

    def __init__(
        self,
        data_publishers: Dict[str, Any],
        dataset_root_dir: str,
        split: str,
        publish_camera_images: bool = True,
        publish_lidar_pointclouds: bool = True,
        publish_lidar_object_lists: bool = True,
        extract_archives: bool = True,
        sync_tolerance_seconds: float = 0.1,
        rosbag_duration_seconds: float = 20.0,
        labels_in_base_frame: bool = False,
        start_scene_index: int = 0,
    ) -> None:
        """Initialize the adapter, extract downloaded archives and discover the recordings.

        Args:
            data_publishers: Mapping of topic names to publisher instances.
            dataset_root_dir: Root directory holding the downloaded archives or recordings.
            split: Filter selecting the recordings to publish; ``all`` selects every recording,
                any other value selects the recordings whose path contains it (e.g. ``r02``).
            publish_camera_images: Whether to publish camera images.
            publish_lidar_pointclouds: Whether to publish lidar point clouds.
            publish_lidar_object_lists: Whether to publish lidar_01 object lists.
            extract_archives: Whether to extract downloaded archives found in the dataset directory.
            sync_tolerance_seconds: Maximum time difference for matching a sensor to a frame.
            rosbag_duration_seconds: Duration of each rosbag scene in seconds.
            labels_in_base_frame: Whether 3D labels are annotated in the station base frame
                instead of the frame of the sensor they are stored for.
            start_scene_index: Number of scenes to skip before generating samples.

        Raises:
            ValueError: If a configuration value is out of range.
            FileNotFoundError: If the dataset directory holds no matching recording.
        """
        super().__init__(data_publishers=data_publishers)
        if sync_tolerance_seconds <= 0:
            raise ValueError("TUM Traffic sync_tolerance_seconds must be greater than 0")
        if rosbag_duration_seconds <= 0:
            raise ValueError("TUM Traffic rosbag_duration_seconds must be greater than 0")

        self.dataset_root_dir = Path(dataset_root_dir)
        self.split = split
        self.publish_camera_images = publish_camera_images
        self.publish_lidar_pointclouds = publish_lidar_pointclouds
        self.sync_tolerance_ns = int(sync_tolerance_seconds * 1e9)
        self.rosbag_duration_ns = int(rosbag_duration_seconds * 1e9)
        self.labels_in_base_frame = labels_in_base_frame
        self.start_scene_index = start_scene_index

        if not self.dataset_root_dir.is_dir():
            raise FileNotFoundError(
                f"TUM Traffic data directory not found: '{self.dataset_root_dir}'. The dataset requires "
                f"registration at https://a9-dataset.innovation-mobility.com/en/register; download the "
                f"archives and place them in this directory."
            )
        if extract_archives:
            _extract_archives(self.dataset_root_dir)

        self.recordings = _discover_recordings(self.dataset_root_dir, split)
        if not self.recordings:
            raise FileNotFoundError(
                f"No TUM Traffic recording matching split '{split}' found in '{self.dataset_root_dir}'. "
                f"Download the archives from https://a9-dataset.innovation-mobility.com/downloads and "
                f"place them in this directory."
            )
        print(
            f"Found {len(self.recordings)} TUM Traffic recording(s) for split '{split}': "
            f"{', '.join(recording.name for recording in self.recordings)}",
            flush=True,
        )

        # Sensors are numbered across all selected recordings, so that a sensor keeps its topic
        # even when a recording of the split does not contain it.
        self.camera_topics = _assign_topics(self._sensor_ids("camera"), _CAMERA_ORDER, "camera")
        self.lidar_topics = _assign_topics(self._sensor_ids("lidar"), _LIDAR_ORDER, "lidar")
        self._warn_about_heterogeneous_recordings()

        self.reference_lidar = next(iter(self.lidar_topics), None)
        self.publish_lidar_object_lists = publish_lidar_object_lists and self._has_labels(self.reference_lidar)

        if self.publish_lidar_pointclouds:
            for topic in self.lidar_topics.values():
                self.data_publishers[f"{topic}/point_cloud"] = None
        if self.publish_camera_images:
            for topic in self.camera_topics.values():
                self.data_publishers[f"{topic}/image_raw"] = None
                self.data_publishers[f"{topic}/camera_info"] = None
        if self.publish_lidar_object_lists:
            self.data_publishers["object_list/lidar_01"] = None

    def generate_samples(self) -> Iterator[Tuple[int, Dict[str, Any]]]:
        """Yield time-synchronized native TUM Traffic frames of every selected recording."""
        sample_index = 0
        scene_index = 0
        for recording in self.recordings:
            reference_sensor = self._reference_sensor(recording)
            if reference_sensor is None:
                print(f"Skipping TUM Traffic recording '{recording.name}': no enabled sensor data", flush=True)
                continue
            frames = self._synchronized_frames(recording, reference_sensor)
            if not frames:
                print(f"Skipping TUM Traffic recording '{recording.name}': no synchronized frames", flush=True)
                continue

            calibration = recording.calibration()
            static_tf = TFMessage(transforms=self._static_transforms(recording, calibration, reference_sensor))
            track_ids: Dict[str, int] = {}
            first_timestamp_ns = frames[0][0]
            last_scene_id = None
            for timestamp_ns, sensor_frames in frames:
                scene_id = f"{recording.name}_{(timestamp_ns - first_timestamp_ns) // self.rosbag_duration_ns + 1:05d}"
                if scene_id != last_scene_id:
                    last_scene_id = scene_id
                    scene_index += 1
                if scene_index <= self.start_scene_index:
                    continue

                clock = timestamp_micros_to_clock(timestamp_ns // 1000)
                sample: Dict[str, Any] = {
                    "scene_id": scene_id,
                    "/clock": clock,
                    "/tf_static": static_tf,
                    # The sensors are mounted on a static roadside station, so the station's
                    # base frame is at rest in the map frame.
                    "/tf": TFMessage(transforms=[_matrix_transform("map", "base_link", calibration.map_to_base, clock.clock)]),
                }

                if self.publish_lidar_pointclouds:
                    for sensor_id, topic in self.lidar_topics.items():
                        sample[f"{topic}/point_cloud"] = _point_cloud_message(sensor_frames.get(sensor_id), clock.clock, topic)
                if self.publish_camera_images:
                    for sensor_id, topic in self.camera_topics.items():
                        frame = sensor_frames.get(sensor_id)
                        sample[f"{topic}/image_raw"] = _image_message(frame, clock.clock, topic)
                        sample[f"{topic}/camera_info"] = _camera_info_message(
                            calibration.intrinsics.get(sensor_id, {}), clock.clock, topic
                        )
                if self.publish_lidar_object_lists:
                    sample["object_list/lidar_01"] = _object_list_3d(
                        recording.labels_at(self.reference_lidar, timestamp_ns, self.sync_tolerance_ns),
                        "base_link" if self.labels_in_base_frame else "lidar_01",
                        clock.clock,
                        scene_id,
                        track_ids,
                    )

                sample_index += 1
                yield sample_index, sample

    def _sensor_ids(self, modality: str) -> List[str]:
        """Return the IDs of all sensors of a modality found in the selected recordings."""
        sensor_ids = set()
        for recording in self.recordings:
            sensor_ids.update(sensor_id for sensor_id, sensor in recording.sensors.items() if sensor.modality == modality)
        return sorted(sensor_ids)

    def _has_labels(self, sensor_id: Optional[str]) -> bool:
        """Report whether any selected recording annotates the given sensor."""
        return sensor_id is not None and any(recording.labels.get(sensor_id) for recording in self.recordings)

    def _warn_about_heterogeneous_recordings(self) -> None:
        """Report recordings that do not contain every sensor of the selected split."""
        expected = set(self.camera_topics) | set(self.lidar_topics)
        incomplete = [recording.name for recording in self.recordings if set(recording.sensors) != expected]
        if incomplete:
            print(
                f"TUM Traffic recording(s) {', '.join(incomplete)} do not contain all sensors of split "
                f"'{self.split}'; their missing sensors are published as empty messages. Select a "
                f"release-specific split (e.g. 'r02') to publish a homogeneous set of sensors.",
                flush=True,
            )

    def _reference_sensor(self, recording: "_Recording") -> Optional[str]:
        """Return the sensor whose frames drive the playback of a recording."""
        candidates = []
        if self.publish_lidar_pointclouds or self.publish_lidar_object_lists:
            candidates.extend(self.lidar_topics)
        if self.publish_camera_images:
            candidates.extend(self.camera_topics)
        return next((sensor_id for sensor_id in candidates if sensor_id in recording.sensors), None)

    def _synchronized_frames(self, recording: "_Recording", reference_sensor: str) -> List[Tuple[int, Dict[str, "_Frame"]]]:
        """Match every sensor of a recording to the frames of its reference sensor.

        The dataset ships no synchronization table and its sensors are triggered independently,
        so each sensor contributes the frame closest to the reference timestamp. Frames without
        a match within the configured tolerance are skipped, because a sample has to hold data
        for every published topic.
        """
        published = set()
        if self.publish_lidar_pointclouds:
            published.update(self.lidar_topics)
        if self.publish_camera_images:
            published.update(self.camera_topics)
        required = [sensor_id for sensor_id in recording.sensors if sensor_id in published]

        frames = []
        skipped = 0
        for timestamp_ns in recording.sensors[reference_sensor].timestamps:
            sensor_frames = {}
            for sensor_id in required:
                matched = recording.sensors[sensor_id].closest(timestamp_ns, self.sync_tolerance_ns)
                if matched is None:
                    skipped += 1
                    break
                sensor_frames[sensor_id] = matched
            else:
                frames.append((timestamp_ns, sensor_frames))
        if skipped:
            print(
                f"TUM Traffic recording '{recording.name}': {skipped} of "
                f"{len(recording.sensors[reference_sensor].timestamps)} frames are missing data from one or "
                f"more enabled sensors and will be skipped; {len(frames)} frames are available.",
                flush=True,
            )
        return frames

    def _static_transforms(
        self, recording: "_Recording", calibration: "_Calibration", reference_sensor: str
    ) -> List[TransformStamped]:
        """Build the static transforms from the station base frame to every sensor frame.

        A few recordings (the ``R00`` lidar subsets) ship no extrinsic calibration for any of
        their sensors, in either the dataset's ``_calibration`` directory or their OpenLABEL
        label files. Publishing no static transform at all would leave ``base_link`` without a
        connection to any sensor frame, breaking tools that assume it exists (e.g. RViz's
        ``Fixed Frame``). Since there is no calibrated pose to fall back to, ``base_link`` is
        instead aliased to the recording's reference sensor with an identity transform.
        """
        transforms = []
        alias_sensor_id = None
        if not any(sensor_id in calibration.base_to_sensor for sensor_id in recording.sensors):
            alias_sensor_id = reference_sensor
            alias_topic = self.camera_topics.get(alias_sensor_id) or self.lidar_topics.get(alias_sensor_id)
            _print_once(
                f"TUM Traffic recording '{recording.name}' ships no extrinsic calibration; "
                f"aliasing 'base_link' to its reference sensor '{alias_topic}' instead."
            )
            transforms.append(_matrix_transform("base_link", alias_topic, np.eye(4)))
        for sensor_id, topic in (*self.camera_topics.items(), *self.lidar_topics.items()):
            if sensor_id not in recording.sensors or sensor_id == alias_sensor_id:
                continue
            base_to_sensor = calibration.base_to_sensor.get(sensor_id)
            if base_to_sensor is None:
                _print_once(
                    f"TUM Traffic sensor '{sensor_id}' ships no extrinsic calibration; "
                    f"no transform to '{topic}' is published."
                )
                continue
            transforms.append(_matrix_transform("base_link", topic, np.linalg.inv(base_to_sensor)))
        return transforms


class _Frame:
    """A single native sensor file together with the timestamp encoded in its name."""

    def __init__(self, timestamp_ns: int, path: Path) -> None:
        self.timestamp_ns = timestamp_ns
        self.path = path


class _SensorFrames:
    """Time-sorted native frames of a single sensor within a recording."""

    def __init__(self, sensor_id: str, modality: str, files: Dict[int, Path]) -> None:
        self.sensor_id = sensor_id
        self.modality = modality
        self.files = files
        self.timestamps = sorted(files)

    def closest(self, timestamp_ns: int, tolerance_ns: int) -> Optional[_Frame]:
        """Return the frame recorded closest to a timestamp, or None outside the tolerance."""
        index = bisect_left(self.timestamps, timestamp_ns)
        candidates = self.timestamps[max(index - 1, 0) : index + 1]
        if not candidates:
            return None
        closest = min(candidates, key=lambda candidate: abs(candidate - timestamp_ns))
        if abs(closest - timestamp_ns) > tolerance_ns:
            return None
        return _Frame(closest, self.files[closest])


class _Calibration:
    """Intrinsic and extrinsic calibration of one recording."""

    def __init__(
        self,
        map_to_base: Optional[np.ndarray] = None,
        base_to_sensor: Optional[Dict[str, np.ndarray]] = None,
        intrinsics: Optional[Dict[str, Dict[str, Any]]] = None,
    ) -> None:
        #: Pose of the station base frame in the map frame
        self.map_to_base = np.eye(4) if map_to_base is None else map_to_base
        #: Transformation from the station base frame into the respective sensor frame
        self.base_to_sensor = base_to_sensor or {}
        #: Camera matrix, distortion coefficients and image size per camera
        self.intrinsics = intrinsics or {}


class _Recording:
    """A single recorded TUM Traffic subset, published as one or more ROS scenes."""

    def __init__(
        self,
        name: str,
        path: Path,
        sensors: Dict[str, _SensorFrames],
        labels: Dict[str, Dict[int, Path]],
        calibration_files: Dict[str, Path],
    ) -> None:
        self.name = name
        self.path = path
        self.sensors = sensors
        self.labels = labels
        self.calibration_files = calibration_files
        self._calibration: Optional[_Calibration] = None

    def calibration(self) -> _Calibration:
        """Return the calibration of the recording, reading it from the dataset on first use.

        Releases that ship a ``_calibration`` directory are calibrated from its files; the
        releases without one (``R02`` and newer) describe their sensor setup in the
        ``coordinate_systems`` and ``streams`` sections of every OpenLABEL label file.
        """
        if self._calibration is None:
            if self.calibration_files:
                self._calibration = _calibration_from_files(self.calibration_files)
            else:
                self._calibration = _calibration_from_openlabel(self._label_files_for_calibration())
        return self._calibration

    def labels_at(self, sensor_id: Optional[str], timestamp_ns: int, tolerance_ns: int) -> List[Dict[str, Any]]:
        """Read the annotations of a sensor at the frame closest to a timestamp."""
        if sensor_id is None:
            return []
        files = self.labels.get(sensor_id)
        if not files:
            return []
        timestamps = sorted(files)
        index = bisect_left(timestamps, timestamp_ns)
        candidates = timestamps[max(index - 1, 0) : index + 1]
        closest = min(candidates, key=lambda candidate: abs(candidate - timestamp_ns))
        if abs(closest - timestamp_ns) > tolerance_ns:
            return []
        return _load_labels(files[closest])

    def _label_files_for_calibration(self) -> List[Path]:
        """Return one label file per annotated sensor, used to read the sensor setup.

        Each label file only describes the sensors it relates to, so the setup is read from one
        file per annotated sensor. The files are ordered by sensor, so that the calibration of
        the reference sensors wins where the files disagree.
        """
        files = []
        for sensor_id in sorted(self.labels, key=_sensor_sort_key):
            sensor_files = self.labels[sensor_id]
            if sensor_files:
                files.append(sensor_files[min(sensor_files)])
        return files


def _extract_archives(dataset_root: Path) -> None:
    """Extract downloaded dataset archives into a directory named after the archive.

    The TUM Traffic Dataset requires registration and is downloaded manually, so the archives
    are picked up from the dataset directory instead of being fetched automatically.
    """
    for archive_path in sorted(dataset_root.glob("*.zip")):
        target = dataset_root / archive_path.stem
        if target.is_dir() and any(target.iterdir()):
            continue
        print(f"Extracting TUM Traffic archive '{archive_path.name}' into '{target}'.", flush=True)
        with zipfile.ZipFile(archive_path) as archive:
            members = [member for member in archive.infolist() if not member.is_dir()]
            # Archives are released with and without a wrapping top-level directory; strip it so
            # that the extracted layout is the same in both cases.
            top_level = {PurePosixPath(member.filename).parts[0] for member in members}
            strip_prefix = len(top_level) == 1 and all(len(PurePosixPath(member.filename).parts) > 1 for member in members)
            for member in members:
                parts = PurePosixPath(member.filename).parts[1:] if strip_prefix else PurePosixPath(member.filename).parts
                if not parts:
                    continue
                destination = (target / Path(*parts)).resolve()
                if target.resolve() not in destination.parents:
                    raise ValueError(f"Unsafe path in TUM Traffic archive: {member.filename}")
                destination.parent.mkdir(parents=True, exist_ok=True)
                with archive.open(member) as source, destination.open("wb") as output:
                    while chunk := source.read(1024 * 1024):
                        output.write(chunk)


def _discover_recordings(dataset_root: Path, split: str) -> List[_Recording]:
    """Find all recordings below the dataset directory that match the requested split."""
    recordings = []
    for directory in _find_recording_dirs(dataset_root):
        relative = directory.relative_to(dataset_root)
        # A recording placed directly in the dataset directory is named after that directory
        name = "_".join(relative.parts) if relative.parts else dataset_root.name
        if split != "all" and split.lower() not in name.lower():
            continue
        recording = _load_recording(name, directory)
        if recording.sensors:
            recordings.append(recording)
    return sorted(recordings, key=lambda recording: recording.name)


def _find_recording_dirs(directory: Path, depth: int = 0) -> List[Path]:
    """Return the directories that hold the modality subdirectories of a recording."""
    subdirectories = sorted(
        path
        for path in directory.iterdir()
        # The generated rosbags live next to the recordings and hold no native data
        if path.is_dir() and not path.name.startswith(".") and not (depth == 0 and path.name == _ROSBAG_DIR_NAME)
    )
    if any(subdirectory.name in _DATA_DIR_NAMES for subdirectory in subdirectories):
        return [directory]
    if depth >= _MAX_DISCOVERY_DEPTH:
        return []
    found = []
    for subdirectory in subdirectories:
        found.extend(_find_recording_dirs(subdirectory, depth + 1))
    return found


def _load_recording(name: str, directory: Path) -> _Recording:
    """Index the sensor data, labels and calibration files of a single recording."""
    sensors: Dict[str, _SensorFrames] = {}
    for modality, dir_names, suffixes in (
        ("lidar", _POINT_CLOUD_DIR_NAMES, _POINT_CLOUD_SUFFIXES),
        ("camera", _IMAGE_DIR_NAMES, _IMAGE_SUFFIXES),
    ):
        for sensor_id, files in _collect_frame_files(directory, dir_names, suffixes).items():
            sensors[sensor_id] = _SensorFrames(sensor_id, modality, files)

    labels = _collect_frame_files(directory, _LABEL_DIR_NAMES, _LABEL_SUFFIXES)

    calibration_files: Dict[str, Path] = {}
    for dir_name in _CALIBRATION_DIR_NAMES:
        calibration_dir = directory / dir_name
        if not calibration_dir.is_dir():
            continue
        for path in sorted(calibration_dir.rglob("*.json")):
            calibration_files.setdefault(_normalize_sensor_id(path.stem), path)

    return _Recording(name, directory, sensors, labels, calibration_files)


def _collect_frame_files(directory: Path, dir_names: Sequence[str], suffixes: Sequence[str]) -> Dict[str, Dict[int, Path]]:
    """Index the files of every sensor below the given modality directories by timestamp."""
    collected: Dict[str, Dict[int, Path]] = {}
    for dir_name in dir_names:
        modality_dir = directory / dir_name
        if not modality_dir.is_dir():
            continue
        for path in sorted(modality_dir.rglob("*")):
            if not path.is_file() or path.suffix.lower() not in suffixes:
                continue
            match = _FRAME_FILE_PATTERN.match(path.stem)
            if match is None:
                _print_once(f"Ignoring TUM Traffic file with unexpected name: '{path}'")
                continue
            sensor_id = _normalize_sensor_id(match.group("sensor") or path.parent.name)
            timestamp_ns = int(match.group("seconds")) * 1_000_000_000 + int(match.group("nanoseconds"))
            collected.setdefault(sensor_id, {})[timestamp_ns] = path
    return collected


def _normalize_sensor_id(sensor_id: str) -> str:
    """Normalize the station prefix of a sensor ID, which is written as 's40' and as 's040'."""
    match = _STATION_PREFIX_PATTERN.match(sensor_id)
    if match is None:
        return sensor_id
    return f"s{int(match.group(1)):03d}_{sensor_id[match.end():]}"


def _assign_topics(sensor_ids: Sequence[str], known_order: Sequence[str], prefix: str) -> Dict[str, str]:
    """Map sensor IDs onto canonical ``<prefix>_XX`` topic names in a deterministic order."""
    ordered = [sensor_id for sensor_id in known_order if sensor_id in sensor_ids]
    ordered.extend(sorted(sensor_id for sensor_id in sensor_ids if sensor_id not in known_order))
    return {sensor_id: f"{prefix}_{index:02d}" for index, sensor_id in enumerate(ordered, 1)}


def _sensor_sort_key(sensor_id: str) -> Tuple[int, int, str]:
    """Sort sensors by their canonical topic order, unknown sensors alphabetically last."""
    known_order = _LIDAR_ORDER + _CAMERA_ORDER
    if sensor_id in known_order:
        return (0, known_order.index(sensor_id), sensor_id)
    return (1, 0, sensor_id)


def _calibration_from_files(calibration_files: Dict[str, Path]) -> _Calibration:
    """Read the calibration from the ``_calibration`` directory of a recording (R00/R01)."""
    base_to_sensor = {}
    intrinsics = {}
    for sensor_id, path in calibration_files.items():
        with path.open() as file:
            data = json.load(file)
        extrinsic = _base_to_sensor(data)
        if extrinsic is not None:
            base_to_sensor[sensor_id] = extrinsic
        camera_matrix = data.get("intrinsic_camera_matrix") or data.get("intrinsic_matrix")
        if camera_matrix:
            intrinsics[sensor_id] = {
                "camera_matrix": np.asarray(camera_matrix, dtype=float).reshape(3, 3),
                "distortion": [float(value) for value in data.get("dist_coefficients", [])],
                "width": int(data.get("image_width", 0)),
                "height": int(data.get("image_height", 0)),
            }
    return _Calibration(base_to_sensor=base_to_sensor, intrinsics=intrinsics)


def _base_to_sensor(data: Dict[str, Any]) -> Optional[np.ndarray]:
    """Return the transformation from the station base frame into the sensor frame.

    The releases store the extrinsic calibration under different keys and in both directions,
    so the known spellings are resolved in the order of their specificity. ``rotation_matrix``
    is paired with ``translation_matrix`` in the base-to-sensor direction, but with
    ``translation`` in the sensor-to-base direction.
    """
    for key in ("transformation_matrix", "extrinsic_matrix", "transformation_sensor_station_base_to_sensor"):
        if key in data:
            return _to_homogeneous(np.asarray(data[key], dtype=float))
    if "rotation_matrix" in data and "translation_matrix" in data:
        return _compose(data["rotation_matrix"], data["translation_matrix"])
    for key, value in data.items():
        if _SENSOR_TO_BASE_KEY_PATTERN.match(key):
            return np.linalg.inv(_to_homogeneous(np.asarray(value, dtype=float)))
    if "rotation_matrix" in data and "translation" in data:
        # Pose of the sensor in the base frame, i.e. the inverse of the extrinsic calibration
        return np.linalg.inv(_compose(data["rotation_matrix"], data["translation"]))
    return None


def _calibration_from_openlabel(label_paths: Sequence[Path]) -> _Calibration:
    """Read the sensor setup from the OpenLABEL labels of a recording (R02 and newer).

    The label files describe the sensor setup as a tree of coordinate systems rooted in a scene
    coordinate system (published as ``map``) that holds the station base frame (published as
    ``base_link``). A label file only describes the sensors it relates to, so the setups of all
    given files are merged; where they disagree, the first file wins.
    """
    if not label_paths:
        _print_once("No TUM Traffic calibration found; sensor transforms and camera info are published empty")
        return _Calibration()
    calibration = _Calibration()
    for label_path in label_paths:
        with label_path.open() as file:
            data = json.load(file).get("openlabel", {})
        systems = data.get("coordinate_systems", {})
        streams = data.get("streams", {})
        if not systems:
            _print_once(f"TUM Traffic labels '{label_path}' hold no coordinate systems; sensor transforms are not published")
            continue

        base_id = next(
            (name for name, system in systems.items() if system.get("type") == "local_cs"),
            next((name for name, system in systems.items() if not system.get("parent")), None),
        )
        for name in systems:
            sensor_id = _normalize_sensor_id(name)
            if name == base_id or sensor_id in calibration.base_to_sensor:
                continue
            transform = _compose_openlabel_chain(systems, streams, name, base_id)
            if transform is not None:
                calibration.base_to_sensor[sensor_id] = transform

        # The base coordinate system is not a camera, so its matrix is the pose of the station
        # base frame in the scene coordinate system published as map.
        base_pose = _openlabel_matrix(systems.get(base_id, {}))
        if base_pose is not None:
            calibration.map_to_base = base_pose

        for name, stream in streams.items():
            pinhole = stream.get("stream_properties", {}).get("intrinsics_pinhole")
            sensor_id = _normalize_sensor_id(name)
            if not pinhole or sensor_id in calibration.intrinsics:
                continue
            camera_matrix = np.asarray(pinhole["camera_matrix_3x4"], dtype=float).reshape(3, 4)
            calibration.intrinsics[sensor_id] = {
                "camera_matrix": camera_matrix[:, :3],
                # The released images of these subsets are already undistorted
                "distortion": [float(value) for value in pinhole.get("distortion_coeffs_1xN", [])],
                "width": int(pinhole.get("width_px", 0)),
                "height": int(pinhole.get("height_px", 0)),
            }
    return calibration


def _compose_openlabel_chain(
    systems: Dict[str, Any], streams: Dict[str, Any], name: str, base_id: Optional[str]
) -> Optional[np.ndarray]:
    """Compose the transformation from the base coordinate system into the given one."""
    transform = np.eye(4)
    while name != base_id:
        system = systems.get(name)
        if system is None:
            return None
        matrix = _openlabel_matrix(system)
        if matrix is None:
            return None
        transform = transform @ _parent_to_child(name, streams, matrix)
        parent = system.get("parent")
        if not parent:
            # The coordinate system is not a descendant of the base coordinate system
            return None
        name = parent
    return transform


def _parent_to_child(name: str, streams: Dict[str, Any], matrix: np.ndarray) -> np.ndarray:
    """Return the transformation from a coordinate system's parent into the system itself.

    The dataset writes the ``pose_wrt_parent`` of a camera as its extrinsic calibration, which
    maps points of the parent into the camera, but the ``pose_wrt_parent`` of every other
    coordinate system as its pose in the parent, which is the inverse direction.
    """
    if streams.get(name, {}).get("type") == "camera" or "camera" in name:
        return matrix
    return np.linalg.inv(matrix)


def _openlabel_matrix(system: Dict[str, Any]) -> Optional[np.ndarray]:
    """Return the 4x4 pose matrix of an OpenLABEL coordinate system, if it has one."""
    matrix = system.get("pose_wrt_parent", {}).get("matrix4x4")
    if not matrix:
        return None
    return np.asarray(matrix, dtype=float).reshape(4, 4)


def _compose(rotation: Any, translation: Any) -> np.ndarray:
    """Build a 4x4 transformation matrix from a rotation matrix and a translation."""
    return _to_homogeneous(np.hstack([np.asarray(rotation, dtype=float).reshape(3, 3), _to_vector(translation).reshape(3, 1)]))


def _to_homogeneous(matrix: np.ndarray) -> np.ndarray:
    """Return a 4x4 transformation matrix from a 3x4 or 4x4 matrix."""
    if matrix.shape == (4, 4):
        return matrix
    return np.vstack([matrix, [0.0, 0.0, 0.0, 1.0]])


def _to_vector(value: Any) -> np.ndarray:
    """Return a translation vector from a list or from an {x, y, z} mapping."""
    if isinstance(value, dict):
        return np.asarray([value["x"], value["y"], value["z"]], dtype=float)
    return np.asarray(value, dtype=float).reshape(3)


def _matrix_transform(parent: str, child: str, matrix: np.ndarray, stamp: Optional[Time] = None) -> TransformStamped:
    """Build a TransformStamped message from a 4x4 transformation matrix."""
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


def _point_cloud_message(frame: Optional[_Frame], stamp: Time, frame_id: str) -> PointCloud2:
    """Load a native ``.pcd`` file and convert it to a ROS PointCloud2 message."""
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="timestamp", offset=16, datatype=PointField.FLOAT64, count=1),
    ]
    dtype = [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4"), ("timestamp", "<f8")]
    if frame is None:
        return create_cloud(Header(frame_id=frame_id, stamp=stamp), fields, np.empty(0, dtype=dtype))
    point_cloud = PointCloud.from_path(str(frame.path))
    data = point_cloud.pc_data
    points = np.empty(len(data["x"]), dtype=dtype)
    for name in ("x", "y", "z"):
        points[name] = data[name]
    points["intensity"] = data["intensity"] if "intensity" in point_cloud.fields else 0.0
    # The Ouster lidars store the offset of a point within its scan in nanoseconds; publish the
    # native per-point timing as absolute seconds.
    offsets = data["t"] * 1e-9 if "t" in point_cloud.fields else 0.0
    points["timestamp"] = frame.timestamp_ns * 1e-9 + offsets
    return create_cloud(Header(frame_id=frame_id, stamp=stamp), fields, points)


def _image_message(frame: Optional[_Frame], stamp: Time, frame_id: str) -> Image:
    """Load a native image file and convert it to a ROS Image message."""
    if frame is None:
        return Image(header=Header(frame_id=frame_id, stamp=stamp), encoding="rgb8")
    image = cv2.imread(str(frame.path), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"Failed to read image: {frame.path}")
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    return Image(
        header=Header(frame_id=frame_id, stamp=stamp),
        height=image.shape[0],
        width=image.shape[1],
        encoding="rgb8",
        step=image.shape[1] * 3,
        data=image.tobytes(),
    )


def _camera_info_message(intrinsics: Dict[str, Any], stamp: Time, frame_id: str) -> CameraInfo:
    """Convert a native camera calibration to a ROS CameraInfo message."""
    message = CameraInfo(header=Header(frame_id=frame_id, stamp=stamp))
    message.width = intrinsics.get("width", 0)
    message.height = intrinsics.get("height", 0)
    message.r = np.eye(3).flatten().tolist()
    camera_matrix = intrinsics.get("camera_matrix")
    if camera_matrix is not None:
        message.k = camera_matrix.flatten().tolist()
        message.p = np.hstack([camera_matrix, np.zeros((3, 1))]).flatten().tolist()
    message.d = intrinsics.get("distortion", [])
    message.distortion_model = "plumb_bob" if message.d else ""
    return message


def _load_labels(path: Path) -> List[Dict[str, Any]]:
    """Read the 3D cuboid annotations of a label file, in whichever of the two formats it uses.

    Labels associated with a lidar sensor are always released as real 3D cuboids, in either the
    OpenLABEL format or the native pre-OpenLABEL format of the ``R00`` lidar subsets (identified
    by their distinctive top-level ``point_cloud_file_name`` key, which OpenLABEL only nests
    under a frame's ``frame_properties``). The legacy 2D-projected format of the earliest
    camera-only releases only ever appears under camera-labeled directories, which are never
    looked up here (see the module docstring for why).
    """
    with path.open() as file:
        data = json.load(file)
    if "openlabel" in data:
        return _parse_openlabel_labels(data)
    if "point_cloud_file_name" in data:
        return _parse_native_lidar_labels(data)
    _print_once(f"TUM Traffic labels '{path}' are in an unrecognized format; no object list is published for them")
    return []


def _parse_openlabel_labels(data: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Parse the OpenLABEL annotations released with R02 and newer."""
    labels = []
    for frame in data["openlabel"].get("frames", {}).values():
        for track_id, entry in frame.get("objects", {}).items():
            object_data = entry.get("object_data", {})
            cuboid = object_data.get("cuboid")
            if not cuboid or not cuboid.get("val"):
                continue
            attributes = cuboid.get("attributes", {})
            labels.append(
                {
                    "track_id": track_id,
                    "category": object_data.get("type", "OTHER"),
                    "cuboid": [float(value) for value in cuboid["val"]],
                    "attributes": {
                        str(attribute["name"]): attribute["val"]
                        for group in ("text", "num", "boolean")
                        for attribute in attributes.get(group, [])
                        if "name" in attribute
                    },
                }
            )
    return labels


def _parse_native_lidar_labels(data: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Parse the native pre-OpenLABEL 3D cuboids of the ``R00`` lidar subsets.

    Every label is a plain ``box3d`` (``location``, ``dimension``, ``orientation``) directly in
    the lidar frame; only ``rotationYaw`` is used for the orientation, matching the dev kit's own
    loader, because roll and pitch are not populated meaningfully in this format. IDs are unique
    per recording but not stable across frames, so no tracking information survives this format.

    Unlike the OpenLABEL cuboid, ``location.z`` here is the box's bottom (ground contact) rather
    than its geometric center: every object in a scene sits at the same ``location.z`` (the flat
    road surface) regardless of its height, and it matches the lower bound of the corresponding
    lidar points rather than their middle. ``HEXAMOTION.Z`` is defined as the geometric center, so
    half the height is added here to convert between the two.
    """
    labels = []
    for entry in data.get("labels", []):
        box = entry.get("box3d")
        if not box:
            continue
        location = box.get("location", {})
        dimension = box.get("dimension", {})
        height = float(dimension.get("height", 0.0))
        yaw = float(box.get("orientation", {}).get("rotationYaw", 0.0))
        quaternion = Rotation.from_euler("z", yaw).as_quat()
        labels.append(
            {
                "track_id": entry.get("id", ""),
                "category": entry.get("category", "OTHER"),
                "cuboid": [
                    float(location.get("x", 0.0)),
                    float(location.get("y", 0.0)),
                    float(location.get("z", 0.0)) + height / 2.0,
                    float(quaternion[0]),
                    float(quaternion[1]),
                    float(quaternion[2]),
                    float(quaternion[3]),
                    float(dimension.get("length", 0.0)),
                    float(dimension.get("width", 0.0)),
                    height,
                ],
                "attributes": dict(entry.get("attributes", {})),
            }
        )
    return labels


def _object_list_3d(
    labels: List[Dict[str, Any]],
    frame_id: str,
    stamp: Time,
    scene_id: str,
    track_ids: Dict[str, int],
) -> ObjectList:
    """Convert OpenLABEL cuboids to a ROS ObjectList message."""
    message = ObjectList(header=Header(frame_id=frame_id, stamp=stamp))
    for label in labels:
        cuboid = label.get("cuboid")
        if cuboid is None:
            continue
        obj = Object(id=_track_id(label["track_id"], track_ids), existence_probability=1.0)
        pmu.initialize_state(obj.state, HEXAMOTION.MODEL_ID)
        obj.state.continuous_state[HEXAMOTION.X] = cuboid[0]
        obj.state.continuous_state[HEXAMOTION.Y] = cuboid[1]
        obj.state.continuous_state[HEXAMOTION.Z] = cuboid[2]
        # OpenLABEL stores the orientation as an [x, y, z, w] quaternion
        roll, pitch, yaw = Rotation.from_quat(cuboid[3:7]).as_euler("xyz")
        obj.state.continuous_state[HEXAMOTION.ROLL] = float(roll)
        obj.state.continuous_state[HEXAMOTION.PITCH] = float(pitch)
        obj.state.continuous_state[HEXAMOTION.YAW] = float(yaw)
        obj.state.continuous_state[HEXAMOTION.LENGTH] = cuboid[7]
        obj.state.continuous_state[HEXAMOTION.WIDTH] = cuboid[8]
        obj.state.continuous_state[HEXAMOTION.HEIGHT] = cuboid[9]
        obj.state.discrete_state[HEXAMOTION.TURN_INDICATOR] = HEXAMOTION.TURN_INDICATOR_UNKNOWN
        obj.state.discrete_state[HEXAMOTION.BRAKE_LIGHT] = HEXAMOTION.LIGHT_UNKNOWN
        obj.state.discrete_state[HEXAMOTION.REVERSE_LIGHT] = HEXAMOTION.LIGHT_UNKNOWN
        obj.state.classifications = [ObjectClassification(type=_classification(label["category"]), probability=1.0)]
        _add_meta_info(obj, scene_id, label)
        message.objects.append(obj)
    return message


def _classification(category: str) -> int:
    """Map a dataset class name to a ROS ObjectClassification type."""
    return _CLASS_MAPPING.get(str(category).upper(), ObjectClassification.UNKNOWN)


def _track_id(track_id: str, track_ids: Dict[str, int]) -> int:
    """Map the UUID a track is annotated with to a consecutive integer ID."""
    return track_ids.setdefault(str(track_id), len(track_ids))


def _add_meta_info(obj: Object, scene_id: str, label: Dict[str, Any]) -> None:
    """Attach the scene, the native class and the native attributes for evaluation."""
    if not hasattr(obj, "meta_info"):
        _print_once("Warning: Object message does not have 'meta_info' field, skipping annotation metadata")
        return
    obj.meta_info.append(f"scene_id:{scene_id}")
    obj.meta_info.append(f"original_class:{label['category']}")
    obj.meta_info.append(f"track_uuid:{label['track_id']}")
    for name, value in label.get("attributes", {}).items():
        obj.meta_info.append(f"{name}:{value}")


def _print_once(message: str) -> None:
    """Print a message the first time it occurs, to keep the playback log readable."""
    if message not in _PRINTED_MESSAGES:
        _PRINTED_MESSAGES.add(message)
        print(message, flush=True)
