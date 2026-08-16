# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

"""Adapter for the MAN TruckScenes dataset.

MAN TruckScenes reuses the nuScenes database schema (see the `schema documentation
<https://github.com/TUMFTM/truckscenes-devkit/blob/main/docs/schema_truckscenes.md>`_) but ships a
truck sensor setup: 4 cameras, 6 lidars and 6 radars mounted on a tractor unit. Sensor data is
stored as ``.pcd`` point clouds instead of the nuScenes ``.bin`` format, and the ego motion is
provided in the dedicated ``ego_motion_cabin`` / ``ego_motion_chassis`` tables.
"""

import os
import re
import shutil
import time
import zipfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path, PurePosixPath
from typing import Any, Dict, Iterator, List, Optional, Tuple
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import cv2
import numpy as np
import perception_msgs_utils as pmu
from autonomy_datasets.datasets.dataset import DatasetAdapter
from autonomy_datasets.datasets.utils import timestamp_micros_to_clock
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from perception_msgs.msg import EGO, EgoData, HEXAMOTION, Object, ObjectClassification, ObjectList, ObjectReferencePoint
from rclpy.logging import get_logger
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from truckscenes import TruckScenes
from truckscenes.utils.data_classes import LidarPointCloud, RadarPointCloud
from truckscenes.utils.geometry_utils import BoxVisibility
from truckscenes.utils.splits import create_splits_scenes

LOGGER = get_logger("autonomy_datasets.truckscenes")

# Mapping from dataset class names to ROS ObjectClassification types. TruckScenes uses the
# nuScenes taxonomy extended by traffic signs, trains, generic vehicles and the ego trailer.
_CLASS_MAPPING: Dict[str, List[int]] = {
    "animal": [ObjectClassification.ANIMAL],
    "human.pedestrian.adult": [ObjectClassification.PEDESTRIAN],
    "human.pedestrian.child": [ObjectClassification.PEDESTRIAN],
    "human.pedestrian.construction_worker": [ObjectClassification.PEDESTRIAN],
    "human.pedestrian.personal_mobility": [ObjectClassification.MICRO],
    "human.pedestrian.police_officer": [ObjectClassification.PEDESTRIAN],
    "human.pedestrian.stroller": [ObjectClassification.VRU],
    "human.pedestrian.wheelchair": [ObjectClassification.VRU],
    "movable_object.barrier": [ObjectClassification.UNKNOWN],
    "movable_object.debris": [ObjectClassification.UNKNOWN],
    "movable_object.pushable_pullable": [ObjectClassification.UNKNOWN],
    "movable_object.trafficcone": [ObjectClassification.UNKNOWN],
    "static_object.bicycle_rack": [ObjectClassification.UNKNOWN],
    "static_object.traffic_sign": [ObjectClassification.UNKNOWN],
    "vehicle.bicycle": [ObjectClassification.BICYCLE],
    "vehicle.bus.bendy": [ObjectClassification.BUS],
    "vehicle.bus.rigid": [ObjectClassification.BUS],
    "vehicle.car": [ObjectClassification.CAR],
    "vehicle.construction": [ObjectClassification.UTILITY],
    "vehicle.ego_trailer": [ObjectClassification.UTILITY],
    "vehicle.emergency.ambulance": [ObjectClassification.UTILITY],
    "vehicle.emergency.police": [ObjectClassification.UTILITY],
    "vehicle.motorcycle": [ObjectClassification.MOTORCYCLE],
    "vehicle.other": [ObjectClassification.UTILITY],
    "vehicle.trailer": [ObjectClassification.UTILITY],
    "vehicle.train": [ObjectClassification.TRAIN],
    "vehicle.truck": [ObjectClassification.UTILITY],
}

# Sensor channels ordered as they are mapped to the canonical camera_XX/lidar_XX/radar_XX topics.
# camera_01 and lidar_01 are the reference sensors the object lists are published in.
_CAMERA_CHANNELS = (
    "CAMERA_LEFT_FRONT",
    "CAMERA_RIGHT_FRONT",
    "CAMERA_RIGHT_BACK",
    "CAMERA_LEFT_BACK",
)
_LIDAR_CHANNELS = (
    "LIDAR_LEFT",
    "LIDAR_RIGHT",
    "LIDAR_TOP_FRONT",
    "LIDAR_TOP_LEFT",
    "LIDAR_TOP_RIGHT",
    "LIDAR_REAR",
)
_RADAR_CHANNELS = (
    "RADAR_LEFT_FRONT",
    "RADAR_RIGHT_FRONT",
    "RADAR_RIGHT_SIDE",
    "RADAR_RIGHT_BACK",
    "RADAR_LEFT_BACK",
    "RADAR_LEFT_SIDE",
)

_SENSOR_CHANNEL_TO_TOPIC: Dict[str, str] = {
    **{channel: f"camera_{index:02d}" for index, channel in enumerate(_CAMERA_CHANNELS, 1)},
    **{channel: f"lidar_{index:02d}" for index, channel in enumerate(_LIDAR_CHANNELS, 1)},
    **{channel: f"radar_{index:02d}" for index, channel in enumerate(_RADAR_CHANNELS, 1)},
}

# TF frame IDs derived from the native channel names, e.g. CAMERA_LEFT_FRONT -> camera_left_front.
_SENSOR_CHANNEL_TO_FRAME_ID: Dict[str, str] = {channel: channel.lower() for channel in _SENSOR_CHANNEL_TO_TOPIC}

# Reference sensors: annotations are published in the frames of these channels.
_REFERENCE_LIDAR_CHANNEL = _LIDAR_CHANNELS[0]
_REFERENCE_CAMERA_CHANNEL = _CAMERA_CHANNELS[0]

# Public AWS Open Data release, downloadable without credentials.
_AWS_ROOT = "https://man-truckscenes.s3.eu-central-1.amazonaws.com/release"
_RELEASE_VERSION = "v1.2"
_RELEASE_ARCHIVES: Dict[str, Tuple[str, ...]] = {
    "mini": (
        f"man-truckscenes_metadata_{_RELEASE_VERSION}-mini.zip",
        f"man-truckscenes_sensordata_{_RELEASE_VERSION}-mini.zip",
    ),
    "trainval": (
        f"man-truckscenes_metadata_{_RELEASE_VERSION}-trainval.zip",
        *(f"man-truckscenes_sensordata{index:02d}_{_RELEASE_VERSION}-trainval.zip" for index in range(1, 8)),
    ),
    "test": (
        f"man-truckscenes_metadata_{_RELEASE_VERSION}-test.zip",
        *(f"man-truckscenes_sensordata{index:02d}_{_RELEASE_VERSION}-test.zip" for index in range(1, 3)),
    ),
}

# Scene splits mapped to the release archive that contains them.
_SPLIT_TO_RELEASE = {
    "mini_train": "mini",
    "mini_val": "mini",
    "train": "trainval",
    "train_detect": "trainval",
    "train_track": "trainval",
    "val": "trainval",
    "test": "test",
}

_VERSION_PATTERN = re.compile(r"^v(\d+)\.(\d+)-(mini|trainval|test)$")
_DOWNLOAD_DIR_NAME = ".truckscenes_download"
_DOWNLOAD_ATTEMPTS = 5
_HTTP_TIMEOUT_SECONDS = 60

# Official dimensions of the MAN TGX 18.510 tractor unit (www.man.eu/truckscenes; combined
# length with trailer is 16.0m, unused here). The dataset does not ship ego dimensions itself,
# and only some recordings tow a trailer, so EgoData models the rigid tractor only; when present,
# the articulated trailer is published as its own "vehicle.ego_trailer" annotation with its own
# pose rather than folded into a fixed combined ego length.
_EGO_LENGTH = 6.0
_EGO_WIDTH = 3.0
_EGO_HEIGHT = 4.0

# Rear overhang (rear axle to back of cab), estimated from calibrated_sensor extrinsics: LIDAR_REAR
# sits ~0.86m behind the vehicle frame origin (near the back of the cab), while the front corner
# sensors sit ~5.2m ahead of it - a ~6.1m span that lines up with _EGO_LENGTH. No official rear
# overhang figure is published for the TGX, so this is rounded slightly up from the raw sensor
# extent to account for sensor housings sitting a bit inboard of the actual body edges.
_EGO_REAR_OVERHANG = 0.9

_MISSING_META_INFO_WARNING_PRINTED = False


class TruckScenesAdapter(DatasetAdapter):
    """Converts MAN TruckScenes dataset files to ROS 2 messages."""

    VERSION = "1.0.0"
    RELEASE_NOTES = {
        "1.0.0": "Initial integration into autonomy_datasets",
    }

    def __init__(
        self,
        data_publishers: Dict[str, Any],
        split: str,
        dataset_root_dir: str,
        publish_ego_data: bool = True,
        publish_camera_images: bool = True,
        publish_lidar_pointclouds: bool = True,
        publish_radar_pointclouds: bool = True,
        publish_lidar_object_lists: bool = True,
        publish_camera_01_object_lists: bool = True,
        min_lidar_points_in_bbox: int = 1,
        camera_box_visibility: BoxVisibility = BoxVisibility.ANY,
        camera_box_min_points: int = 1,
        auto_download: bool = True,
        download_workers: int = 8,
        start_scene_index: int = 0,
    ) -> None:
        """Initialize the TruckScenes dataset adapter.

        Args:
            data_publishers: Mapping of topic names to publisher instances.
            split: Dataset split name (mini_train, mini_val, train, val, test, ...).
            dataset_root_dir: Root directory of the extracted TruckScenes dataset.
            publish_ego_data: Whether to publish ego data.
            publish_camera_images: Whether to publish camera image data.
            publish_lidar_pointclouds: Whether to publish lidar point cloud data.
            publish_radar_pointclouds: Whether to publish radar point cloud data.
            publish_lidar_object_lists: Whether to publish lidar object lists.
            publish_camera_01_object_lists: Whether to publish camera_01 (left front) object lists.
            min_lidar_points_in_bbox: Minimum lidar points required for lidar object labels.
            camera_box_visibility: Required camera box visibility filter for annotations.
            camera_box_min_points: Minimum lidar+radar points required for camera object labels.
            auto_download: Whether to download the release when it is not available locally.
            download_workers: Number of release archives to download concurrently.
            start_scene_index: Number of scenes to skip before generating samples.

        Raises:
            ValueError: If the split is unknown or download_workers is invalid.
            FileNotFoundError: If the dataset is missing and auto_download is disabled.
        """

        super().__init__(data_publishers=data_publishers)
        if split not in _SPLIT_TO_RELEASE:
            raise ValueError(f"Unsupported TruckScenes split '{split}'; expected one of: {', '.join(sorted(_SPLIT_TO_RELEASE))}")
        if download_workers < 1:
            raise ValueError("TruckScenes download_workers must be at least 1")

        self.split = split
        self.publish_ego_data = publish_ego_data
        self.publish_camera_images = publish_camera_images
        self.publish_lidar_pointclouds = publish_lidar_pointclouds
        self.publish_radar_pointclouds = publish_radar_pointclouds
        self.publish_lidar_object_lists = publish_lidar_object_lists
        self.publish_camera_01_object_lists = publish_camera_01_object_lists
        self.start_scene_index = start_scene_index

        # Root directory of the extracted TruckScenes dataset
        self.dataset_root_dir = dataset_root_dir

        # Minimum number of lidar points in bounding box to be considered in
        # "lidar_objects" datasets
        self.min_lidar_points_in_bbox = min_lidar_points_in_bbox

        # Required visibility of bounding box to be considered in "camera_objects"
        # datasets. Options: ALL (all corners inside the image), ANY (at least one
        # corner), NONE (no corners). This does not consider occlusions, use
        # CAMERA_BOX_MIN_POINTS for that.
        self.camera_box_visibility = camera_box_visibility

        # Minimum number of lidar or radar points in bounding box to be considered in
        # "camera_objects" datasets
        self.camera_box_min_points = camera_box_min_points

        release = _SPLIT_TO_RELEASE[split]
        dataset_version = _resolve_version(self.dataset_root_dir, release)
        if dataset_version is None:
            if not auto_download:
                raise FileNotFoundError(
                    f"TruckScenes '{release}' metadata not found in '{self.dataset_root_dir}'; "
                    f"download it manually or enable 'truckscenes_auto_download'"
                )
            _download_release(Path(self.dataset_root_dir), release, download_workers)
            dataset_version = _resolve_version(self.dataset_root_dir, release)
            if dataset_version is None:
                raise FileNotFoundError(
                    f"TruckScenes download did not create the expected metadata directory in " f"'{self.dataset_root_dir}'"
                )

        self.trucksc = TruckScenes(version=dataset_version, dataroot=str(self.dataset_root_dir), verbose=True)

        # add publishers for outgoing messages, actual publisher will be created in AutonomyDatasets node
        if self.publish_ego_data:
            self.data_publishers["ego_data"] = None
        if self.publish_lidar_object_lists:
            self.data_publishers["object_list/lidar_01"] = None
        if self.publish_camera_01_object_lists:
            self.data_publishers["object_list/camera_01"] = None
        for channel, topic in _SENSOR_CHANNEL_TO_TOPIC.items():
            if self.publish_camera_images and channel in _CAMERA_CHANNELS:
                self.data_publishers[f"{topic}/image_raw"] = None
                self.data_publishers[f"{topic}/camera_info"] = None
            if self.publish_lidar_pointclouds and channel in _LIDAR_CHANNELS:
                self.data_publishers[f"{topic}/point_cloud"] = None
            if self.publish_radar_pointclouds and channel in _RADAR_CHANNELS:
                self.data_publishers[f"{topic}/point_cloud"] = None

    def generate_samples(self) -> Iterator[Tuple[int, Dict[str, Any]]]:
        """Yield sequential sample indices and ROS-ready sample payloads for the configured split."""
        scene_splits = create_splits_scenes()
        count_examples = 0
        skipped_scene_count = 0
        for scene in self.trucksc.scene:
            if scene["name"] not in scene_splits[self.split]:
                continue
            if skipped_scene_count < self.start_scene_index:
                skipped_scene_count += 1
                LOGGER.info(f"Skipping already stored scene {skipped_scene_count}: {scene['token']}")
                continue

            scene_id = scene["token"]
            instance_id_map: Dict[str, int] = {}
            sample_token = scene["first_sample_token"]
            while sample_token != "":
                trucksc_sample = self.trucksc.get("sample", sample_token)
                sample: Dict[str, Any] = {}
                clock_msg = timestamp_micros_to_clock(int(trucksc_sample["timestamp"]))

                # The devkit transforms annotations using the ego pose closest to the sample
                # timestamp, so the published ego pose has to be looked up the same way.
                ego_pose = self.trucksc.getclosest("ego_pose", trucksc_sample["timestamp"])
                ego_motion = self.trucksc.getclosest("ego_motion_chassis", trucksc_sample["timestamp"])
                ego_data_msg, tf_msg = _egomotion_to_ego_data(ego_pose, ego_motion, clock_msg.clock)

                if self.publish_ego_data:
                    sample["ego_data"] = ego_data_msg

                if self.publish_lidar_pointclouds:
                    for channel in _LIDAR_CHANNELS:
                        topic = _SENSOR_CHANNEL_TO_TOPIC[channel]
                        pcl_path = self.trucksc.get_sample_data_path(trucksc_sample["data"][channel])
                        sample[f"{topic}/point_cloud"] = _get_lidar_point_cloud(
                            pcl_path, clock_msg.clock, _SENSOR_CHANNEL_TO_FRAME_ID[channel]
                        )

                if self.publish_radar_pointclouds:
                    for channel in _RADAR_CHANNELS:
                        topic = _SENSOR_CHANNEL_TO_TOPIC[channel]
                        pcl_path = self.trucksc.get_sample_data_path(trucksc_sample["data"][channel])
                        sample[f"{topic}/point_cloud"] = _get_radar_point_cloud(
                            pcl_path, clock_msg.clock, _SENSOR_CHANNEL_TO_FRAME_ID[channel]
                        )

                if self.publish_lidar_object_lists:
                    sample_data_lidar_token = trucksc_sample["data"][_REFERENCE_LIDAR_CHANNEL]
                    _, annotations, _ = self.trucksc.get_sample_data(sample_data_lidar_token)

                    # Object list with meta information for evaluation
                    object_list = []
                    for ann in annotations:
                        sample_annotation = self.trucksc.get("sample_annotation", ann.token)
                        num_lidar_pts = sample_annotation["num_lidar_pts"]
                        num_radar_pts = sample_annotation["num_radar_pts"]
                        if num_lidar_pts < self.min_lidar_points_in_bbox:
                            continue
                        instance_token = sample_annotation["instance_token"]
                        if instance_token not in instance_id_map:
                            instance_id_map[instance_token] = len(instance_id_map)
                        attributes = [
                            self.trucksc.get("attribute", attribute_token)["name"]
                            for attribute_token in sample_annotation["attribute_tokens"]
                        ]
                        object_list.append((ann, num_lidar_pts, num_radar_pts, attributes, instance_id_map[instance_token]))
                    sample["object_list/lidar_01"] = _labels_to_object_list(
                        object_list,
                        _SENSOR_CHANNEL_TO_FRAME_ID[_REFERENCE_LIDAR_CHANNEL],
                        clock_msg.clock,
                        scene_id,
                    )

                if self.publish_camera_images:
                    for channel in _CAMERA_CHANNELS:
                        topic = _SENSOR_CHANNEL_TO_TOPIC[channel]
                        sample_data_token = trucksc_sample["data"][channel]
                        sample_data = self.trucksc.get("sample_data", sample_data_token)
                        calibrated_sensor = self.trucksc.get("calibrated_sensor", sample_data["calibrated_sensor_token"])
                        camera_intrinsic = np.asarray(calibrated_sensor["camera_intrinsic"], dtype=np.float64)
                        image_path = self.trucksc.get_sample_data_path(sample_data_token)
                        camera_frame_id = _SENSOR_CHANNEL_TO_FRAME_ID[channel]

                        sample[f"{topic}/image_raw"] = _image_path_to_ros_msg(image_path, clock_msg.clock, camera_frame_id)
                        sample[f"{topic}/camera_info"] = _camera_intrinsic_to_camera_info_msg(
                            camera_intrinsic,
                            sample_data["width"],
                            sample_data["height"],
                            clock_msg.clock,
                            camera_frame_id,
                        )

                if self.publish_camera_01_object_lists:
                    sample_data_camera_token = trucksc_sample["data"][_REFERENCE_CAMERA_CHANNEL]
                    _, annotations, _ = self.trucksc.get_sample_data(
                        sample_data_camera_token, box_vis_level=self.camera_box_visibility
                    )
                    camera_frame_id = _SENSOR_CHANNEL_TO_FRAME_ID[_REFERENCE_CAMERA_CHANNEL]

                    object_list = []
                    for ann in annotations:
                        object_classification = _CLASS_MAPPING[ann.name]
                        # Ignore annotations with too less lidar or radar points
                        # as they may not be visible in the camera image
                        sample_annotation = self.trucksc.get("sample_annotation", ann.token)
                        instance_token = sample_annotation["instance_token"]
                        if instance_token not in instance_id_map:
                            instance_id_map[instance_token] = len(instance_id_map)
                        num_lidar_pts = sample_annotation["num_lidar_pts"]
                        num_radar_pts = sample_annotation["num_radar_pts"]
                        num_pts = num_lidar_pts + num_radar_pts
                        if num_pts < self.camera_box_min_points:
                            continue

                        ann_x, ann_y, ann_z = ann.center
                        ann_q = ann.orientation
                        ann_w, ann_l, ann_h = ann.wlh

                        # Check if object is in front of camera (z > 0 in camera frame)
                        if ann_z <= 0:
                            continue

                        rot_cam = Rotation.from_quat([ann_q.q[1], ann_q.q[2], ann_q.q[3], ann_q.q[0]])
                        roll_cam, pitch_cam, yaw_cam = rot_cam.as_euler("xyz")

                        object_list.append(
                            (
                                instance_id_map[instance_token],
                                ann.name,
                                object_classification,
                                ann_x,
                                ann_y,
                                ann_z,
                                roll_cam,
                                pitch_cam,
                                yaw_cam,
                                ann_l,
                                ann_w,
                                ann_h,
                                num_pts,
                            )
                        )

                    sample["object_list/camera_01"] = _camera_labels_to_object_list(
                        object_list,
                        camera_frame_id,
                        clock_msg.clock,
                        scene_id,
                    )

                # Build static TF messages from sensor calibration
                tf_msgs = _build_tf_msgs(self.trucksc, trucksc_sample)

                sample["scene_id"] = scene_id
                sample["/clock"] = clock_msg
                sample["/tf"] = tf_msg
                sample["/tf_static"] = TFMessage(transforms=tf_msgs)

                sample_token = trucksc_sample["next"]
                count_examples += 1
                yield count_examples, sample


def _resolve_version(dataset_root_dir: str, release: str) -> Optional[str]:
    """Return the newest locally available metadata directory of a release, if any.

    TruckScenes stores its tables in a ``v<major>.<minor>-<release>`` directory. The release
    version is not pinned by this adapter so that already downloaded datasets keep working.

    Args:
        dataset_root_dir: Root directory of the extracted TruckScenes dataset.
        release: Release name (mini, trainval or test).

    Returns:
        Name of the metadata directory, or None when the release is not available locally.
    """
    if not os.path.isdir(dataset_root_dir):
        return None
    versions = []
    for entry in os.listdir(dataset_root_dir):
        match = _VERSION_PATTERN.match(entry)
        if match is None or match.group(3) != release:
            continue
        if not os.path.isdir(os.path.join(dataset_root_dir, entry)):
            continue
        versions.append((int(match.group(1)), int(match.group(2)), entry))
    if not versions:
        return None
    return max(versions)[2]


def _download_release(dataset_root: Path, release: str, download_workers: int) -> None:
    """Download and extract all archives of a TruckScenes release.

    The archives are hosted on the public AWS Open Data registry and require no credentials.
    Only keyframe sensor data (``samples/``) and the metadata tables are extracted; the
    unannotated ``sweeps/`` are skipped because this adapter publishes keyframes only.

    Args:
        dataset_root: Root directory the release is extracted into.
        release: Release name (mini, trainval or test).
        download_workers: Number of archives to download concurrently.
    """
    archives = _RELEASE_ARCHIVES[release]
    download_dir = dataset_root / _DOWNLOAD_DIR_NAME
    download_dir.mkdir(parents=True, exist_ok=True)
    LOGGER.info(
        f"TruckScenes '{release}' was not found in '{dataset_root}'; "
        f"downloading {len(archives)} archive(s) from the AWS Open Data registry."
    )
    workers = min(download_workers, len(archives))
    with ThreadPoolExecutor(max_workers=workers, thread_name_prefix="truckscenes-archive") as executor:
        list(
            executor.map(
                lambda item: _download_and_extract_archive(release, item[0], item[1], len(archives), dataset_root),
                enumerate(archives, start=1),
            )
        )
    shutil.rmtree(download_dir, ignore_errors=True)
    LOGGER.info(f"TruckScenes '{release}' is ready in '{dataset_root}'.")


def _download_and_extract_archive(
    release: str,
    archive_number: int,
    archive_name: str,
    archive_total: int,
    dataset_root: Path,
) -> None:
    """Download a single release archive and extract it into the dataset root."""
    marker = dataset_root / _DOWNLOAD_DIR_NAME / ".extracted" / archive_name
    if marker.is_file():
        LOGGER.info(f"TruckScenes archive {archive_number}/{archive_total} already extracted: {archive_name}")
        return
    archive_path = dataset_root / _DOWNLOAD_DIR_NAME / archive_name
    _download_archive(
        f"{_AWS_ROOT}/{release}/{archive_name}",
        archive_path,
        f"{archive_number}/{archive_total}",
    )
    _extract_archive(archive_path, dataset_root, f"{archive_number}/{archive_total}")
    archive_path.unlink(missing_ok=True)
    marker.parent.mkdir(parents=True, exist_ok=True)
    marker.touch()


def _download_archive(url: str, destination: Path, archive_position: str) -> None:
    """Download an archive with resume support, retrying transient failures."""
    for attempt in range(1, _DOWNLOAD_ATTEMPTS + 1):
        try:
            _download_archive_once(url, destination, archive_position)
            return
        except (HTTPError, URLError, TimeoutError, OSError) as error:
            if attempt == _DOWNLOAD_ATTEMPTS:
                raise
            delay = min(2 ** (attempt - 1), 30)
            LOGGER.warn(
                f"TruckScenes archive {archive_position} failed: {error}. "
                f"Retrying in {delay} seconds ({attempt}/{_DOWNLOAD_ATTEMPTS})."
            )
            time.sleep(delay)


def _download_archive_once(url: str, destination: Path, archive_position: str) -> None:
    """Download an archive, resuming a previously interrupted transfer when possible."""
    if destination.is_file():
        LOGGER.info(f"Using existing TruckScenes archive {archive_position}: {destination.name}")
        return
    destination.parent.mkdir(parents=True, exist_ok=True)
    partial = destination.with_suffix(destination.suffix + ".part")
    offset = partial.stat().st_size if partial.exists() else 0
    request = Request(url, headers={"User-Agent": "autonomy_datasets"})
    if offset:
        request.add_header("Range", f"bytes={offset}-")
    LOGGER.info(f"Downloading TruckScenes archive {archive_position}: {destination.name}")
    with urlopen(request, timeout=_HTTP_TIMEOUT_SECONDS) as response:
        resuming = bool(offset) and response.status == 206
        mode = "ab" if resuming else "wb"
        remaining = int(response.headers.get("Content-Length", 0))
        downloaded = offset if resuming else 0
        total = downloaded + remaining
        report_step = max(total // 20, 64 * 1024 * 1024) if total else 512 * 1024 * 1024
        next_report = downloaded + report_step
        with partial.open(mode) as output:
            while chunk := response.read(1024 * 1024):
                output.write(chunk)
                downloaded += len(chunk)
                if downloaded >= next_report:
                    _log_progress(f"  Archive {archive_position} ({destination.name})", downloaded, total)
                    next_report += report_step
        _log_progress(f"  Archive {archive_position} ({destination.name})", downloaded, total)
    os.replace(partial, destination)


def _extract_archive(archive_path: Path, dataset_root: Path, archive_position: str) -> None:
    """Extract keyframe sensor data and metadata tables from a release archive.

    Every entry in a release archive is nested under a shared top-level directory (e.g.
    ``man-truckscenes/v1.2-mini/scene.json``, ``man-truckscenes/samples/LIDAR_LEFT/...``); that
    wrapper is stripped so the extracted layout matches what :class:`TruckScenes` expects
    directly under ``dataset_root`` (``dataset_root/v1.2-mini/*.json``, ``dataset_root/samples/...``).
    """
    LOGGER.info(f"Extracting TruckScenes archive {archive_position}: {archive_path.name}")
    with zipfile.ZipFile(archive_path) as archive:
        for member in archive.infolist():
            if member.is_dir():
                continue
            relative_parts = PurePosixPath(member.filename).parts[1:]
            # Unannotated intermediate frames are never published by this adapter and would
            # multiply the required disk space.
            if not relative_parts or relative_parts[0] == "sweeps":
                continue
            target = (dataset_root / Path(*relative_parts)).resolve()
            if dataset_root.resolve() not in target.parents:
                raise ValueError(f"Unsafe path in TruckScenes archive: {member.filename}")
            target.parent.mkdir(parents=True, exist_ok=True)
            with archive.open(member) as source, target.open("wb") as destination:
                shutil.copyfileobj(source, destination)


def _log_progress(label: str, completed: int, total: int) -> None:
    """Log a stable, single-line byte progress update."""
    completed_gib = completed / 1024**3
    if total:
        LOGGER.info(f"{label}: {completed_gib:.2f} / {total / 1024**3:.2f} GiB ({completed / total:.0%})")
    else:
        LOGGER.info(f"{label}: {completed_gib:.2f} GiB")


def _build_tf_msgs(trucksc: TruckScenes, trucksc_sample: Dict[str, Any]) -> List[TransformStamped]:
    """Build static TF messages from TruckScenes sensor calibration.

    Retrieves the calibrated sensor extrinsics (translation + rotation) for each sensor channel
    in the sample and creates TransformStamped messages from base_link to the respective sensor
    frame.

    Args:
        trucksc: TruckScenes database instance.
        trucksc_sample: A TruckScenes sample record dict.

    Returns:
        List of TransformStamped messages.
    """
    tf_msgs = []
    for sensor_channel, child_frame_id in _SENSOR_CHANNEL_TO_FRAME_ID.items():
        if sensor_channel not in trucksc_sample["data"]:
            continue
        sample_data = trucksc.get("sample_data", trucksc_sample["data"][sensor_channel])
        calibrated_sensor = trucksc.get("calibrated_sensor", sample_data["calibrated_sensor_token"])
        translation = calibrated_sensor["translation"]
        # TruckScenes quaternion is [w, x, y, z]
        qw, qx, qy, qz = calibrated_sensor["rotation"]
        tf_msgs.append(
            TransformStamped(
                header=Header(frame_id="base_link"),
                child_frame_id=child_frame_id,
                transform=Transform(
                    translation=Vector3(
                        x=float(translation[0]),
                        y=float(translation[1]),
                        z=float(translation[2]),
                    ),
                    rotation=Quaternion(
                        x=float(qx),
                        y=float(qy),
                        z=float(qz),
                        w=float(qw),
                    ),
                ),
            )
        )
    return tf_msgs


def _labels_to_object_list(labels: List[Any], frame_id: str, stamp_msg: Time, scene_id: str) -> ObjectList:
    """Convert labels to a ROS ObjectList message."""
    object_list_msg = ObjectList()
    object_list_msg.header.frame_id = frame_id
    object_list_msg.header.stamp = stamp_msg
    objects: List[Object] = []

    for label, num_lidar_pts, num_radar_pts, attributes, instance_id in labels:
        obj_msg = Object()
        obj_msg.id = instance_id
        obj_msg.existence_probability = 1.0

        pmu.initialize_state(obj_msg.state, HEXAMOTION.MODEL_ID)

        # Position
        obj_msg.state.continuous_state[HEXAMOTION.X] = float(label.center[0])
        obj_msg.state.continuous_state[HEXAMOTION.Y] = float(label.center[1])
        obj_msg.state.continuous_state[HEXAMOTION.Z] = float(label.center[2])

        # Orientation: extract roll, pitch, yaw from quaternion
        rot = Rotation.from_quat([label.orientation.q[1], label.orientation.q[2], label.orientation.q[3], label.orientation.q[0]])
        roll, pitch, yaw = rot.as_euler("xyz")
        obj_msg.state.continuous_state[HEXAMOTION.ROLL] = float(roll)
        obj_msg.state.continuous_state[HEXAMOTION.PITCH] = float(pitch)
        obj_msg.state.continuous_state[HEXAMOTION.YAW] = float(yaw)

        # Dimensions
        obj_msg.state.continuous_state[HEXAMOTION.WIDTH] = float(label.wlh[0])
        obj_msg.state.continuous_state[HEXAMOTION.LENGTH] = float(label.wlh[1])
        obj_msg.state.continuous_state[HEXAMOTION.HEIGHT] = float(label.wlh[2])

        # Discrete state
        obj_msg.state.discrete_state[HEXAMOTION.TURN_INDICATOR] = HEXAMOTION.TURN_INDICATOR_UNKNOWN
        obj_msg.state.discrete_state[HEXAMOTION.BRAKE_LIGHT] = HEXAMOTION.LIGHT_UNKNOWN
        obj_msg.state.discrete_state[HEXAMOTION.REVERSE_LIGHT] = HEXAMOTION.LIGHT_UNKNOWN

        # Classification
        class_types = _CLASS_MAPPING[label.name]
        obj_msg.state.classifications = [ObjectClassification(type=ct, probability=1.0) for ct in class_types]

        # Meta information for evaluation
        if hasattr(obj_msg, "meta_info"):
            obj_msg.meta_info.append(f"scene_id:{scene_id}")
            obj_msg.meta_info.append(f"original_class:{label.name}")
            obj_msg.meta_info.append(f"num_lidar_pts:{num_lidar_pts}")
            obj_msg.meta_info.append(f"num_radar_pts:{num_radar_pts}")
            for attr in attributes:
                obj_msg.meta_info.append(f"attribute:{attr}")
        else:
            _warn_missing_meta_info_once()

        objects.append(obj_msg)

    object_list_msg.objects = objects
    return object_list_msg


def _camera_labels_to_object_list(labels: List[Any], frame_id: str, stamp_msg: Time, scene_id: str) -> ObjectList:
    """Convert camera annotations to a ROS ObjectList message."""
    object_list_msg = ObjectList()
    object_list_msg.header.frame_id = frame_id
    object_list_msg.header.stamp = stamp_msg
    objects: List[Object] = []

    for label in labels:
        obj_msg = Object()
        obj_msg.existence_probability = 1.0

        (
            instance_id,
            original_class,
            class_types,
            x_cam,
            y_cam,
            z_cam,
            roll_cam,
            pitch_cam,
            yaw_cam,
            length,
            width,
            height,
            num_pts,
        ) = label
        obj_msg.id = instance_id
        pmu.initialize_state(obj_msg.state, HEXAMOTION.MODEL_ID)
        obj_msg.state.continuous_state[HEXAMOTION.X] = float(x_cam)
        obj_msg.state.continuous_state[HEXAMOTION.Y] = float(y_cam)
        obj_msg.state.continuous_state[HEXAMOTION.Z] = float(z_cam)
        obj_msg.state.continuous_state[HEXAMOTION.ROLL] = float(roll_cam)
        obj_msg.state.continuous_state[HEXAMOTION.PITCH] = float(pitch_cam)
        obj_msg.state.continuous_state[HEXAMOTION.YAW] = float(yaw_cam)
        obj_msg.state.continuous_state[HEXAMOTION.LENGTH] = float(length)
        obj_msg.state.continuous_state[HEXAMOTION.WIDTH] = float(width)
        obj_msg.state.continuous_state[HEXAMOTION.HEIGHT] = float(height)
        obj_msg.state.discrete_state[HEXAMOTION.TURN_INDICATOR] = HEXAMOTION.TURN_INDICATOR_UNKNOWN
        obj_msg.state.discrete_state[HEXAMOTION.BRAKE_LIGHT] = HEXAMOTION.LIGHT_UNKNOWN
        obj_msg.state.discrete_state[HEXAMOTION.REVERSE_LIGHT] = HEXAMOTION.LIGHT_UNKNOWN

        obj_msg.state.classifications = [ObjectClassification(type=class_type, probability=1.0) for class_type in class_types]
        if hasattr(obj_msg, "meta_info"):
            obj_msg.meta_info.append(f"scene_id:{scene_id}")
            obj_msg.meta_info.append(f"original_class:{original_class}")
            obj_msg.meta_info.append(f"num_points:{num_pts}")
        else:
            _warn_missing_meta_info_once()
        objects.append(obj_msg)

    object_list_msg.objects = objects
    return object_list_msg


def _warn_missing_meta_info_once() -> None:
    global _MISSING_META_INFO_WARNING_PRINTED

    if not _MISSING_META_INFO_WARNING_PRINTED:
        LOGGER.warn("Object message does not have 'meta_info' field, skipping annotation metadata")
        _MISSING_META_INFO_WARNING_PRINTED = True


def _egomotion_to_ego_data(
    ego_pose: Dict[str, Any],
    ego_motion: Optional[Dict[str, Any]],
    stamp_msg: Time,
) -> Tuple[EgoData, TFMessage]:
    """Convert TruckScenes ego records to a ROS EgoData message and TF.

    Args:
        ego_pose: TruckScenes ego_pose record with 'translation' [x, y, z] in UTM-WGS84 (zone
            U32) and 'rotation' [w, x, y, z] quaternion.
        ego_motion: TruckScenes ego_motion_chassis record with velocities, accelerations and
            rates in the vehicle frame, or None when unavailable.
        stamp_msg: ROS Time message.

    Returns:
        Tuple of (EgoData message, TFMessage with map->base_link transform).
    """
    tx, ty, tz = ego_pose["translation"]
    qw, qx, qy, qz = ego_pose["rotation"]

    ego_data_msg = EgoData()
    ego_data_msg.header.frame_id = "map"
    ego_data_msg.header.stamp = stamp_msg
    pmu.initialize_state(ego_data_msg.state, EGO.MODEL_ID)

    # Reference Point - TruckScenes ego_pose is the center of the rear axle projected onto the
    # ground (ISO 8855), not the tractor's geometric center.
    # x: length/2 - rear_overhang = 3.0 - 0.9 = 2.1m forward to geometric center
    # z: height/2 = 2.0m up to geometric center
    ego_data_msg.state.reference_point = ObjectReferencePoint(
        value=ObjectReferencePoint.REAR_AXLE_GROUND,
        translation_to_geometric_center=Vector3(x=_EGO_LENGTH / 2.0 - _EGO_REAR_OVERHANG, y=0.0, z=_EGO_HEIGHT / 2.0),
    )

    # Position
    ego_data_msg.state.continuous_state[EGO.X] = float(tx)
    ego_data_msg.state.continuous_state[EGO.Y] = float(ty)
    ego_data_msg.state.continuous_state[EGO.Z] = float(tz)

    # Orientation: extract roll, pitch, yaw from quaternion
    rot = Rotation.from_quat([qx, qy, qz, qw])
    roll, pitch, yaw = rot.as_euler("xyz")
    ego_data_msg.state.continuous_state[EGO.ROLL] = float(roll)
    ego_data_msg.state.continuous_state[EGO.PITCH] = float(pitch)
    ego_data_msg.state.continuous_state[EGO.YAW] = float(yaw)

    # Dynamics from the chassis motion table (given in the vehicle coordinate system)
    if ego_motion is not None:
        ego_data_msg.state.continuous_state[EGO.VEL_LON] = float(ego_motion["vx"])
        ego_data_msg.state.continuous_state[EGO.VEL_LAT] = float(ego_motion["vy"])
        ego_data_msg.state.continuous_state[EGO.ACC_LON] = float(ego_motion["ax"])
        ego_data_msg.state.continuous_state[EGO.ACC_LAT] = float(ego_motion["ay"])
        ego_data_msg.state.continuous_state[EGO.YAW_RATE] = float(ego_motion["yaw_rate"])

    # Dimensions of the tractor unit; the towed semi-trailer is annotated as "vehicle.ego_trailer"
    ego_data_msg.length = _EGO_LENGTH
    ego_data_msg.width = _EGO_WIDTH
    ego_data_msg.height = _EGO_HEIGHT

    # Create TFMessage for ego pose in map frame
    tf_msg = TFMessage(
        transforms=[
            TransformStamped(
                header=Header(frame_id="map", stamp=stamp_msg),
                child_frame_id="base_link",
                transform=Transform(
                    translation=Vector3(
                        x=float(tx),
                        y=float(ty),
                        z=float(tz),
                    ),
                    rotation=Quaternion(
                        x=float(qx),
                        y=float(qy),
                        z=float(qz),
                        w=float(qw),
                    ),
                ),
            )
        ]
    )

    return ego_data_msg, tf_msg


def _get_lidar_point_cloud(pcl_path: str, stamp_msg: Time, frame_id: str) -> PointCloud2:
    """Load a TruckScenes lidar .pcd file and convert it to a ROS PointCloud2 message."""
    point_cloud = LidarPointCloud.from_file(pcl_path)
    x, y, z, intensity = point_cloud.points

    points = np.empty(
        point_cloud.points.shape[1],
        dtype=[
            ("x", "<f4"),
            ("y", "<f4"),
            ("z", "<f4"),
            ("intensity", "<f4"),
            ("timestamp", "<f8"),
        ],
    )
    points["x"] = x
    points["y"] = y
    points["z"] = z
    points["intensity"] = intensity
    # Native per-point timestamps are absolute microseconds; publish them as absolute seconds.
    points["timestamp"] = np.ravel(point_cloud.timestamps) * 1e-6

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="timestamp", offset=16, datatype=PointField.FLOAT64, count=1),
    ]

    return create_cloud(Header(frame_id=frame_id, stamp=stamp_msg), fields, points)


def _get_radar_point_cloud(pcl_path: str, stamp_msg: Time, frame_id: str) -> PointCloud2:
    """Load a TruckScenes radar .pcd file and convert it to a ROS PointCloud2 message."""
    point_cloud = RadarPointCloud.from_file(pcl_path)
    x, y, z, vrel_x, vrel_y, vrel_z, rcs = point_cloud.points

    points = np.empty(
        point_cloud.points.shape[1],
        dtype=[
            ("x", "<f4"),
            ("y", "<f4"),
            ("z", "<f4"),
            ("vrel_x", "<f4"),
            ("vrel_y", "<f4"),
            ("vrel_z", "<f4"),
            ("rcs", "<f4"),
        ],
    )
    points["x"] = x
    points["y"] = y
    points["z"] = z
    points["vrel_x"] = vrel_x
    points["vrel_y"] = vrel_y
    points["vrel_z"] = vrel_z
    points["rcs"] = rcs

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="vrel_x", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="vrel_y", offset=16, datatype=PointField.FLOAT32, count=1),
        PointField(name="vrel_z", offset=20, datatype=PointField.FLOAT32, count=1),
        PointField(name="rcs", offset=24, datatype=PointField.FLOAT32, count=1),
    ]

    return create_cloud(Header(frame_id=frame_id, stamp=stamp_msg), fields, points)


def _image_path_to_ros_msg(image_path: str, stamp_msg: Time, frame_id: str) -> Image:
    """Load an image file and convert it to a ROS Image message."""
    img_array = cv2.imread(image_path, cv2.IMREAD_COLOR)
    if img_array is None:
        raise ValueError(f"Failed to read image: {image_path}")
    img_rgb = cv2.cvtColor(img_array, cv2.COLOR_BGR2RGB)

    image_msg = Image()
    image_msg.header.frame_id = frame_id
    image_msg.header.stamp = stamp_msg
    image_msg.height = img_rgb.shape[0]
    image_msg.width = img_rgb.shape[1]
    image_msg.encoding = "rgb8"
    image_msg.is_bigendian = False
    image_msg.step = img_rgb.shape[1] * 3
    image_msg.data = img_rgb.tobytes()

    return image_msg


def _camera_intrinsic_to_camera_info_msg(
    camera_intrinsic: np.ndarray,
    width: int,
    height: int,
    stamp_msg: Time,
    frame_id: str,
) -> CameraInfo:
    """Convert a TruckScenes intrinsic matrix to a ROS CameraInfo message."""
    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = frame_id
    camera_info_msg.header.stamp = stamp_msg
    camera_info_msg.width = int(width)
    camera_info_msg.height = int(height)
    camera_info_msg.k = [
        float(camera_intrinsic[0, 0]),
        float(camera_intrinsic[0, 1]),
        float(camera_intrinsic[0, 2]),
        float(camera_intrinsic[1, 0]),
        float(camera_intrinsic[1, 1]),
        float(camera_intrinsic[1, 2]),
        float(camera_intrinsic[2, 0]),
        float(camera_intrinsic[2, 1]),
        float(camera_intrinsic[2, 2]),
    ]
    camera_info_msg.r = [
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
    camera_info_msg.p = [
        float(camera_intrinsic[0, 0]),
        float(camera_intrinsic[0, 1]),
        float(camera_intrinsic[0, 2]),
        0.0,
        float(camera_intrinsic[1, 0]),
        float(camera_intrinsic[1, 1]),
        float(camera_intrinsic[1, 2]),
        0.0,
        float(camera_intrinsic[2, 0]),
        float(camera_intrinsic[2, 1]),
        float(camera_intrinsic[2, 2]),
        0.0,
    ]

    return camera_info_msg
