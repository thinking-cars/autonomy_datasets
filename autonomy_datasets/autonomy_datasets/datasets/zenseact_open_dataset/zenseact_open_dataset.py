# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

"""Devkit-based adapter for the Zenseact Open Dataset (ZOD).

The `Zenseact Open Dataset <https://zod.zenseact.com>`_ is a multimodal driving dataset recorded
across 14 European countries. Its sensor suite is a single forward-looking 8 MP camera, three
roof-mounted Velodyne lidars merged into one point cloud per scan, and an OxTS RT3000 GNSS/IMU.

ZOD ships three sub-datasets, which this adapter selects through ``dataset_split``:

* ``frames``: 100k independent, fully annotated keyframes, each with one camera image and one
  second of surrounding lidar scans in either direction. One ROS sample is published per frame.
* ``sequences``: 1473 clips of 20 seconds. Annotated at the keyframe (the middle frame) only.
* ``drives``: 29 clips of a few minutes. Not annotated at all.

Rather than reading the native files directly, this adapter builds on the official
`zod <https://pypi.org/project/zod>`_ development kit, which resolves the dataset layout,
parses the annotations and provides the ego-motion interpolation and lidar motion compensation
used here. Its ``ZodFrames`` / ``ZodSequences`` / ``ZodDrives`` classes expect a dataset root
holding the sub-dataset directory next to the ``trainval-<subset>-<version>.json`` index::

    <dataset root>/
        trainval-frames-mini.json
        single_frames/<frame id>/
            calibration.json
            ego_motion.json
            metadata.json
            oxts.hdf5
            annotations/*.json
            camera_front_blur|camera_front_dnat/*.jpg
            lidar_velodyne/*.npy
        trainval-sequences-mini.json
        sequences/<sequence id>/...
        trainval-drives-mini.json
        drives/<drive id>/...

This is the layout the ``zod download`` CLI produces. Because the sub-datasets are commonly
downloaded into separate directories instead, an index that is not found in the dataset
directory itself is also looked up one level below it (e.g. ``frames_mini/trainval-frames-mini.json``).

Coordinate frames
-----------------
ZOD calibrates its sensors against an ISO-8855 reference frame at the center of the rear axle at
ground level, which is published as ``base_link``. The native ``camera_front`` frame is a
standard optical frame (x right, y down, z forward) and is published as ``camera_01``; the
native ``lidar_velodyne`` frame (x right, y forward, z up) is published as ``lidar_01``.

The dataset's ego-motion poses are expressed in a scene-local Cartesian frame that is anchored
at the first GNSS/IMU sample of the scene and whose x axis points along the ego vehicle's
heading at that sample. To publish an ENU-aligned ``map`` frame, the poses are rotated by that
initial heading, which is read from the scene's ``oxts.hdf5``. Scenes without an OXTS file keep
the native, heading-aligned orientation.

Sensor data
-----------
The sensors are triggered independently (camera at 10.1 Hz, lidar at 9 Hz), so each sample is
built from the frames closest in time to the reference sensor, which is the camera because ZOD
defines the camera images as its keyframes. Point clouds are motion-compensated onto the
sample's timestamp so that lidar, camera and annotations describe the same instant.
"""

import os
import subprocess
import sys
from bisect import bisect_left
from pathlib import Path
from typing import Any, Dict, Iterator, List, Optional, Sequence, Tuple

import cv2
import h5py
import numpy as np
import perception_msgs_utils as pmu
from autonomy_datasets.datasets.dataset import DatasetAdapter
from autonomy_datasets.datasets.meta_info import (
    add_object_list_publishers,
    add_object_meta_info,
    create_object_list_meta_info,
    set_object_list_sample,
)
from autonomy_datasets.datasets.utils import timestamp_micros_to_clock
from autonomy_datasets_msgs.msg import ObjectListMetaInfo
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from perception_msgs.msg import EGO, EgoData, HEXAMOTION, Object, ObjectClassification, ObjectList, ObjectReferencePoint
from rclpy.logging import get_logger
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from zod import ZodDrives, ZodFrames, ZodSequences
from zod.constants import (
    AnnotationProject,
    Anonymization,
    Camera,
    DRIVES,
    FRAMES,
    Lidar,
    SEQUENCES,
    TRAINVAL_FILES,
    VERSIONS,
)
from zod.data_classes.calibration import Calibration, LidarCalibration
from zod.data_classes.ego_motion import EgoMotion
from zod.data_classes.geometry import Pose
from zod.data_classes.info import Information
from zod.data_classes.sensor import LidarData

LOGGER = get_logger("autonomy_datasets.zenseact_open_dataset")

# Sub-datasets selectable via dataset_split, mapped to their devkit class and index directory.
_SUBSETS = {
    "frames": (ZodFrames, FRAMES),
    "sequences": (ZodSequences, SEQUENCES),
    "drives": (ZodDrives, DRIVES),
}

# Splits of a sub-dataset; "all" publishes the train and val split together.
_SPLITS = ("train", "val", "all")

# Anonymizations ZOD releases its camera images in; the original images are not published.
_ANONYMIZATIONS = (Anonymization.BLUR.value, Anonymization.DNAT.value)

# ZOD annotates its objects with a superclass and, for some of them, an object type. The ROS
# classification is looked up by "<class>_<type>" first and by "<class>" second, so that classes
# without a type and types that ZOD does not release for every object still resolve. Objects
# flagged "unclear" are published as UNCLASSIFIED regardless of their class.
#
# TRUCK, TRAILER, TRAIN and VAN are deprecated in perception_msgs and map to UTILITY / CAR.
# The static roadside classes (poles, signs, signals, guides, barriers) have no representation
# in perception_msgs at all and are therefore published as UNKNOWN, i.e. "definitely none of the
# other defined classes"; their ZOD class is preserved in the object list's meta information.
_CLASS_MAPPING: Dict[str, int] = {
    "Vehicle": ObjectClassification.CAR,
    "Vehicle_Car": ObjectClassification.CAR,
    "Vehicle_Van": ObjectClassification.CAR,
    "Vehicle_Truck": ObjectClassification.UTILITY,
    "Vehicle_Bus": ObjectClassification.BUS,
    "Vehicle_Trailer": ObjectClassification.UTILITY,
    "Vehicle_TramTrain": ObjectClassification.UTILITY,
    "Vehicle_HeavyEquip": ObjectClassification.UTILITY,
    "Vehicle_Emergency": ObjectClassification.UTILITY,
    "Vehicle_Other": ObjectClassification.UNKNOWN,
    "VulnerableVehicle": ObjectClassification.BICYCLE,
    "VulnerableVehicle_Bicycle": ObjectClassification.BICYCLE,
    "VulnerableVehicle_Motorcycle": ObjectClassification.MOTORCYCLE,
    "VulnerableVehicle_Stroller": ObjectClassification.VRU,
    "VulnerableVehicle_Wheelchair": ObjectClassification.VRU,
    "VulnerableVehicle_PersonalTransporter": ObjectClassification.MICRO,
    "VulnerableVehicle_Other": ObjectClassification.VRU,
    "Pedestrian": ObjectClassification.PEDESTRIAN,
    "Animal": ObjectClassification.ANIMAL,
    "PoleObject": ObjectClassification.UNKNOWN,
    "TrafficBeacon": ObjectClassification.UNKNOWN,
    "TrafficSign": ObjectClassification.UNKNOWN,
    "TrafficSignal": ObjectClassification.UNKNOWN,
    "TrafficGuide": ObjectClassification.UNKNOWN,
    "DynamicBarrier": ObjectClassification.UNKNOWN,
    "Unclear": ObjectClassification.UNCLASSIFIED,
}

# ZOD publishes no dimensions for its collection vehicles, so EgoData is filled with the
# dimensions of a large passenger estate car. They are consistent with the released calibration
# (roof lidar 1.70m above ground, camera 2.05m ahead of the rear axle) and with the devkit's own
# ego-return box, which discards lidar returns within 3.0m x 6.0m around the roof lidar.
_EGO_LENGTH = 4.95
_EGO_WIDTH = 1.95
_EGO_HEIGHT = 1.75

# Speed below which the ego vehicle is reported to be at standstill [m/s]
_STANDSTILL_VELOCITY = 0.1

_PRINTED_MESSAGES: set = set()


class ZenseactOpenDatasetAdapter(DatasetAdapter):
    """Converts Zenseact Open Dataset scenes to normalized ROS 2 messages."""

    VERSION = "1.0.0"
    RELEASE_NOTES = {"1.0.0": "Initial integration into Autonomy.Datasets"}

    def __init__(
        self,
        data_publishers: Dict[str, Any],
        dataset_root_dir: str,
        split: str,
        publish_ego_data: bool = True,
        publish_camera_images: bool = True,
        publish_lidar_pointclouds: bool = True,
        publish_lidar_object_lists: bool = True,
        publish_camera_01_object_lists: bool = True,
        anonymization: str = "blur",
        image_scale: float = 1.0,
        sync_tolerance_seconds: float = 0.1,
        rosbag_duration_seconds: float = 20.0,
        frames_per_scene: int = 100,
        motion_compensate_lidar: bool = True,
        auto_download: bool = True,
        download_url: str = "",
        start_scene_index: int = 0,
    ) -> None:
        """Initialize the adapter, download missing data and index the selected scenes.

        Args:
            data_publishers: Mapping of topic names to publisher instances.
            dataset_root_dir: Root directory the dataset was downloaded into.
            split: Sub-dataset, version and split as ``<subset>_<version>_<split>``, e.g.
                ``frames_mini_val``; ``subset`` is one of frames, sequences, drives, ``version``
                one of mini, full, and ``split`` one of train, val, all.
            publish_ego_data: Whether to publish ego data.
            publish_camera_images: Whether to publish camera images.
            publish_lidar_pointclouds: Whether to publish lidar point clouds.
            publish_lidar_object_lists: Whether to publish lidar_01 object lists.
            publish_camera_01_object_lists: Whether to publish camera_01 object lists.
            anonymization: Anonymization of the published camera images, ``blur`` or ``dnat``.
            image_scale: Factor the native 3848x2168 camera images are scaled by.
            sync_tolerance_seconds: Maximum time difference for matching a sensor to a frame.
            rosbag_duration_seconds: Duration of a rosbag scene of a sequence or drive in seconds.
            frames_per_scene: Number of frames of the frames sub-dataset per rosbag scene.
            motion_compensate_lidar: Whether to motion-compensate point clouds onto the timestamp
                of the sample they are published in.
            auto_download: Whether to download missing data with the ZOD CLI.
            download_url: Personal ZOD download link used by the ZOD CLI.
            start_scene_index: Number of scenes to skip before generating samples.

        Raises:
            ValueError: If a configuration value is out of range.
            FileNotFoundError: If the requested sub-dataset is not available locally.
        """
        super().__init__(data_publishers=data_publishers)
        self.subset, self.version, self.split = _parse_split(split)
        if anonymization not in _ANONYMIZATIONS:
            raise ValueError(
                f"Unsupported Zenseact Open Dataset anonymization '{anonymization}'; "
                f"expected one of: {', '.join(_ANONYMIZATIONS)}"
            )
        if image_scale <= 0 or image_scale > 1:
            raise ValueError("Zenseact Open Dataset image_scale must be in (0, 1]")
        if sync_tolerance_seconds <= 0:
            raise ValueError("Zenseact Open Dataset sync_tolerance_seconds must be greater than 0")
        if rosbag_duration_seconds <= 0:
            raise ValueError("Zenseact Open Dataset rosbag_duration_seconds must be greater than 0")
        if frames_per_scene < 1:
            raise ValueError("Zenseact Open Dataset frames_per_scene must be at least 1")

        self.dataset_root_dir = Path(dataset_root_dir)
        self.publish_ego_data = publish_ego_data
        self.publish_camera_images = publish_camera_images
        self.publish_lidar_pointclouds = publish_lidar_pointclouds
        self.publish_lidar_object_lists = publish_lidar_object_lists
        self.publish_camera_01_object_lists = publish_camera_01_object_lists
        self.anonymization = Anonymization(anonymization)
        self.image_scale = image_scale
        self.sync_tolerance_seconds = sync_tolerance_seconds
        self.rosbag_duration_seconds = rosbag_duration_seconds
        self.frames_per_scene = frames_per_scene
        self.motion_compensate_lidar = motion_compensate_lidar
        self.start_scene_index = start_scene_index

        dataset_class, index_key = _SUBSETS[self.subset]
        trainval_file = TRAINVAL_FILES[index_key][self.version]
        subset_root = _find_subset_root(self.dataset_root_dir, trainval_file)
        if subset_root is None and auto_download:
            _download(self.dataset_root_dir, self.subset, self.version, download_url)
            subset_root = _find_subset_root(self.dataset_root_dir, trainval_file)
        if subset_root is None:
            raise FileNotFoundError(
                f"Zenseact Open Dataset index '{trainval_file}' not found in '{self.dataset_root_dir}'. "
                f"Apply for access at https://zod.zenseact.com to receive a personal download link, "
                f"then set it via the 'zod_download_url' parameter or the ZOD_DOWNLOAD_URL environment "
                f"variable, or download the data manually by running "
                f"'zod download --url=<link> --output-dir={self.dataset_root_dir} "
                f"--subset={self.subset} --version={self.version}' with the link in quotes."
            )
        self.subset_root = subset_root
        LOGGER.info(f"Reading Zenseact Open Dataset {self.subset} ({self.version}) from '{subset_root}'")

        # The full index holds 100k entries; it is read in-process because the devkit's
        # multiprocessing fallback would fork the running ROS node.
        self.dataset = dataset_class(str(subset_root), version=self.version, mp=False)
        infos = self.dataset.get_all_infos()
        ids = self.dataset.get_all_ids() if self.split == "all" else self.dataset.get_split(self.split)
        # Scenes are published in recording order, so that the samples of a rosbag stay ordered
        # in time and are replayed in the order they were written.
        self.scene_ids = sorted(ids, key=lambda scene_id: (infos[scene_id].keyframe_time, scene_id))
        if not self.scene_ids:
            raise FileNotFoundError(
                f"Zenseact Open Dataset {self.subset} ({self.version}) holds no scene for split '{self.split}'"
            )
        LOGGER.info(f"Found {len(self.scene_ids)} Zenseact Open Dataset scene(s) for split '{split}'")

        if self.publish_ego_data:
            self.data_publishers["ego_data"] = None
        if self.publish_lidar_pointclouds:
            self.data_publishers["lidar_01/point_cloud"] = None
        if self.publish_camera_images:
            self.data_publishers["camera_01/image_raw"] = None
            self.data_publishers["camera_01/camera_info"] = None
        if self.publish_lidar_object_lists:
            add_object_list_publishers(self.data_publishers, "object_list/lidar_01")
        if self.publish_camera_01_object_lists:
            add_object_list_publishers(self.data_publishers, "object_list/camera_01")

    def generate_samples(self) -> Iterator[Tuple[int, Dict[str, Any]]]:
        """Yield time-synchronized samples of every selected frame, sequence or drive."""
        sample_index = 0
        scene_index = 0
        last_scene_id = None
        infos = self.dataset.get_all_infos()
        for position, dataset_scene_id in enumerate(self.scene_ids):
            info = infos[dataset_scene_id]
            calibration = Calibration.from_json_path(info.calibration_path)
            ego_motion = EgoMotion.from_json_path(info.ego_motion_path)
            map_from_local = _map_from_local(info.oxts_path)
            annotations = (
                _read_object_annotations(info) if self.publish_lidar_object_lists or self.publish_camera_01_object_lists else []
            )
            camera_frames = _SensorFrames(self._camera_frames(info))
            lidar_frames = _SensorFrames(info.get_lidar_frames(Lidar.VELODYNE))
            static_tf = TFMessage(transforms=_static_transforms(calibration))
            track_ids: Dict[str, int] = {}

            reference_times = self._reference_times(info, camera_frames, lidar_frames)
            keyframe_index = _keyframe_index(reference_times, info) if annotations else None
            for index, timestamp in enumerate(reference_times):
                scene_id = self._scene_id(info, position, timestamp, reference_times[0])
                if scene_id != last_scene_id:
                    last_scene_id = scene_id
                    scene_index += 1
                if scene_index <= self.start_scene_index:
                    continue

                camera_frame = camera_frames.closest(timestamp, self.sync_tolerance_seconds)
                lidar_frame = lidar_frames.closest(timestamp, self.sync_tolerance_seconds)
                if (self.publish_camera_images and camera_frame is None) or (
                    self.publish_lidar_pointclouds and lidar_frame is None
                ):
                    _print_once(
                        "Zenseact Open Dataset samples without data from one or more enabled sensors within "
                        "'zod_sync_tolerance_seconds' are skipped, as a sample has to hold data for every topic."
                    )
                    continue

                clock = timestamp_micros_to_clock(round(timestamp * 1e6))
                stamp = clock.clock
                map_from_ego = map_from_local @ ego_motion.get_poses(_clamp(timestamp, ego_motion))
                sample: Dict[str, Any] = {
                    "scene_id": scene_id,
                    "/clock": clock,
                    "/tf_static": static_tf,
                    "/tf": TFMessage(transforms=[_matrix_transform("map", "base_link", map_from_ego, stamp)]),
                }
                if self.publish_ego_data:
                    sample["ego_data"] = _ego_data(map_from_ego, ego_motion, timestamp, stamp)
                if self.publish_lidar_pointclouds:
                    assert lidar_frame is not None
                    sample["lidar_01/point_cloud"] = self._point_cloud(
                        lidar_frame, calibration.lidars[Lidar.VELODYNE], ego_motion, timestamp, stamp
                    )
                if self.publish_camera_images:
                    assert camera_frame is not None
                    sample["camera_01/image_raw"] = self._image(camera_frame, stamp)
                    sample["camera_01/camera_info"] = self._camera_info(calibration, stamp)
                # ZOD annotates the keyframe of a scene only; every other sample of a sequence or
                # drive publishes an empty object list.
                objects = annotations if index == keyframe_index else []
                if self.publish_lidar_object_lists:
                    set_object_list_sample(
                        sample,
                        "object_list/lidar_01",
                        *_object_list(objects, Lidar.VELODYNE, "lidar_01", calibration, stamp, dataset_scene_id, track_ids),
                    )
                if self.publish_camera_01_object_lists:
                    set_object_list_sample(
                        sample,
                        "object_list/camera_01",
                        *_object_list(objects, Camera.FRONT, "camera_01", calibration, stamp, dataset_scene_id, track_ids),
                    )

                sample_index += 1
                yield sample_index, sample

    def _camera_frames(self, info: Information) -> List[Any]:
        """Return the camera frames of a scene in the requested anonymization.

        Only the frames sub-dataset releases both anonymizations; sequences and drives ship the
        blurred images only, so a scene that does not hold the requested one falls back to the
        anonymization it does hold. A scene that was downloaded without any images contributes
        no camera frames at all.
        """
        available = {name.rsplit("_", 1)[-1]: frames for name, frames in info.camera_frames.items()}
        if self.anonymization.value in available:
            return available[self.anonymization.value]
        if not available:
            _print_once(
                "Zenseact Open Dataset scenes without camera images are published without them; "
                "re-download the dataset without '--no-images' to publish camera images."
            )
            return []
        fallback = sorted(available)[0]
        _print_once(
            f"Zenseact Open Dataset {self.subset} do not ship '{self.anonymization.value}' anonymized camera "
            f"images; publishing '{fallback}' anonymized images instead."
        )
        return available[fallback]

    def _reference_times(self, info: Information, camera_frames: "_SensorFrames", lidar_frames: "_SensorFrames") -> List[float]:
        """Return the timestamps a scene publishes a sample for.

        A frame contributes its single annotated keyframe; a sequence or drive is played back at
        the rate of its reference sensor, which is the camera because ZOD defines the camera
        images as its keyframes.
        """
        if self.subset == "frames":
            return [info.keyframe_time.timestamp()]
        if self.publish_camera_images and camera_frames.times:
            return list(camera_frames.times)
        if self.publish_lidar_pointclouds and lidar_frames.times:
            return list(lidar_frames.times)
        return [info.keyframe_time.timestamp()]

    def _scene_id(self, info: Information, position: int, timestamp: float, first_timestamp: float) -> str:
        """Return the ID of the rosbag scene a sample belongs to.

        Frames are independent recordings, so they are grouped into scenes of ``frames_per_scene``
        consecutive frames named after the first frame they contain. Sequences and drives are
        continuous recordings and are split into scenes of ``rosbag_duration_seconds``.
        """
        if self.subset == "frames":
            return self.scene_ids[position - position % self.frames_per_scene]
        return f"{info.id}_{int((timestamp - first_timestamp) // self.rosbag_duration_seconds) + 1:05d}"

    def _point_cloud(
        self,
        lidar_frame: Any,
        lidar_calibration: LidarCalibration,
        ego_motion: EgoMotion,
        timestamp: float,
        stamp: Time,
    ) -> PointCloud2:
        """Read a lidar scan and convert it to a ROS PointCloud2 message in the lidar_01 frame."""
        lidar_data = lidar_frame.read()
        if self.motion_compensate_lidar:
            _motion_compensate(lidar_data, ego_motion, lidar_calibration, timestamp)
        return _point_cloud_message(lidar_data, stamp)

    def _image(self, camera_frame: Any, stamp: Time) -> Image:
        """Read a camera image and convert it to a ROS Image message."""
        image = cv2.imread(camera_frame.filepath, cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f"Failed to read image: {camera_frame.filepath}")
        if self.image_scale != 1.0:
            image = cv2.resize(image, self._image_size(image.shape[1], image.shape[0]), interpolation=cv2.INTER_AREA)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return Image(
            header=Header(frame_id="camera_01", stamp=stamp),
            height=image.shape[0],
            width=image.shape[1],
            encoding="rgb8",
            step=image.shape[1] * 3,
            data=image.tobytes(),
        )

    def _camera_info(self, calibration: Calibration, stamp: Time) -> CameraInfo:
        """Convert the native camera calibration to a ROS CameraInfo message."""
        camera_calibration = calibration.cameras[Camera.FRONT]
        native_width, native_height = (int(value) for value in camera_calibration.image_dimensions)
        width, height = self._image_size(native_width, native_height)
        # Scaling an image scales the focal lengths and the principal point with it, while the
        # coefficients of the Kannala-Brandt model are scale-invariant.
        camera_matrix = np.asarray(camera_calibration.intrinsics, dtype=float)[:, :3].copy()
        camera_matrix[:2, :] *= width / native_width
        message = CameraInfo(header=Header(frame_id="camera_01", stamp=stamp), width=width, height=height)
        message.k = camera_matrix.flatten().tolist()
        message.r = np.eye(3).flatten().tolist()
        message.p = np.hstack([camera_matrix, np.zeros((3, 1))]).flatten().tolist()
        message.d = [float(value) for value in camera_calibration.distortion]
        # ZOD releases fisheye images calibrated with the Kannala-Brandt model, which ROS calls
        # the equidistant distortion model.
        message.distortion_model = "equidistant" if len(message.d) == 4 else "plumb_bob"
        return message

    def _image_size(self, native_width: int, native_height: int) -> Tuple[int, int]:
        """Return the published image size for a native image size."""
        return max(1, round(native_width * self.image_scale)), max(1, round(native_height * self.image_scale))


def _parse_split(split: str) -> Tuple[str, str, str]:
    """Split the ``<subset>_<version>_<split>`` selector into its three parts.

    Raises:
        ValueError: If the selector does not name a known sub-dataset, version and split.
    """
    supported = f"expected '<{'|'.join(_SUBSETS)}>_<{'|'.join(VERSIONS)}>_<{'|'.join(_SPLITS)}>', e.g. 'frames_mini_val'"
    parts = split.split("_")
    if len(parts) != 3:
        raise ValueError(f"Unsupported Zenseact Open Dataset split '{split}'; {supported}")
    subset, version, dataset_split = parts
    if subset not in _SUBSETS or version not in VERSIONS or dataset_split not in _SPLITS:
        raise ValueError(f"Unsupported Zenseact Open Dataset split '{split}'; {supported}")
    return subset, version, dataset_split


def _find_subset_root(dataset_root: Path, trainval_file: str) -> Optional[Path]:
    """Return the directory holding a sub-dataset index, or None if it is not available.

    The index is looked up in the dataset directory itself, which is the layout the ``zod
    download`` CLI produces, and one level below it, which is the layout that results from
    downloading the sub-datasets into separate directories.
    """
    if (dataset_root / trainval_file).is_file():
        return dataset_root
    if not dataset_root.is_dir():
        return None
    return next((child for child in sorted(dataset_root.iterdir()) if (child / trainval_file).is_file()), None)


def _download(dataset_root: Path, subset: str, version: str, download_url: str) -> None:
    """Download a sub-dataset with the ZOD CLI.

    ZOD requires registration; a personal download link is issued per user, so it has to be
    passed in rather than being hard-coded. Downloading is delegated to the CLI shipped with the
    devkit, which resolves the archives of the requested sub-dataset, downloads them in parallel
    and extracts them into the dataset directory.
    """
    download_url = download_url or os.environ.get("ZOD_DOWNLOAD_URL", "")
    if not download_url:
        LOGGER.warn(
            "No Zenseact Open Dataset download link configured; set the 'zod_download_url' parameter or the "
            "ZOD_DOWNLOAD_URL environment variable to the link you received from https://zod.zenseact.com"
        )
        return
    dataset_root.mkdir(parents=True, exist_ok=True)
    command = [
        sys.executable,
        "-m",
        "zod.cli.main",
        "download",
        "--no-confirm",
        f"--url={download_url}",
        f"--output-dir={dataset_root}",
        f"--subset={subset}",
        f"--version={version}",
    ]
    # The download link is personal, so it is not logged; the CLI reports its own progress.
    LOGGER.info(f"Downloading Zenseact Open Dataset {subset} ({version}) into '{dataset_root}'")
    result = subprocess.run(command)
    if result.returncode != 0:
        raise RuntimeError(
            f"Downloading the Zenseact Open Dataset failed with exit code {result.returncode}; "
            f"check the download link and rerun, the CLI resumes partial downloads."
        )


def _read_object_annotations(info: Information) -> List[Any]:
    """Return the 3D object annotations of a scene's keyframe, if it is annotated.

    ZOD annotates every object with a 2D box in the camera image and most of them with a 3D
    cuboid on top. Objects without a released 3D cuboid cannot be published as a 3D object and
    are left out; the drives sub-dataset is not annotated at all.
    """
    if AnnotationProject.OBJECT_DETECTION not in info.annotations:
        return []
    objects = info.annotations[AnnotationProject.OBJECT_DETECTION].read()
    annotated_3d = [obj for obj in objects if obj.box3d is not None]
    if len(annotated_3d) < len(objects):
        _print_once(
            "Zenseact Open Dataset annotates a share of its objects with a 2D image box only; "
            "those are not published, as they cannot be expressed as a 3D object."
        )
    return annotated_3d


def _keyframe_index(reference_times: Sequence[float], info: Information) -> int:
    """Return the index of the sample that carries the annotations of a scene.

    ZOD annotates one keyframe per frame and per sequence, so exactly the sample recorded closest
    to that keyframe publishes the object lists; every other sample publishes an empty one.
    """
    keyframe_time = info.keyframe_time.timestamp()
    return min(range(len(reference_times)), key=lambda index: abs(reference_times[index] - keyframe_time))


class _SensorFrames:
    """Time-sorted sensor frames of one sensor within a scene."""

    def __init__(self, frames: Sequence[Any]) -> None:
        """Index the given sensor frames by their timestamp.

        Args:
            frames: Sensor frames of one sensor, as released by the devkit.
        """
        self.frames = sorted(frames, key=lambda frame: frame.time)
        self.times = [frame.time.timestamp() for frame in self.frames]

    def closest(self, timestamp: float, tolerance_seconds: float) -> Optional[Any]:
        """Return the frame recorded closest to a timestamp, or None outside the tolerance."""
        if not self.frames:
            return None
        index = bisect_left(self.times, timestamp)
        candidates = range(max(index - 1, 0), min(index, len(self.frames) - 1) + 1)
        closest = min(candidates, key=lambda candidate: abs(self.times[candidate] - timestamp))
        if abs(self.times[closest] - timestamp) > tolerance_seconds:
            return None
        return self.frames[closest]


def _clamp(timestamp: float, ego_motion: EgoMotion) -> float:
    """Clamp a timestamp into the range the ego motion of a scene covers.

    The ego motion covers exactly the recorded time span of a scene, so its first and last sample
    can fall marginally outside of it through the rounding of the timestamps parsed from the
    sensor file names. The devkit rejects those instead of extrapolating, so they are pinned to
    the closest ego motion sample.
    """
    return float(min(max(timestamp, ego_motion.timestamps[0]), ego_motion.timestamps[-1]))


def _map_from_local(oxts_path: str) -> np.ndarray:
    """Return the rotation from a scene's native pose frame into an ENU-aligned map frame.

    ZOD expresses its poses relative to the first GNSS/IMU sample of the scene, with the x axis
    along the ego vehicle's heading at that sample. Rotating them by that heading, which is
    published in the OXTS file as degrees clockwise from north, yields an east-north-up frame.
    """
    try:
        with h5py.File(oxts_path, "r") as oxts:
            heading = float(oxts["heading"][0])
    except (OSError, KeyError, IndexError) as error:
        _print_once(
            f"Zenseact Open Dataset OXTS data is not available ({error}); the 'map' frame keeps the "
            f"dataset's own orientation, which is aligned with the ego vehicle's initial heading "
            f"instead of with east-north-up."
        )
        return np.eye(4)
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler("z", np.pi / 2 - np.deg2rad(heading)).as_matrix()
    return transform


def _motion_compensate(
    lidar_data: LidarData, ego_motion: EgoMotion, lidar_calibration: LidarCalibration, target_timestamp: float
) -> None:
    """Move a lidar scan onto the timestamp of the sample it is published in, in place.

    This mirrors the devkit's ``motion_compensate_scanwise``, but pins both timestamps into the
    range the ego motion covers (see :func:`_clamp`). The scan is rigidly shifted by the ego
    motion between the two timestamps, so that it lines up with the camera image and the
    annotations of the same sample.
    """
    source_pose = ego_motion.get_poses(_clamp(lidar_data.core_timestamp, ego_motion))
    target_pose = ego_motion.get_poses(_clamp(target_timestamp, ego_motion))
    odometry = np.linalg.inv(target_pose) @ source_pose
    lidar_data.transform(lidar_calibration.extrinsics)
    lidar_data.transform(Pose(odometry))
    lidar_data.transform(lidar_calibration.extrinsics.inverse)


def _static_transforms(calibration: Calibration) -> List[TransformStamped]:
    """Build the static transforms from the ISO-8855 vehicle frame to every sensor frame."""
    return [
        _matrix_transform("base_link", "lidar_01", calibration.lidars[Lidar.VELODYNE].extrinsics.transform),
        _matrix_transform("base_link", "camera_01", calibration.cameras[Camera.FRONT].extrinsics.transform),
    ]


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


def _ego_data(map_from_ego: np.ndarray, ego_motion: EgoMotion, timestamp: float, stamp: Time) -> EgoData:
    """Build the EgoData message of a sample from the interpolated GNSS/IMU state.

    The OXTS state is not published in a single convention: the velocities are given in the
    ISO-8855 vehicle frame (x forward, y left, z up), while the accelerations and angular rates
    are given in the sensor's own frame (x forward, y right, z down). The lateral acceleration
    and the yaw rate are therefore negated, which was confirmed against the lateral acceleration
    and the heading rate derived from the released poses.
    """
    state = ego_motion.interpolate(np.array([_clamp(timestamp, ego_motion)]))
    ego_data = EgoData(header=Header(frame_id="map", stamp=stamp))
    pmu.initialize_state(ego_data.state, EGO.MODEL_ID)
    # ZOD references its sensor calibration to the center of the rear axle at ground level
    ego_data.state.reference_point = ObjectReferencePoint(value=ObjectReferencePoint.REAR_AXLE_GROUND)
    ego_data.state.continuous_state[EGO.X] = float(map_from_ego[0, 3])
    ego_data.state.continuous_state[EGO.Y] = float(map_from_ego[1, 3])
    ego_data.state.continuous_state[EGO.Z] = float(map_from_ego[2, 3])
    roll, pitch, yaw = Rotation.from_matrix(map_from_ego[:3, :3]).as_euler("xyz")
    ego_data.state.continuous_state[EGO.ROLL] = float(roll)
    ego_data.state.continuous_state[EGO.PITCH] = float(pitch)
    ego_data.state.continuous_state[EGO.YAW] = float(yaw)
    ego_data.state.continuous_state[EGO.VEL_LON] = float(state.velocities[0, 0])
    ego_data.state.continuous_state[EGO.VEL_LAT] = float(state.velocities[0, 1])
    ego_data.state.continuous_state[EGO.ACC_LON] = float(state.accelerations[0, 0])
    ego_data.state.continuous_state[EGO.ACC_LAT] = -float(state.accelerations[0, 1])
    ego_data.state.continuous_state[EGO.YAW_RATE] = -float(np.deg2rad(state.angular_rates[0, 2]))
    ego_data.state.discrete_state[EGO.STANDSTILL] = int(np.linalg.norm(state.velocities[0, :2]) < _STANDSTILL_VELOCITY)
    ego_data.state.discrete_state[EGO.TURN_INDICATOR] = EGO.TURN_INDICATOR_UNKNOWN
    ego_data.state.discrete_state[EGO.BRAKE_LIGHT] = EGO.LIGHT_UNKNOWN
    ego_data.state.discrete_state[EGO.REVERSE_LIGHT] = EGO.LIGHT_UNKNOWN
    ego_data.length, ego_data.width, ego_data.height = _EGO_LENGTH, _EGO_WIDTH, _EGO_HEIGHT
    return ego_data


def _point_cloud_message(lidar_data: LidarData, stamp: Time) -> PointCloud2:
    """Convert a native lidar scan to a ROS PointCloud2 message.

    ZOD merges the returns of all three roof lidars into one scan; ``diode_index`` identifies the
    emitter, and therefore the lidar, a point was measured by. The native per-point timing is
    published as absolute seconds.
    """
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="timestamp", offset=16, datatype=PointField.FLOAT64, count=1),
        PointField(name="diode_index", offset=24, datatype=PointField.UINT8, count=1),
    ]
    dtype = [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4"), ("timestamp", "<f8"), ("diode_index", "u1")]
    points = np.empty(len(lidar_data.points), dtype=dtype)
    points["x"] = lidar_data.points[:, 0]
    points["y"] = lidar_data.points[:, 1]
    points["z"] = lidar_data.points[:, 2]
    points["intensity"] = lidar_data.intensity
    points["timestamp"] = lidar_data.timestamps
    points["diode_index"] = lidar_data.diode_idx
    return create_cloud(Header(frame_id="lidar_01", stamp=stamp), fields, points)


def _object_list(
    objects: List[Any],
    frame: Any,
    frame_id: str,
    calibration: Calibration,
    stamp: Time,
    dataset_scene_id: str,
    track_ids: Dict[str, int],
) -> Tuple[ObjectList, ObjectListMetaInfo]:
    """Convert ZOD 3D cuboids into a ROS ObjectList and its meta information.

    Args:
        objects: Annotated objects of the sample, all of which carry a 3D cuboid.
        frame: ZOD coordinate frame the cuboids are converted into.
        frame_id: ROS frame the object list is published in.
        calibration: Calibration of the scene, used to convert between the frames.
        stamp: ROS Time message of the sample.
        dataset_scene_id: ID of the ZOD frame, sequence or drive the objects were annotated in.
        track_ids: Mapping of annotation UUIDs to object IDs, shared across a scene.
    """
    message = ObjectList(header=Header(frame_id=frame_id, stamp=stamp))
    meta_info_msg = create_object_list_meta_info(message, dataset_scene_id)
    for annotation in objects:
        box = annotation.box3d.copy()
        box.convert_to(frame, calibration)
        obj = Object(id=_track_id(annotation.uuid, track_ids), existence_probability=1.0)
        pmu.initialize_state(obj.state, HEXAMOTION.MODEL_ID)
        obj.state.continuous_state[HEXAMOTION.X] = float(box.center[0])
        obj.state.continuous_state[HEXAMOTION.Y] = float(box.center[1])
        obj.state.continuous_state[HEXAMOTION.Z] = float(box.center[2])
        # ZOD stores the orientation as a [w, x, y, z] quaternion
        roll, pitch, yaw = Rotation.from_quat(np.roll(box.orientation.elements, -1)).as_euler("xyz")
        obj.state.continuous_state[HEXAMOTION.ROLL] = float(roll)
        obj.state.continuous_state[HEXAMOTION.PITCH] = float(pitch)
        obj.state.continuous_state[HEXAMOTION.YAW] = float(yaw)
        obj.state.continuous_state[HEXAMOTION.LENGTH] = float(box.size[0])
        obj.state.continuous_state[HEXAMOTION.WIDTH] = float(box.size[1])
        obj.state.continuous_state[HEXAMOTION.HEIGHT] = float(box.size[2])
        obj.state.discrete_state[HEXAMOTION.TURN_INDICATOR] = HEXAMOTION.TURN_INDICATOR_UNKNOWN
        obj.state.discrete_state[HEXAMOTION.BRAKE_LIGHT] = HEXAMOTION.LIGHT_UNKNOWN
        obj.state.discrete_state[HEXAMOTION.REVERSE_LIGHT] = HEXAMOTION.LIGHT_UNKNOWN
        obj.state.classifications = [ObjectClassification(type=_classification(annotation), probability=1.0)]
        add_object_meta_info(meta_info_msg, obj.id, _object_meta_info(annotation))
        message.objects.append(obj)
    return message, meta_info_msg


def _object_meta_info(annotation: Any) -> List[Tuple[str, Any]]:
    """Return the ZOD annotations of an object that perception_msgs cannot represent."""
    info: List[Tuple[str, Any]] = [
        ("original_class", annotation.name),
        ("original_subclass", _subclass(annotation)),
        ("annotation_uuid", annotation.uuid),
        ("unclear", annotation.unclear),
    ]
    for key, value in (
        ("object_type", annotation.object_type),
        ("occlusion_level", annotation.occlusion_level),
        ("with_rider", annotation.with_rider),
        ("emergency", annotation.emergency),
        ("artificial", annotation.artificial),
        ("traffic_content_visible", annotation.traffic_content_visible),
    ):
        if value is not None:
            info.append((key, value))
    return info


def _subclass(annotation: Any) -> str:
    """Return the ZOD sub-class of an object, or its class if it cannot be determined."""
    try:
        return annotation.subclass
    except ValueError:
        return annotation.name


def _classification(annotation: Any) -> int:
    """Map a ZOD class and object type to a ROS ObjectClassification type."""
    if annotation.unclear:
        return ObjectClassification.UNCLASSIFIED
    return _CLASS_MAPPING.get(
        f"{annotation.name}_{annotation.object_type}",
        _CLASS_MAPPING.get(annotation.name, ObjectClassification.UNKNOWN),
    )


def _track_id(uuid: str, track_ids: Dict[str, int]) -> int:
    """Map the UUID an object is annotated with to a consecutive integer ID."""
    return track_ids.setdefault(str(uuid), len(track_ids))


def _print_once(message: str) -> None:
    """Log a message the first time it occurs, to keep the playback log readable."""
    if message not in _PRINTED_MESSAGES:
        _PRINTED_MESSAGES.add(message)
        LOGGER.info(message)
