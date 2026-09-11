# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

"""SDK-based adapter for the FZI-AURA dataset.

`FZI-AURA <https://huggingface.co/datasets/fzi-forschungszentrum-informatik/FZI-AURA>`_ is a
multimodal driving dataset recorded across southern Germany with FZI's CoCar NextGen research
vehicle. This adapter builds on the official
`FZI-AURA SDK <https://github.com/fzi-forschungszentrum-informatik/fzi-aura-sdk>`_, which
resolves the dataset layout, the per-frame sensor associations, the annotations, the calibration
and the ego poses. Its ``FZIAURADataset`` expects a dataset root as produced by the
``fzi-aura-download`` CLI::

    <dataset root>/
        dataset.json
        available_data.json
        splits/v1.0/[train|val|test].txt
        scenes/<scene name>/
            scene.json
            samples.jsonl
            calibration.json
            camera/<camera id>/*.jpg
            lidar/[raw|motion_compensated]/<lidar id>/*.pcd
            radar/<radar id>/*.pcd
            labels/boxes_3d.jsonl
            labels/boxes_3d_sensor_frame/<lidar id>/boxes_3d.jsonl
            labels/semantic/<lidar id>/*.label
            ego/[poses|vehicle_signals].parquet

Scenes and samples
------------------
A scene is a self-contained recording of roughly 20 seconds sampled at 10 Hz, which is already
the granularity a rosbag scene is written at; every dataset scene therefore becomes one scene.
FZI-AURA annotates 2 Hz keyframes only, and publishes the sensor payloads of keyframes and
non-keyframes as separate download layers. ``samples`` selects which of the two the adapter
plays back: ``keyframes`` publishes the annotated 2 Hz samples that the default download covers,
``all`` publishes the full 10 Hz sample stream, whose non-keyframe samples carry sensor data only
where the non-keyframe layers were downloaded, and never carry object lists.

Coordinate frames
-----------------
FZI-AURA calibrates its sensors against ``base_link``, which follows the ROS convention (x
forward, y left, z up) and is published unchanged. Sensor frames are published as
``<modality>_<sensor id>``, e.g. ``lidar_top_left``, because a bare sensor ID is not unique
across modalities (both a lidar and a radar are called ``front_left``).

The scene-wide frame the ego poses are expressed in is called ``odom`` in the dataset and is
published as ``map``, which is the frame the remaining adapters of this package publish their ego
poses in. Its axes carry an arbitrary orientation per recording that is neither gravity-aligned
nor north-referenced: the released poses of a scene can hold a roll of more than ten degrees
while the vehicle drives level. The INS state in the vehicle signals does publish a proper
east-north-up attitude, so the poses are rotated by the offset between the two at the first
sample of a scene, which yields an ENU-aligned ``map`` frame. Scenes without vehicle signals keep
the native orientation of the released poses.

Sensor data
-----------
Point clouds are published with the fields of their PCD file under their native names, so that
the differing schemas of the sensors reach consumers unchanged and every field keeps the meaning
the dataset documents for it. No field is renamed to match the naming of another dataset: the
Ouster clouds report their return strength as ``reflectivity`` and hold no ``intensity`` at all,
the Aeva clouds carry both fields side by side, and the radar detections report their radial
velocity as ``range_rate``. The RViz configuration of this dataset selects the matching channel
per sensor.

Semantic lidar labels are published as additional ``semantic_id`` and ``instance_id`` point
fields of the cloud they annotate. They follow the point order of both lidar stages, so they are
attached to raw and motion-compensated clouds alike.
"""

import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, Iterator, List, Optional, Sequence, Tuple

import cv2
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
from fzi_aura import Box3D, FZIAURADataset, FZIAURAFrame, FZIAURAScene, PointCloud
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from perception_msgs.msg import EGO, EgoData, HEXAMOTION, Object, ObjectClassification, ObjectList, ObjectReferencePoint
from rclpy.logging import get_logger
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

LOGGER = get_logger("autonomy_datasets.fzi_aura")

# Scene-level splits FZI-AURA releases; "all" publishes every scene of the dataset.
_SPLITS = ("train", "val", "test", "all")

# Sample streams selectable via the 'fzi_aura_samples' parameter.
_SAMPLE_STREAMS = ("keyframes", "all")

# Lidar processing stages the dataset releases its point clouds in.
_LIDAR_STAGES = ("motion_compensated", "raw")

# Sensor IDs ordered as they are mapped to the canonical camera_XX/lidar_XX/radar_XX topics.
# The three forward-facing cameras come first, followed by the remaining ones clockwise around
# the vehicle; the rotating Ouster lidars come before the Aeva FMCW lidars, because they are the
# ones the semantic labels and the per-sensor object lists are released for.
# camera_01 and lidar_01 are the reference sensors the object lists are published in.
_CAMERA_IDS = (
    "front_medium",
    "front_wide",
    "front_tele",
    "right_forward",
    "right_rearward",
    "rear_wide",
    "left_rearward",
    "left_forward",
)
_LIDAR_IDS = (
    "top_left",
    "top_right",
    "front_left",
    "front_right",
    "rear_left",
    "rear_right",
    "aeva_front_center",
    "aeva_front_left",
    "aeva_front_right",
    "aeva_side_left",
    "aeva_side_right",
    "aeva_rear_center",
)
_RADAR_IDS = (
    "front_left",
    "front_right",
    "rear_center",
)

# Modality of a sensor mapped to the availability key of its data layers. The lidar key depends
# on the processing stage and is completed with it.
_MODALITY_TO_AVAILABILITY = {"camera": "camera", "radar": "radar", "lidar": "lidar_{stage}"}

# Data layers of the release, as accepted by the 'fzi-aura-download' CLI. 'base_keyframes' holds
# the indexes, calibration, ego data and annotations and is added to every download by the CLI.
_DOWNLOAD_LAYERS = (
    "base_keyframes",
    "camera_keyframes",
    "lidar_motion_compensated_keyframes",
    "lidar_raw_keyframes",
    "radar_keyframes",
    "camera_nonkeyframes",
    "lidar_raw_nonkeyframes",
    "radar_nonkeyframes",
)

# FZI-AURA annotates its objects with the detection classes below. The static and unclassifiable
# ones have no representation in perception_msgs and are published as UNKNOWN, i.e. "definitely
# none of the other defined classes"; the dataset's own class is preserved in the object list's
# meta information.
#
# TRUCK, TRAILER, TRAIN and VAN are deprecated in perception_msgs and map to UTILITY / CAR.
# The dataset annotates two-wheelers both as the bare vehicle ("bicycle") and as the vehicle
# together with its rider ("bicyclist"); perception_msgs defines BICYCLE, MOTORCYCLE and MICRO as
# covering the vehicle and its rider, so both spellings map to the same class. A "rider" box
# holds the person alone, who is a vulnerable road user but not a pedestrian on foot.
_CLASS_MAPPING: Dict[str, int] = {
    "person": ObjectClassification.PEDESTRIAN,
    "rider": ObjectClassification.VRU,
    "car": ObjectClassification.CAR,
    "truck": ObjectClassification.UTILITY,
    "bus": ObjectClassification.BUS,
    "on rails": ObjectClassification.UTILITY,
    "motorcycle": ObjectClassification.MOTORCYCLE,
    "motorcyclist": ObjectClassification.MOTORCYCLE,
    "bicycle": ObjectClassification.BICYCLE,
    "bicyclist": ObjectClassification.BICYCLE,
    "portable": ObjectClassification.MICRO,
    "portable-rider": ObjectClassification.MICRO,
    "caravan": ObjectClassification.UTILITY,
    "trailer": ObjectClassification.UTILITY,
    "dynamic": ObjectClassification.UNKNOWN,
}

# PCD type codes and sizes mapped to their PointCloud2 datatype and numpy dtype.
_PCD_TO_POINT_FIELD: Dict[Tuple[str, int], Tuple[int, str]] = {
    ("F", 4): (PointField.FLOAT32, "<f4"),
    ("F", 8): (PointField.FLOAT64, "<f8"),
    ("U", 1): (PointField.UINT8, "u1"),
    ("U", 2): (PointField.UINT16, "<u2"),
    ("U", 4): (PointField.UINT32, "<u4"),
    ("I", 1): (PointField.INT8, "i1"),
    ("I", 2): (PointField.INT16, "<i2"),
    ("I", 4): (PointField.INT32, "<i4"),
}

# Dimensions of the CoCar NextGen research vehicle (Audi A6 Avant C8) in meters.
_EGO_LENGTH = 4.94
_EGO_WIDTH = 1.89
_EGO_HEIGHT = 1.47

# Speed below which the ego vehicle is reported to be at standstill [m/s]
_STANDSTILL_VELOCITY = 0.1

_PRINTED_MESSAGES: set = set()


class FziAuraAdapter(DatasetAdapter):
    """Converts FZI-AURA scenes to normalized ROS 2 messages."""

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
        publish_radar_pointclouds: bool = True,
        publish_lidar_object_lists: bool = True,
        publish_base_link_object_lists: bool = True,
        scenes: str = "",
        samples: str = "keyframes",
        lidar_stage: str = "motion_compensated",
        publish_semantic_labels: bool = True,
        image_scale: float = 1.0,
        auto_download: bool = True,
        download_layers: str = "",
        start_scene_index: int = 0,
    ) -> None:
        """Initialize the adapter, download missing data and index the selected scenes.

        Args:
            data_publishers: Mapping of topic names to publisher instances.
            dataset_root_dir: Root directory the dataset was downloaded into.
            split: Scene-level split to publish; one of train, val, test, all.
            publish_ego_data: Whether to publish ego data.
            publish_camera_images: Whether to publish camera images.
            publish_lidar_pointclouds: Whether to publish lidar point clouds.
            publish_radar_pointclouds: Whether to publish radar point clouds.
            publish_lidar_object_lists: Whether to publish lidar_01 object lists.
            publish_base_link_object_lists: Whether to publish base_link object lists.
            scenes: Comma-separated dataset scene IDs to publish; all scenes of the split if empty.
            samples: Sample stream to publish, ``keyframes`` (2 Hz, annotated) or ``all`` (10 Hz).
            lidar_stage: Lidar processing stage to publish, ``motion_compensated`` or ``raw``.
            publish_semantic_labels: Whether to publish semantic lidar labels as point fields.
            image_scale: Factor the native camera images are scaled by.
            auto_download: Whether to download missing data with the FZI-AURA SDK downloader.
            download_layers: Comma-separated data layers to download; the SDK default if empty.
            start_scene_index: Number of scenes to skip before generating samples.

        Raises:
            ValueError: If a configuration value is out of range.
            FileNotFoundError: If the dataset is not available locally.
        """
        super().__init__(data_publishers=data_publishers)
        if split not in _SPLITS:
            raise ValueError(f"Unsupported FZI-AURA split '{split}'; expected one of: {', '.join(_SPLITS)}")
        if samples not in _SAMPLE_STREAMS:
            raise ValueError(f"Unsupported FZI-AURA sample stream '{samples}'; expected one of: {', '.join(_SAMPLE_STREAMS)}")
        if lidar_stage not in _LIDAR_STAGES:
            raise ValueError(f"Unsupported FZI-AURA lidar stage '{lidar_stage}'; expected one of: {', '.join(_LIDAR_STAGES)}")
        if image_scale <= 0 or image_scale > 1:
            raise ValueError("FZI-AURA image_scale must be in (0, 1]")

        self.dataset_root_dir = Path(dataset_root_dir)
        self.split = split
        self.publish_ego_data = publish_ego_data
        self.publish_camera_images = publish_camera_images
        self.publish_lidar_pointclouds = publish_lidar_pointclouds
        self.publish_radar_pointclouds = publish_radar_pointclouds
        self.publish_lidar_object_lists = publish_lidar_object_lists
        self.publish_base_link_object_lists = publish_base_link_object_lists
        self.scene_ids = _split_list(scenes)
        self.samples = samples
        self.lidar_stage = lidar_stage
        self.publish_semantic_labels = publish_semantic_labels
        self.image_scale = image_scale
        self.start_scene_index = start_scene_index

        if not (self.dataset_root_dir / "dataset.json").is_file() and auto_download:
            _download(self.dataset_root_dir, self.split, self.scene_ids, _split_list(download_layers))
        if not (self.dataset_root_dir / "dataset.json").is_file():
            raise FileNotFoundError(
                f"FZI-AURA index 'dataset.json' not found in '{self.dataset_root_dir}'. Accept the dataset "
                f"terms at https://huggingface.co/datasets/fzi-forschungszentrum-informatik/FZI-AURA, log in "
                f"with 'hf auth login' and download the data by running "
                f"'fzi-aura-download {self.dataset_root_dir}', or enable the 'fzi_aura_auto_download' parameter."
            )

        LOGGER.info(f"Reading FZI-AURA from '{self.dataset_root_dir}'")
        self.dataset = FZIAURADataset(self.dataset_root_dir, split=None if self.split == "all" else self.split)
        self.scenes = self._select_scenes()
        if not self.scenes:
            raise FileNotFoundError(
                f"FZI-AURA holds no scene for split '{self.split}'"
                + (f" and the requested scene(s) {', '.join(self.scene_ids)}" if self.scene_ids else "")
            )
        LOGGER.info(f"Found {len(self.scenes)} FZI-AURA scene(s) for split '{self.split}'")

        # The sensor suite differs between scenes, so a sensor is mapped to its canonical topic
        # only when at least one selected scene holds it; a publisher is therefore never
        # advertised for a topic that no selected scene can fill.
        self.camera_topics = self._sensor_topics("camera", _CAMERA_IDS)
        self.lidar_topics = self._sensor_topics("lidar", _LIDAR_IDS)
        self.radar_topics = self._sensor_topics("radar", _RADAR_IDS)
        # The sensor payloads are downloaded per modality, while the annotations are always part
        # of the download, so a missing payload layer silences the sensor topics of a modality
        # without silencing the object lists annotated against its reference sensor.
        self.cameras = self._published_sensors("camera") if self.publish_camera_images else {}
        self.lidars = self._published_sensors("lidar") if self.publish_lidar_pointclouds else {}
        self.radars = self._published_sensors("radar") if self.publish_radar_pointclouds else {}
        # Object lists are annotated against the first lidar of the sensor suite a scene holds.
        self.reference_lidar = next(iter(self.lidar_topics), None)

        if self.publish_ego_data:
            self.data_publishers["ego_data"] = None
        for topic in self.lidars.values():
            self.data_publishers[f"{topic}/point_cloud"] = None
        for topic in self.radars.values():
            self.data_publishers[f"{topic}/point_cloud"] = None
        for topic in self.cameras.values():
            self.data_publishers[f"{topic}/image_raw"] = None
            self.data_publishers[f"{topic}/camera_info"] = None
        if self.publish_lidar_object_lists and self.reference_lidar is not None:
            add_object_list_publishers(self.data_publishers, self.lidar_object_list_topic)
        if self.publish_base_link_object_lists:
            add_object_list_publishers(self.data_publishers, "object_list/base_link")

    @property
    def lidar_object_list_topic(self) -> str:
        """Return the topic the object lists of the reference lidar are published on."""
        return f"object_list/{self.lidar_topics[self.reference_lidar]}"

    def generate_samples(self) -> Iterator[Tuple[int, Dict[str, Any]]]:
        """Yield the samples of every selected scene as ROS messages."""
        sample_index = 0
        for scene_index, scene in enumerate(self.scenes, start=1):
            if scene_index <= self.start_scene_index:
                continue
            calibration = scene.calibration()
            static_tf = TFMessage(transforms=_static_transforms(scene, calibration))
            scene_lidars = set(scene.load_metadata().get("sensors", {}).get("lidar", []))
            map_from_odom = _map_from_odom(scene)
            camera_infos: Dict[str, CameraInfo] = {}
            track_ids: Dict[str, int] = {}
            frames = scene.frames(sample_filter="any_label" if self.samples == "keyframes" else "all")
            for frame in frames.iter_frames():
                clock = timestamp_micros_to_clock(frame.timestamp_ns // 1_000)
                stamp = clock.clock
                map_from_ego = map_from_odom @ frame.load_ego_pose()
                sample: Dict[str, Any] = {
                    "scene_id": scene.name,
                    "/clock": clock,
                    "/tf_static": static_tf,
                    "/tf": TFMessage(transforms=[_matrix_transform("map", "base_link", map_from_ego, stamp)]),
                }
                if self.publish_ego_data:
                    sample["ego_data"] = _ego_data(frame, map_from_ego, stamp)
                for sensor_id, topic in self.lidars.items():
                    if frame.has_lidar(sensor_id, stage=self.lidar_stage):
                        sample[f"{topic}/point_cloud"] = self._lidar_point_cloud(frame, sensor_id, stamp)
                for sensor_id, topic in self.radars.items():
                    if frame.has_radar(sensor_id):
                        cloud = frame.load_radar(sensor_id)
                        sample[f"{topic}/point_cloud"] = _point_cloud_message(cloud, _frame_id("radar", sensor_id), stamp)
                for sensor_id, topic in self.cameras.items():
                    if not frame.has_camera(sensor_id):
                        continue
                    sample[f"{topic}/image_raw"] = self._image(frame, sensor_id, stamp)
                    if sensor_id not in camera_infos:
                        camera_infos[sensor_id] = self._camera_info(calibration, sensor_id)
                    sample[f"{topic}/camera_info"] = _stamped(camera_infos[sensor_id], stamp)
                # FZI-AURA annotates 2 Hz keyframes only, so a non-keyframe sample is published
                # without the object list topics rather than with an empty object list. An
                # annotated keyframe holding no object does publish an empty one, which states
                # that nothing was annotated in it.
                if frame.has_boxes_3d:
                    if self.publish_lidar_object_lists and self.reference_lidar in scene_lidars:
                        set_object_list_sample(
                            sample,
                            self.lidar_object_list_topic,
                            *_object_list(
                                frame.load_boxes(sensor_id=self.reference_lidar),
                                _frame_id("lidar", self.reference_lidar),
                                stamp,
                                scene.scene_id,
                                track_ids,
                            ),
                        )
                    if self.publish_base_link_object_lists:
                        set_object_list_sample(
                            sample,
                            "object_list/base_link",
                            *_object_list(frame.load_boxes(frame="base_link"), "base_link", stamp, scene.scene_id, track_ids),
                        )

                sample_index += 1
                yield sample_index, sample

    def _select_scenes(self) -> List[FZIAURAScene]:
        """Return the scenes to publish, in the recording order the dataset indexes them in."""
        if not self.scene_ids:
            return list(self.dataset.iter_scenes())
        scenes = []
        for scene_id in self.scene_ids:
            try:
                scenes.append(self.dataset.get_scene(scene_id))
            except Exception as error:
                raise FileNotFoundError(
                    f"FZI-AURA scene '{scene_id}' requested via 'fzi_aura_scenes' is not part of split "
                    f"'{self.split}' of the dataset in '{self.dataset_root_dir}': {error}"
                ) from error
        return sorted(scenes, key=lambda scene: scene.index)

    def _sensor_topics(self, modality: str, sensor_ids: Sequence[str]) -> Dict[str, str]:
        """Return the sensors of a modality held by the selected scenes, mapped to their topic.

        The sensor suite differs between scenes, so the sensors that occur at all are resolved
        from the scene metadata rather than from the sample index, which would have to be read in
        full. A sensor keeps the topic its position in the sensor suite assigns to it, so that the
        same sensor is published on the same topic regardless of the scenes that were selected.

        Args:
            modality: Modality of the sensors, i.e. ``camera``, ``lidar`` or ``radar``.
            sensor_ids: Sensor IDs of the modality, ordered as they are mapped to their topics.
        """
        available: set = set()
        for scene in self.scenes:
            available.update(scene.load_metadata().get("sensors", {}).get(modality, []))
        unknown = available.difference(sensor_ids)
        if unknown:
            _print_once(
                f"FZI-AURA scenes hold the unknown {modality} sensor(s) {', '.join(sorted(unknown))}, "
                f"which are not published; the adapter maps the released sensors to fixed topics."
            )
        return {
            sensor_id: f"{modality}_{index:02d}" for index, sensor_id in enumerate(sensor_ids, start=1) if sensor_id in available
        }

    def _published_sensors(self, modality: str) -> Dict[str, str]:
        """Return the sensors of a modality whose payload layers the download holds."""
        topics = getattr(self, f"{modality}_topics")
        availability_key = _MODALITY_TO_AVAILABILITY[modality].format(stage=self.lidar_stage)
        if topics and not self.dataset.availability.allows_any_sensor(availability_key):
            _print_once(
                f"FZI-AURA '{availability_key}' data is not part of the download in "
                f"'{self.dataset_root_dir}'; re-run 'fzi-aura-download' with its layers to publish it."
            )
            return {}
        return topics

    def _lidar_point_cloud(self, frame: FZIAURAFrame, sensor_id: str, stamp: Time) -> PointCloud2:
        """Read a lidar scan and convert it to a PointCloud2 message in the sensor's frame.

        The semantic labels of a scan are attached as additional point fields, as they are
        annotated per point and have no representation of their own in the ROS interface.
        """
        cloud = frame.load_lidar(sensor_id, stage=self.lidar_stage)
        labels = None
        if self.publish_semantic_labels and frame.has_semantics(sensor_id):
            labels = frame.load_semantics(sensor_id)
            if len(labels) != len(cloud):
                _print_once(
                    f"FZI-AURA semantic labels of '{sensor_id}' do not cover its point cloud "
                    f"({len(labels)} labels for {len(cloud)} points); publishing the cloud without them."
                )
                labels = None
        extra_fields = (
            {"semantic_id": labels.semantic_id.astype("<u2"), "instance_id": labels.instance_id.astype("<u2")}
            if labels is not None
            else {}
        )
        return _point_cloud_message(cloud, _frame_id("lidar", sensor_id), stamp, extra_fields)

    def _image(self, frame: FZIAURAFrame, sensor_id: str, stamp: Time) -> Image:
        """Read a camera image and convert it to a ROS Image message."""
        image = frame.load_camera(sensor_id)
        if self.image_scale != 1.0:
            width, height = self._image_size(image.shape[1], image.shape[0])
            image = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
        return Image(
            header=Header(frame_id=_frame_id("camera", sensor_id), stamp=stamp),
            height=image.shape[0],
            width=image.shape[1],
            encoding="rgb8",
            step=image.shape[1] * 3,
            data=np.ascontiguousarray(image).tobytes(),
        )

    def _camera_info(self, calibration: Any, sensor_id: str) -> CameraInfo:
        """Convert the native camera calibration to a ROS CameraInfo message.

        FZI-AURA releases rectified images together with their projection matrix, so the camera
        matrix is the left 3x3 block of that projection matrix and the distortion coefficients
        are zero. Scaling an image scales the focal lengths and the principal point with it.
        """
        intrinsics = calibration.sensor(f"camera/{sensor_id}").get("intrinsics", {})
        native_width, native_height = int(intrinsics["width"]), int(intrinsics["height"])
        width, height = self._image_size(native_width, native_height)
        projection_matrix = calibration.camera_projection_matrix(sensor_id).copy()
        projection_matrix[0, :] *= width / native_width
        projection_matrix[1, :] *= height / native_height
        message = CameraInfo(header=Header(frame_id=_frame_id("camera", sensor_id)), width=width, height=height)
        message.k = projection_matrix[:, :3].flatten().tolist()
        message.r = np.eye(3).flatten().tolist()
        message.p = projection_matrix.flatten().tolist()
        message.distortion_model = str(intrinsics.get("distortion_model", "plumb_bob"))
        message.d = [0.0] * 5
        return message

    def _image_size(self, native_width: int, native_height: int) -> Tuple[int, int]:
        """Return the published image size for a native image size."""
        return max(1, round(native_width * self.image_scale)), max(1, round(native_height * self.image_scale))


def _split_list(value: str) -> List[str]:
    """Split a comma-separated parameter value into its non-empty entries."""
    return [entry.strip() for entry in value.split(",") if entry.strip()]


def _download(dataset_root: Path, split: str, scene_ids: Sequence[str], layers: Sequence[str]) -> None:
    """Download the selected scenes with the FZI-AURA SDK downloader.

    FZI-AURA is gated on Hugging Face, so the download requires the dataset terms to have been
    accepted and a Hugging Face login. Downloading is delegated to the CLI shipped with the SDK,
    which resolves the archives of the requested layers, downloads them in parallel and extracts
    them into the dataset directory.
    """
    unknown = sorted(set(layers).difference(_DOWNLOAD_LAYERS))
    if unknown:
        raise ValueError(
            f"Unsupported FZI-AURA download layer(s) {', '.join(unknown)}; expected any of: {', '.join(_DOWNLOAD_LAYERS)}"
        )
    dataset_root.mkdir(parents=True, exist_ok=True)
    command = [sys.executable, "-m", "fzi_aura.download", str(dataset_root)]
    if split != "all":
        command.append(f"--splits={split}")
    if scene_ids:
        command.append(f"--scenes={','.join(scene_ids)}")
    if layers:
        command.append(f"--layers={','.join(layers)}")
    if not scene_ids:
        LOGGER.warn(
            f"Downloading the complete '{split}' split of FZI-AURA, which holds hundreds of scenes; "
            f"set the 'fzi_aura_scenes' parameter to download individual scenes instead."
        )
    LOGGER.info(f"Downloading FZI-AURA into '{dataset_root}'")
    result = subprocess.run(command)
    if result.returncode != 0:
        raise RuntimeError(
            f"Downloading FZI-AURA failed with exit code {result.returncode}; accept the dataset terms at "
            f"https://huggingface.co/datasets/fzi-forschungszentrum-informatik/FZI-AURA, log in with "
            f"'hf auth login' and rerun, the downloader keeps the data that is already present."
        )


def _frame_id(modality: str, sensor_id: str) -> str:
    """Return the TF frame a sensor's data is published in.

    The frame is derived from the modality and the sensor ID, because the sensor IDs are only
    unique within a modality: both a lidar and a radar of the sensor suite are called
    ``front_left``.
    """
    return f"{modality}_{sensor_id}"


def _static_transforms(scene: FZIAURAScene, calibration: Any) -> List[TransformStamped]:
    """Build the static transforms from the vehicle frame to every calibrated sensor frame."""
    transforms = []
    for modality, sensor_ids in sorted(scene.load_metadata().get("sensors", {}).items()):
        for sensor_id in sensor_ids:
            sensor_key = f"{modality}/{sensor_id}"
            try:
                base_from_sensor = calibration.base_from_sensor(sensor_key)
            except Exception as error:
                _print_once(f"FZI-AURA sensor '{sensor_key}' is not calibrated and gets no transform: {error}")
                continue
            transforms.append(_matrix_transform("base_link", _frame_id(modality, sensor_id), base_from_sensor))
    return transforms


def _map_from_odom(scene: FZIAURAScene) -> np.ndarray:
    """Return the rotation from a scene's native pose frame into an ENU-aligned map frame.

    The frame the released ego poses are expressed in carries an arbitrary orientation per
    recording, while the INS state in the vehicle signals is an east-north-up attitude of the same
    vehicle frame. Rotating the poses by the offset between the two at the first sample of the
    scene therefore aligns them with east-north-up, up to the drift of the released odometry,
    which stays below about two degrees over a scene. Only the orientation is corrected; the
    origin of the released poses is kept.
    """
    frame = scene[0]
    enu_from_base = _ins_orientation(_vehicle_signals(frame))
    if enu_from_base is None:
        _print_once(
            "FZI-AURA scenes without an INS attitude keep the native orientation of their released "
            "poses, which is neither gravity-aligned nor north-referenced."
        )
        return np.eye(4)
    transform = np.eye(4)
    transform[:3, :3] = enu_from_base @ frame.load_ego_pose()[:3, :3].T
    return transform


def _ins_orientation(signals: Optional[Any]) -> Optional[np.ndarray]:
    """Return the east-north-up attitude of the vehicle as a rotation matrix, if it is recorded."""
    if signals is None:
        return None
    for prefix in ("ins_odom_pose_orientation", "ins_imu_orientation"):
        quaternion = [_signal(signals, f"{prefix}_{axis}") for axis in "xyzw"]
        if np.all(np.isfinite(quaternion)) and np.linalg.norm(quaternion) > 0:
            return Rotation.from_quat(quaternion).as_matrix()
    return None


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


def _stamped(camera_info: CameraInfo, stamp: Time) -> CameraInfo:
    """Return a copy of a camera info message carrying the stamp of the sample it belongs to.

    The calibration is static within a scene, so the message is built once per camera and only
    its header is updated per sample.
    """
    message = CameraInfo(
        header=Header(frame_id=camera_info.header.frame_id, stamp=stamp),
        height=camera_info.height,
        width=camera_info.width,
        distortion_model=camera_info.distortion_model,
        d=camera_info.d,
    )
    message.k = camera_info.k
    message.r = camera_info.r
    message.p = camera_info.p
    return message


def _ego_data(frame: FZIAURAFrame, map_from_ego: np.ndarray, stamp: Time) -> EgoData:
    """Build the EgoData message of a sample from its ego pose and vehicle signals.

    The pose is taken from the dataset's ego poses, the dynamics and the light states from the
    vehicle signals recorded by the INS and the vehicle's CAN bus. Signals are recorded at a
    higher rate than the samples and are joined on the exact sample timestamp the sample index
    references; a scene without vehicle signals is published with its pose alone.
    """
    ego_data = EgoData(header=Header(frame_id="map", stamp=stamp))
    pmu.initialize_state(ego_data.state, EGO.MODEL_ID)
    # FZI-AURA references its sensor calibration to the center of the rear axle at ground level
    ego_data.state.reference_point = ObjectReferencePoint(value=ObjectReferencePoint.REAR_AXLE_GROUND)
    ego_data.state.continuous_state[EGO.X] = float(map_from_ego[0, 3])
    ego_data.state.continuous_state[EGO.Y] = float(map_from_ego[1, 3])
    ego_data.state.continuous_state[EGO.Z] = float(map_from_ego[2, 3])
    roll, pitch, yaw = Rotation.from_matrix(map_from_ego[:3, :3]).as_euler("xyz")
    ego_data.state.continuous_state[EGO.ROLL] = float(roll)
    ego_data.state.continuous_state[EGO.PITCH] = float(pitch)
    ego_data.state.continuous_state[EGO.YAW] = float(yaw)
    ego_data.length, ego_data.width, ego_data.height = _EGO_LENGTH, _EGO_WIDTH, _EGO_HEIGHT
    ego_data.state.discrete_state[EGO.TURN_INDICATOR] = EGO.TURN_INDICATOR_UNKNOWN
    ego_data.state.discrete_state[EGO.BRAKE_LIGHT] = EGO.LIGHT_UNKNOWN
    ego_data.state.discrete_state[EGO.REVERSE_LIGHT] = EGO.LIGHT_UNKNOWN

    signals = _vehicle_signals(frame)
    if signals is None:
        # The velocity of the ego vehicle is the only dynamic state the ego poses themselves can
        # provide, so a scene without vehicle signals falls back to differentiating them.
        velocity = frame.ego_velocity(target_frame="base_link")
        if np.all(np.isfinite(velocity)):
            ego_data.state.continuous_state[EGO.VEL_LON] = float(velocity[0])
            ego_data.state.continuous_state[EGO.VEL_LAT] = float(velocity[1])
            ego_data.state.discrete_state[EGO.STANDSTILL] = int(np.linalg.norm(velocity[:2]) < _STANDSTILL_VELOCITY)
        return ego_data

    velocity_lon = _signal(signals, "velocity_x", _signal(signals, "speed_kph", default=np.nan) / 3.6)
    ego_data.state.continuous_state[EGO.VEL_LON] = velocity_lon
    ego_data.state.continuous_state[EGO.VEL_LAT] = _signal(signals, "ins_odom_twist_linear_y")
    ego_data.state.continuous_state[EGO.ACC_LON] = _signal(signals, "ins_imu_linear_acceleration_x")
    ego_data.state.continuous_state[EGO.ACC_LAT] = _signal(signals, "ins_imu_linear_acceleration_y")
    ego_data.state.continuous_state[EGO.YAW_RATE] = _signal(
        signals, "velocity_angular_z", _signal(signals, "ins_imu_angular_velocity_z", default=np.nan)
    )
    ego_data.state.continuous_state[EGO.STEERING_ANGLE_ACK] = _signal(signals, "steering_angle_rad")
    if np.isfinite(velocity_lon):
        ego_data.state.discrete_state[EGO.STANDSTILL] = int(abs(velocity_lon) < _STANDSTILL_VELOCITY)
    ego_data.state.discrete_state[EGO.TURN_INDICATOR] = _turn_indicator(signals)
    return ego_data


def _vehicle_signals(frame: FZIAURAFrame) -> Optional[Any]:
    """Return the vehicle signals recorded at a sample's timestamp, or None if it has none."""
    try:
        return frame.load_vehicle_signals_row()
    except Exception as error:
        _print_once(
            f"FZI-AURA vehicle signals are not available for scene {frame.scene_id} ({error}); the ego data of "
            f"the affected scenes is published with the state its ego poses provide."
        )
        return None


def _signal(signals: Any, name: str, default: float = np.nan) -> float:
    """Return a vehicle signal as a float, or a default if it is missing or not recorded."""
    if name not in signals.index:
        return float(default)
    value = signals[name]
    try:
        value = float(value)
    except (TypeError, ValueError):
        return float(default)
    return value if np.isfinite(value) else float(default)


def _turn_indicator(signals: Any) -> int:
    """Map the indicator light signals of the vehicle to a perception_msgs turn indicator."""
    left, right = _signal(signals, "indicator_light_left_on"), _signal(signals, "indicator_light_right_on")
    if not np.isfinite(left) or not np.isfinite(right):
        return EGO.TURN_INDICATOR_UNKNOWN
    if left and right:
        return EGO.TURN_INDICATOR_HAZARD
    if left:
        return EGO.TURN_INDICATOR_LEFT
    if right:
        return EGO.TURN_INDICATOR_RIGHT
    return EGO.TURN_INDICATOR_OFF


def _point_cloud_message(
    cloud: PointCloud,
    frame_id: str,
    stamp: Time,
    extra_fields: Optional[Dict[str, np.ndarray]] = None,
) -> PointCloud2:
    """Convert a native PCD point cloud to a ROS PointCloud2 message.

    The fields of the PCD file are published under their native names, so that the differing
    schemas of the sensors reach consumers unchanged and every field keeps the meaning the dataset
    documents for it. ``extra_fields`` holds point-aligned arrays that are appended to the cloud,
    which is how the semantic labels of a scan are published; an entry whose name the cloud
    already holds is left out, as a message cannot carry one name twice.
    """
    fields: List[PointField] = []
    dtype: List[Tuple[str, str]] = []
    arrays: List[np.ndarray] = []
    offset = 0
    schema = {name: (kind, size) for name, kind, size in zip(cloud.schema.fields, cloud.schema.types, cloud.schema.sizes)}
    for name in cloud.field_names:
        point_field = _PCD_TO_POINT_FIELD.get(schema[name])
        if point_field is None:
            _print_once(f"FZI-AURA point field '{name}' has an unsupported type {schema[name]} and is not published.")
            continue
        datatype, numpy_dtype = point_field
        fields.append(PointField(name=name, offset=offset, datatype=datatype, count=1))
        dtype.append((name, numpy_dtype))
        arrays.append(cloud.field(name))
        offset += np.dtype(numpy_dtype).itemsize
    for name, values in (extra_fields or {}).items():
        if name in cloud.field_names:
            _print_once(f"FZI-AURA point clouds holding a '{name}' field of their own keep it instead of the annotated one.")
            continue
        datatype, numpy_dtype = _PCD_TO_POINT_FIELD[("U", values.dtype.itemsize)]
        fields.append(PointField(name=name, offset=offset, datatype=datatype, count=1))
        dtype.append((name, numpy_dtype))
        arrays.append(values)
        offset += np.dtype(numpy_dtype).itemsize

    points = np.empty(len(cloud), dtype=dtype)
    for (name, _), values in zip(dtype, arrays):
        points[name] = values
    return create_cloud(Header(frame_id=frame_id, stamp=stamp), fields, points)


def _object_list(
    boxes: Sequence[Box3D],
    frame_id: str,
    stamp: Time,
    scene_id: str,
    track_ids: Dict[str, int],
) -> Tuple[ObjectList, ObjectListMetaInfo]:
    """Convert FZI-AURA 3D boxes into a ROS ObjectList and its meta information.

    Args:
        boxes: Annotated 3D boxes of the sample, expressed in the frame they are published in.
        frame_id: ROS frame the object list is published in.
        stamp: ROS Time message of the sample.
        scene_id: ID of the dataset scene the objects were annotated in.
        track_ids: Mapping of the dataset's object IDs to object IDs, shared across a scene.
    """
    message = ObjectList(header=Header(frame_id=frame_id, stamp=stamp))
    meta_info_msg = create_object_list_meta_info(message, scene_id)
    for box in boxes:
        obj = Object(id=_track_id(box.object_id, track_ids), existence_probability=1.0)
        pmu.initialize_state(obj.state, HEXAMOTION.MODEL_ID)
        obj.state.continuous_state[HEXAMOTION.X] = float(box.center[0])
        obj.state.continuous_state[HEXAMOTION.Y] = float(box.center[1])
        obj.state.continuous_state[HEXAMOTION.Z] = float(box.center[2])
        roll, pitch, yaw = Rotation.from_quat(box.rotation_xyzw).as_euler("xyz")
        obj.state.continuous_state[HEXAMOTION.ROLL] = float(roll)
        obj.state.continuous_state[HEXAMOTION.PITCH] = float(pitch)
        obj.state.continuous_state[HEXAMOTION.YAW] = float(yaw)
        obj.state.continuous_state[HEXAMOTION.LENGTH] = float(box.size_lwh[0])
        obj.state.continuous_state[HEXAMOTION.WIDTH] = float(box.size_lwh[1])
        obj.state.continuous_state[HEXAMOTION.HEIGHT] = float(box.size_lwh[2])
        obj.state.discrete_state[HEXAMOTION.TURN_INDICATOR] = HEXAMOTION.TURN_INDICATOR_UNKNOWN
        obj.state.discrete_state[HEXAMOTION.BRAKE_LIGHT] = HEXAMOTION.LIGHT_UNKNOWN
        obj.state.discrete_state[HEXAMOTION.REVERSE_LIGHT] = HEXAMOTION.LIGHT_UNKNOWN
        obj.state.classifications = [ObjectClassification(type=_classification(box.category), probability=1.0)]
        info: List[Tuple[str, Any]] = [("original_class", box.category), ("object_id", box.object_id)]
        if box.sensor_id is not None:
            info.append(("sensor_id", box.sensor_id))
        add_object_meta_info(meta_info_msg, obj.id, info)
        message.objects.append(obj)
    return message, meta_info_msg


def _classification(category: str) -> int:
    """Map an FZI-AURA detection class to a ROS ObjectClassification type."""
    if category not in _CLASS_MAPPING:
        _print_once(
            f"FZI-AURA objects of the unknown class '{category}' are published as UNCLASSIFIED; "
            f"their dataset class is preserved in the object list's meta information."
        )
        return ObjectClassification.UNCLASSIFIED
    return _CLASS_MAPPING[category]


def _track_id(object_id: str, track_ids: Dict[str, int]) -> int:
    """Map the ID an object is tracked under within a scene to a consecutive integer ID."""
    return track_ids.setdefault(str(object_id), len(track_ids))


def _print_once(message: str) -> None:
    """Log a message the first time it occurs, to keep the playback log readable."""
    if message not in _PRINTED_MESSAGES:
        _PRINTED_MESSAGES.add(message)
        LOGGER.info(message)
