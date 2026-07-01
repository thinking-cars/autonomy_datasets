# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

from typing import Any, Dict, Iterator, List, Optional, Tuple

import DracoPy
import numpy as np
import pandas as pd
import perception_msgs_utils as pmu
import physical_ai_av
from autonomy_datasets.datasets.dataset import DatasetAdapter
from autonomy_datasets.datasets.utils import timestamp_micros_to_clock
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from perception_msgs.msg import EGO, EgoData, HEXAMOTION, Object, ObjectClassification, ObjectList, ObjectReferencePoint
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

# Mapping from dataset class names to ROS ObjectClassification types
_CLASS_MAPPING: Dict[str, List[int]] = {
    "automobile": [ObjectClassification.CAR],
    "heavy_truck": [ObjectClassification.UTILITY],
    "bus": [ObjectClassification.BUS],
    "train_or_tram_car": [ObjectClassification.UTILITY],
    "trolley_bus": [ObjectClassification.BUS],
    "other_vehicle": [ObjectClassification.UNKNOWN, ObjectClassification.MOTORCYCLE],
    "trailer": [ObjectClassification.UTILITY],
    "person": [ObjectClassification.PEDESTRIAN, ObjectClassification.VRU],
    "stroller": [ObjectClassification.VRU],
    "rider": [ObjectClassification.BICYCLE, ObjectClassification.MOTORCYCLE],
    "animal": [ObjectClassification.ANIMAL],
    "protruding_object": [ObjectClassification.UNKNOWN],
}

_SENSOR_FEATURE_TO_FRAME_ID = {
    "camera_front_tele_30fov": "cam_front_tele_30fov",
    "camera_front_wide_120fov": "cam_front_wide_120fov",
    "camera_cross_left_120fov": "cam_cross_left_120fov",
    "camera_cross_right_120fov": "cam_cross_right_120fov",
    "camera_rear_left_70fov": "cam_rear_left_70fov",
    "camera_rear_right_70fov": "cam_rear_right_70fov",
    "camera_rear_tele_30fov": "cam_rear_tele_30fov",
    "lidar_top_360fov": "lidar_top",
    "radar_front_center_srr_0": "radar_front",
}

_SENSOR_FEATURE_TO_TOPIC = {
    "camera_front_tele_30fov": "camera_01",
    "camera_front_wide_120fov": "camera_02",
    "camera_cross_left_120fov": "camera_03",
    "camera_cross_right_120fov": "camera_04",
    "camera_rear_left_70fov": "camera_05",
    "camera_rear_right_70fov": "camera_06",
    "camera_rear_tele_30fov": "camera_07",
    "lidar_top_360fov": "lidar_01",
    "radar_front_center_srr_0": "radar_01",
}

# Maximum time difference (in microseconds) to consider two modality timestamps as matching
_MAX_TIMESTAMP_DIFF_US = 100_000  # 100 ms

# Clip IDs to skip due to known data issues (e.g. corrupted files, missing labels, etc.)
_SKIPPED_CLIPS = ["5b968bb9-1a47-4030-90db-204a08f149fc"]

_MISSING_META_INFO_WARNING_PRINTED = False


class NvidiaPhysicalAiAvDatasetAdapter(DatasetAdapter):
    """Converts NVIDIA Physical AI AV Dataset to ROS 2 messages."""

    def __init__(
        self,
        data_publishers: Dict[str, Any],
        split: str,
        dataset_root_dir: str,
        publish_ego_data: bool = True,
        publish_camera_images: bool = True,
        publish_lidar_pointclouds: bool = True,
        publish_lidar_object_lists: bool = True,
        publish_radar_pointclouds: bool = True,
        filter_countries: Optional[List[str]] = None,
        start_scene_index: int = 0,
    ) -> None:
        """Initialize the NVIDIA Physical AI AV Dataset adapter.

        Args:
            data_publishers: Dictionary of publishers for output ROS messages.
            split: Dataset split to use ('train', 'val', 'test', or 'all').
            dataset_root_dir: Root directory of the extracted nuScenes dataset.
            publish_ego_data: Whether to include ego vehicle data (default: True).
            publish_camera_images: Whether to include camera images (default: True).
            publish_lidar_pointclouds: Whether to include lidar point clouds (default: True).
            publish_lidar_object_lists: Whether to include lidar object lists (default: True).
            publish_radar_pointclouds: Whether to include radar data (default: True).
            filter_countries: Optional list of country codes to filter clips by.
            start_scene_index: Number of clips to skip before generating samples.
        """
        super().__init__(
            data_publishers=data_publishers,
            version="0.1.0",
            release_notes={
                "0.1.0": "Initial integration into Autonomy.Datasets",
            },
        )

        self.split = split
        self.publish_ego_data = publish_ego_data
        self.publish_camera_images = publish_camera_images
        self.publish_lidar_pointclouds = publish_lidar_pointclouds
        self.publish_lidar_object_lists = publish_lidar_object_lists
        self.publish_radar_pointclouds = publish_radar_pointclouds
        self.filter_countries = filter_countries
        self.start_scene_index = start_scene_index

        self.avdi = physical_ai_av.PhysicalAIAVDatasetInterface(local_dir=dataset_root_dir)

        # add publishers for outgoing messages, actual publisher will be created in AutonomyDatasets node
        if self.publish_ego_data:
            self.data_publishers["ego_data"] = None
        if self.publish_lidar_object_lists:
            self.data_publishers["object_list/lidar_01"] = None
        for topic in _SENSOR_FEATURE_TO_TOPIC.values():
            if self.publish_camera_images:
                if topic.startswith("camera_"):
                    self.data_publishers[f"{topic}/image_raw"] = None
                    self.data_publishers[f"{topic}/camera_info"] = None
            if self.publish_lidar_pointclouds:
                if topic.startswith("lidar_"):
                    self.data_publishers[f"{topic}/point_cloud"] = None
            if self.publish_radar_pointclouds:
                if topic.startswith("radar_"):
                    self.data_publishers[f"{topic}/point_cloud"] = None

    def generate_samples(self) -> Iterator[Tuple[int, Dict[str, Any]]]:
        """Generate samples as ROS messages from NVIDIA Physical AI AV Dataset files.

        Yields:
            Tuple of (example_id, example_dict) containing ROS messages for each sample.
        """
        i = -1

        # using all clips with obstacle and ego motion labels
        labels_feature = self.avdi.features.LABELS  # pyright: ignore[reportAttributeAccessIssue]
        mask = self.avdi.feature_presence[labels_feature.OBSTACLE_OFFLINE]
        mask &= self.avdi.feature_presence[labels_feature.EGOMOTION]
        mask &= self.avdi.feature_presence[labels_feature.EGOMOTION_OFFLINE]

        # Filter clips by selected
        if self.split in ["train", "val", "test"]:
            print(f"Using only clips from '{self.split}' split")
            mask &= self.avdi.clip_index["split"] == self.split
        elif self.split == "all":
            print("Using clips from all splits")
        else:
            raise ValueError(f"Invalid split '{self.split}' specified. Must be one of: all, train, val, test.")

        # Build clip selection mask based on split
        for feature_name in _SENSOR_FEATURE_TO_TOPIC.keys():
            if feature_name.startswith("camera_"):
                if self.publish_camera_images:
                    print("Using only samples with camera images")
                    mask &= self.avdi.feature_presence[
                        getattr(self.avdi.features.CAMERA, feature_name.upper())  # pyright: ignore[reportAttributeAccessIssue]
                    ]
            elif feature_name.startswith("lidar_"):
                if self.publish_lidar_pointclouds:
                    print("Using only samples with lidar point clouds")
                    mask &= self.avdi.feature_presence[
                        getattr(self.avdi.features.LIDAR, feature_name.upper())  # pyright: ignore[reportAttributeAccessIssue]
                    ]
            elif feature_name.startswith("radar_"):
                if self.publish_radar_pointclouds:
                    print("Using only samples with radar data")
                    mask &= self.avdi.feature_presence[
                        getattr(self.avdi.features.RADAR, feature_name.upper())  # pyright: ignore[reportAttributeAccessIssue]
                    ]
            else:
                raise ValueError(f"Unknown sensor feature '{feature_name}' in mapping.")

        # Filter by country if specified
        if self.filter_countries:
            mask &= self.avdi.data_collection["country"].isin(self.filter_countries)
            print(f"Using only clips from countries: {self.filter_countries}")

        # Filter clips using mask
        clip_ids = self.avdi.feature_presence.index[mask]
        print(f"Selected {len(clip_ids)} clips after filtering by split, modalities, and country")

        for clip_idx, clip_id in enumerate(clip_ids):
            if clip_idx < self.start_scene_index:
                print(f"Skipping already stored clip {clip_idx + 1}/{len(clip_ids)}: {clip_id}")
                continue

            print(f"Processing clip {clip_id}...")
            if clip_id in _SKIPPED_CLIPS:
                print(f"Skipping clip {clip_id} due to known issues")
                continue
            # Load camera video (SeekVideoReader with .timestamps attribute)
            clip_camera_videos = {}
            for feature_name in _SENSOR_FEATURE_TO_FRAME_ID.keys():
                if feature_name.startswith("camera_"):
                    clip_camera_videos[feature_name] = self.avdi.get_clip_feature(
                        clip_id,
                        feature=getattr(
                            self.avdi.features.CAMERA, feature_name.upper()  # pyright: ignore[reportAttributeAccessIssue]
                        ),
                        maybe_stream=True,
                    )

            # Load vehicle dimensions
            clip_vehicle_dimensions = self.avdi.get_clip_feature(
                clip_id,
                feature=self.avdi.features.CALIBRATION.VEHICLE_DIMENSIONS,  # pyright: ignore[reportAttributeAccessIssue]
                maybe_stream=True,
            )

            # Load camera intrinsics
            clip_camera_intrinsics = {}
            avdi_camera_intrinsics = self.avdi.get_clip_feature(
                clip_id,
                feature=self.avdi.features.CALIBRATION.CAMERA_INTRINSICS,  # pyright: ignore[reportAttributeAccessIssue]
                maybe_stream=True,
            )
            for feature_name, frame_id in _SENSOR_FEATURE_TO_FRAME_ID.items():
                if feature_name.startswith("camera_"):
                    clip_camera_intrinsics[frame_id] = avdi_camera_intrinsics.camera_models.get(feature_name)

            # Load sensor extrinsics and build static TF messages
            clip_sensor_extrinsics = self.avdi.get_clip_feature(
                clip_id,
                feature=self.avdi.features.CALIBRATION.SENSOR_EXTRINSICS,  # pyright: ignore[reportAttributeAccessIssue]
                maybe_stream=True,
            )
            sensor_tf_msgs = _build_tf_msgs(clip_sensor_extrinsics)

            label_features = self.avdi.features.LABELS  # pyright: ignore[reportAttributeAccessIssue]

            # Load obstacle auto-labels
            clip_obstacles = self.avdi.get_clip_feature(
                clip_id,
                feature=label_features.OBSTACLE_OFFLINE,
                maybe_stream=True,
            )["obstacle.offline"]
            label_timestamps = np.sort(clip_obstacles["timestamp_us"].unique())

            # Load ego data
            clip_ego = self.avdi.get_clip_feature(
                clip_id,
                feature=label_features.EGOMOTION,
                maybe_stream=True,
            )

            # Load lidar data if required
            lidar_data = None
            lidar_timestamps = None
            if self.publish_lidar_pointclouds:
                lidar_data = self.avdi.get_clip_feature(
                    clip_id,
                    feature=self.avdi.features.LIDAR.LIDAR_TOP_360FOV,  # pyright: ignore[reportAttributeAccessIssue]
                    maybe_stream=True,
                )
                lidar_df = _resolve_sensor_df(lidar_data)
                if lidar_df is None:
                    raise ValueError(f"No valid DataFrame found for lidar data in clip {clip_id}")
                lidar_timestamps = np.sort(lidar_df["spin_start_timestamp"].unique())

            # Load radar data if required
            radar_data = None
            radar_timestamps = None
            if self.publish_radar_pointclouds:
                radar_data = self.avdi.get_clip_feature(
                    clip_id,
                    feature=self.avdi.features.RADAR.RADAR_FRONT_CENTER_SRR_0,  # pyright: ignore[reportAttributeAccessIssue]
                    maybe_stream=True,
                )
                radar_df = _resolve_sensor_df(radar_data)
                if radar_df is not None and "timestamp" in radar_df.columns:
                    radar_timestamps = np.sort(radar_df["timestamp"].unique())

            # Determine sample timestamps: intersect camera frames with other required modalities
            camera_timestamps = clip_camera_videos["camera_front_tele_30fov"].timestamps  # microseconds, one per frame
            sample_timestamps = _compute_sample_timestamps(
                camera_timestamps, label_timestamps, lidar_timestamps, radar_timestamps
            )

            # Decode all selected camera frames at once
            if len(sample_timestamps) == 0:
                [video.close() for video in clip_camera_videos.values()]
                continue

            all_images = {}
            for feature_name, video in clip_camera_videos.items():
                all_images[feature_name], _ = video.decode_images_from_timestamps(sample_timestamps)

            for frame_idx, sample_ts in enumerate(sample_timestamps):
                clock_msg = timestamp_micros_to_clock(int(sample_ts))
                img_rgb = {}
                for feature_name, image in all_images.items():
                    if frame_idx >= len(image):
                        raise ValueError(f"Frame index {frame_idx} out of bounds for images with length {len(image)}")
                    img_rgb[feature_name] = image[frame_idx]  # (H, W, 3) uint8

                sample: Dict[str, Any] = {}
                sample["scene_id"] = clip_id
                sample["/clock"] = clock_msg
                sample["/tf_static"] = TFMessage(transforms=sensor_tf_msgs)

                # Camera image
                for feature_name, topic in _SENSOR_FEATURE_TO_TOPIC.items():
                    if feature_name.startswith("camera_") and feature_name in all_images:
                        img_rgb_feature = all_images[feature_name][frame_idx]
                        frame_id = _SENSOR_FEATURE_TO_FRAME_ID[feature_name]
                        sample[f"{topic}/image_raw"] = _image_to_ros_msg(img_rgb_feature, clock_msg.clock, frame_id)
                        sample[f"{topic}/camera_info"] = _camera_model_to_camera_info_msg(
                            clip_camera_intrinsics[frame_id],
                            clock_msg.clock,
                            frame_id,
                        )

                # Ego Data: find the closest ego row by timestamp
                ego_data_msg, tf_msgs = _egomotion_to_ego_data(clip_ego(sample_ts), clip_vehicle_dimensions, clock_msg.clock)
                sample["/tf"] = tf_msgs
                if self.publish_ego_data:
                    sample["ego_data"] = ego_data_msg

                # 3D object list: gather all labels within tolerance of the sample timestamp
                if self.publish_lidar_object_lists:
                    label_diffs = np.abs(clip_obstacles["timestamp_us"].values - sample_ts)
                    frame_labels = clip_obstacles[label_diffs <= _MAX_TIMESTAMP_DIFF_US]
                    sample["object_list/lidar_01"] = _labels_to_object_list(frame_labels, clock_msg.clock, clip_id)

                # Lidar point cloud
                if lidar_data is not None:
                    pc_msg = _get_lidar_point_cloud(lidar_data, int(sample_ts), clock_msg.clock)
                    if pc_msg is not None:
                        sample["lidar_01/point_cloud"] = pc_msg

                # Radar point cloud
                if radar_data is not None:
                    radar_msg = _get_radar_point_cloud(radar_data, int(sample_ts), clock_msg.clock)
                    if radar_msg is not None:
                        sample["radar_01/point_cloud"] = radar_msg

                i += 1
                yield i, sample

            [video.close() for video in clip_camera_videos.values()]


def _resolve_sensor_df(data) -> Optional[pd.DataFrame]:
    """Extract a DataFrame from sensor data (may be a dict of DataFrames)."""
    if isinstance(data, pd.DataFrame):
        return data
    if isinstance(data, dict):
        for v in data.values():
            if isinstance(v, pd.DataFrame) and len(v) > 0:
                return v
    return None


def _compute_sample_timestamps(
    camera_ts: np.ndarray,
    label_ts: np.ndarray,
    lidar_ts: Optional[np.ndarray],
    radar_ts: Optional[np.ndarray],
) -> np.ndarray:
    """Return camera timestamps for which all required modalities have data nearby."""
    if lidar_ts is not None:
        print("Filter samples by lidar timestamps")
        sample_ts = lidar_ts.copy()
    elif radar_ts is not None:
        print("Filter samples by radar timestamps")
        sample_ts = radar_ts.copy()
    else:
        print("Use all camera timestamps")
        sample_ts = camera_ts.copy()

    def _filter_by_modality(cam_ts: np.ndarray, mod_ts: np.ndarray) -> np.ndarray:
        """Keep only cam timestamps that have a modality timestamp within tolerance."""
        indices = np.searchsorted(mod_ts, cam_ts)
        indices = np.clip(indices, 0, len(mod_ts) - 1)
        # Check both the found index and the one before it
        diffs = np.abs(mod_ts[indices] - cam_ts)
        prev_indices = np.clip(indices - 1, 0, len(mod_ts) - 1)
        prev_diffs = np.abs(mod_ts[prev_indices] - cam_ts)
        min_diffs = np.minimum(diffs, prev_diffs)
        return cam_ts[min_diffs <= _MAX_TIMESTAMP_DIFF_US]

    if lidar_ts is not None and len(lidar_ts) > 0:
        sample_ts = _filter_by_modality(sample_ts, lidar_ts)

    if radar_ts is not None and len(radar_ts) > 0:
        sample_ts = _filter_by_modality(sample_ts, radar_ts)

    # Also require that labels exist nearby
    if len(label_ts) > 0:
        sample_ts = _filter_by_modality(sample_ts, label_ts)

    return sample_ts


def _build_tf_msgs(extrinsics) -> List[TransformStamped]:
    """Build static TF messages from sensor extrinsics.

    Args:
        extrinsics: Either a SensorExtrinsics object (with sensor_poses dict of RigidTransforms)
            or a raw DataFrame with sensor names as index and qx/qy/qz/qw/x/y/z columns.
    """
    tf_msgs = []

    if hasattr(extrinsics, "sensor_poses"):
        # SensorExtrinsics object with RigidTransform values
        for sensor_name, child_frame_id in _SENSOR_FEATURE_TO_FRAME_ID.items():
            if sensor_name not in extrinsics.sensor_poses:
                continue
            pose = extrinsics.sensor_poses[sensor_name]
            translation = pose.translation
            quat = pose.rotation.as_quat()  # [x, y, z, w]
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
                            x=float(quat[0]),
                            y=float(quat[1]),
                            z=float(quat[2]),
                            w=float(quat[3]),
                        ),
                    ),
                )
            )
    else:
        # Raw DataFrame fallback
        for sensor_name, child_frame_id in _SENSOR_FEATURE_TO_FRAME_ID.items():
            if sensor_name not in extrinsics.index:
                continue
            row = extrinsics.loc[sensor_name]
            tf_msgs.append(
                TransformStamped(
                    header=Header(frame_id="base_link"),
                    child_frame_id=child_frame_id,
                    transform=Transform(
                        translation=Vector3(x=float(row["x"]), y=float(row["y"]), z=float(row["z"])),
                        rotation=Quaternion(
                            x=float(row["qx"]),
                            y=float(row["qy"]),
                            z=float(row["qz"]),
                            w=float(row["qw"]),
                        ),
                    ),
                )
            )

    return tf_msgs


def _image_to_ros_msg(img_rgb: np.ndarray, stamp_msg: Time, frame_id: str) -> Image:
    """Convert an RGB numpy array (H, W, 3) uint8 to a ROS Image message."""
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


def _camera_model_to_camera_info_msg(camera_model, stamp_msg: Time, frame_id: str) -> CameraInfo:
    """Convert a physical_ai_av CameraModel to a ROS CameraInfo message.

    Uses the f-theta polynomial's linear coefficient as an approximate focal length
    for the pinhole K/P matrices. The actual camera uses an f-theta distortion model.
    """
    cx, cy = camera_model.principal_point
    # Approximate focal length from f-theta forward polynomial (th2r) linear coefficient
    f = float(camera_model.th2r.coef[1])

    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = frame_id
    camera_info_msg.header.stamp = stamp_msg
    camera_info_msg.width = camera_model.width
    camera_info_msg.height = camera_model.height
    camera_info_msg.distortion_model = "ftheta"
    camera_info_msg.d = [float(c) for c in camera_model.th2r.coef]
    camera_info_msg.k = [
        f,
        0.0,
        cx,
        0.0,
        f,
        cy,
        0.0,
        0.0,
        1.0,
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
        f,
        0.0,
        cx,
        0.0,
        0.0,
        f,
        cy,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
    ]
    return camera_info_msg


def _labels_to_object_list(labels_df: pd.DataFrame, stamp_msg: Time, clip_id: str) -> ObjectList:
    """Convert obstacle label rows to a ROS ObjectList message."""
    object_list_msg = ObjectList()
    object_list_msg.header.frame_id = "base_link"
    object_list_msg.header.stamp = stamp_msg

    objects = []
    for _, row in labels_df.iterrows():
        obj_msg = Object()
        obj_msg.id = int(row["track_id"])
        obj_msg.existence_probability = 1.0

        pmu.initialize_state(obj_msg.state, HEXAMOTION.MODEL_ID)

        # Position
        obj_msg.state.continuous_state[HEXAMOTION.X] = float(row["center_x"])
        obj_msg.state.continuous_state[HEXAMOTION.Y] = float(row["center_y"])
        obj_msg.state.continuous_state[HEXAMOTION.Z] = float(row["center_z"])

        # Orientation: extract roll, pitch, yaw from quaternion
        rot = Rotation.from_quat(
            [
                row["orientation_x"],
                row["orientation_y"],
                row["orientation_z"],
                row["orientation_w"],
            ]
        )
        roll, pitch, yaw = rot.as_euler("xyz")
        obj_msg.state.continuous_state[HEXAMOTION.ROLL] = float(roll)
        obj_msg.state.continuous_state[HEXAMOTION.PITCH] = float(pitch)
        obj_msg.state.continuous_state[HEXAMOTION.YAW] = float(yaw)

        # Dimensions
        obj_msg.state.continuous_state[HEXAMOTION.LENGTH] = float(row["size_x"])
        obj_msg.state.continuous_state[HEXAMOTION.WIDTH] = float(row["size_y"])
        obj_msg.state.continuous_state[HEXAMOTION.HEIGHT] = float(row["size_z"])

        # Discrete state
        obj_msg.state.discrete_state[HEXAMOTION.TURN_INDICATOR] = HEXAMOTION.TURN_INDICATOR_UNKNOWN
        obj_msg.state.discrete_state[HEXAMOTION.BRAKE_LIGHT] = HEXAMOTION.LIGHT_UNKNOWN
        obj_msg.state.discrete_state[HEXAMOTION.REVERSE_LIGHT] = HEXAMOTION.LIGHT_UNKNOWN

        # Classification
        class_name = row["label_class"]
        class_types = _CLASS_MAPPING.get(class_name, [ObjectClassification.UNKNOWN])
        obj_msg.state.classifications = [ObjectClassification(type=ct, probability=1.0) for ct in class_types]

        # Meta information for evaluation
        if hasattr(obj_msg, "meta_info"):
            obj_msg.meta_info.append(f"scene_id:{clip_id}")
            obj_msg.meta_info.append(f"original_class:{row['label_class']}")
        else:
            _warn_missing_meta_info_once()

        objects.append(obj_msg)

    object_list_msg.objects = objects
    return object_list_msg


def _warn_missing_meta_info_once() -> None:
    global _MISSING_META_INFO_WARNING_PRINTED

    if not _MISSING_META_INFO_WARNING_PRINTED:
        print("Warning: Object message does not have 'meta_info' field, skipping annotation metadata")
        _MISSING_META_INFO_WARNING_PRINTED = True


def _egomotion_to_ego_data(ego: pd.Series, vehicle_dimensions, stamp_msg: Time) -> Tuple[EgoData, TFMessage]:
    """Convert a single egomotion row to a ROS EgoData message."""
    ego_data_msg = EgoData()
    ego_data_msg.header.frame_id = "map"
    ego_data_msg.header.stamp = stamp_msg
    pmu.initialize_state(ego_data_msg.state, EGO.MODEL_ID)

    # Reference point
    ego_data_msg.state.reference_point = ObjectReferencePoint(
        value=ObjectReferencePoint.REAR_AXLE_GROUND,
        translation_to_geometric_center=Vector3(
            x=float(vehicle_dimensions.rear_axle_to_bbox_center), y=0.0, z=float(vehicle_dimensions.height / 2)
        ),
    )

    # Position
    x, y, z = ego.pose.translation
    ego_data_msg.state.continuous_state[EGO.X] = float(x)
    ego_data_msg.state.continuous_state[EGO.Y] = float(y)
    ego_data_msg.state.continuous_state[EGO.Z] = float(z)

    # Orientation: extract roll, pitch, yaw from quaternion
    rot = ego.pose.rotation
    roll, pitch, yaw = rot.as_euler("xyz")
    ego_data_msg.state.continuous_state[EGO.ROLL] = float(roll)
    ego_data_msg.state.continuous_state[EGO.PITCH] = float(pitch)
    ego_data_msg.state.continuous_state[EGO.YAW] = float(yaw)

    # Velocity: transform from global frame to ego-local (longitudinal/lateral)
    vx, vy, vz = ego.velocity
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    vel_lon = cos_yaw * vx + sin_yaw * vy
    vel_lat = -sin_yaw * vx + cos_yaw * vy
    ego_data_msg.state.continuous_state[EGO.VEL_LON] = float(vel_lon)
    ego_data_msg.state.continuous_state[EGO.VEL_LAT] = float(vel_lat)

    # Dimensions from egomotion data
    ego_data_msg.length = float(vehicle_dimensions.length)
    ego_data_msg.width = float(vehicle_dimensions.width)
    ego_data_msg.height = float(vehicle_dimensions.height)

    # Create TFMessage for ego pose in map frame
    tf_msg = TFMessage(
        transforms=[
            TransformStamped(
                header=Header(frame_id="map", stamp=stamp_msg),
                child_frame_id="base_link",
                transform=Transform(
                    translation=Vector3(
                        x=float(x),
                        y=float(y),
                        z=float(z),
                    ),
                    rotation=Quaternion(
                        x=float(rot.as_quat()[0]),
                        y=float(rot.as_quat()[1]),
                        z=float(rot.as_quat()[2]),
                        w=float(rot.as_quat()[3]),
                    ),
                ),
            )
        ]
    )

    return ego_data_msg, tf_msg


def _get_lidar_point_cloud(lidar_data, label_ts: int, stamp_msg: Time) -> Optional[PointCloud2]:
    """Extract the lidar point cloud closest to the label timestamp.

    Args:
        lidar_data: DataFrame or dict of DataFrames from get_clip_feature for lidar.
        label_ts: Label timestamp in microseconds.
        stamp_msg: ROS Time message.

    Returns:
        PointCloud2 message or None if decoding is unavailable.
    """

    # Handle both dict (from zip) and direct DataFrame returns
    if isinstance(lidar_data, dict):
        lidar_df = next((v for v in lidar_data.values() if isinstance(v, pd.DataFrame)), None)
        if lidar_df is None:
            return None
    else:
        lidar_df = lidar_data

    # Find the scan with the closest timestamp
    scan_timestamps = lidar_df["spin_start_timestamp"].values
    assert isinstance(scan_timestamps, np.ndarray), "scan_timestamps must be a numpy array"
    nearest_idx = np.argmin(np.abs(scan_timestamps - label_ts))
    scan_row = lidar_df.iloc[nearest_idx]

    # Decode Draco-encoded point cloud
    draco_bytes = scan_row["draco_encoded_pointcloud"]
    if isinstance(draco_bytes, bytes):
        decoded = DracoPy.decode(draco_bytes)
        points = np.array(decoded.points, dtype=np.float32).reshape(-1, 3)

        # Build fields: x, y, z (intensity from attributes if available)
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # Try to get intensity from decoded attributes
        if hasattr(decoded, "attributes") and len(decoded.attributes) > 0:
            intensity = np.array(decoded.attributes[1]["data"], dtype=np.float32).reshape(-1, 1)
            points = np.hstack([points, intensity])
            fields.append(PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1))

        header = Header(frame_id="lidar_top", stamp=stamp_msg)
        return create_cloud(header, fields, points)

    return None


def _get_radar_point_cloud(radar_data, label_ts: int, stamp_msg: Time) -> Optional[PointCloud2]:
    """Extract radar detections closest to the label timestamp as a PointCloud2.

    Converts spherical radar measurements (azimuth, elevation, distance)
    to Cartesian coordinates (x, y, z) in the sensor frame.

    Args:
        radar_data: DataFrame or dict of DataFrames from get_clip_feature for radar.
        label_ts: Label timestamp in microseconds.
        stamp_msg: ROS Time message.

    Returns:
        PointCloud2 message or None if no matching data.
    """
    # Handle both dict (from zip) and direct DataFrame returns
    if isinstance(radar_data, dict):
        radar_df = next((v for v in radar_data.values() if isinstance(v, pd.DataFrame)), None)
        if radar_df is None:
            return None
    else:
        radar_df = radar_data

    if "timestamp" not in radar_df.columns:
        return None

    # Find the scan with the closest timestamp
    scan_timestamps = radar_df["timestamp"].unique()
    nearest_ts = scan_timestamps[np.argmin(np.abs(scan_timestamps - label_ts))]
    scan_detections = radar_df[radar_df["timestamp"] == nearest_ts]

    if len(scan_detections) == 0:
        return None

    # Convert spherical to Cartesian coordinates (sensor frame: x=forward, y=left, z=up)
    azimuth = scan_detections["azimuth"].values
    elevation = scan_detections["elevation"].values
    distance = scan_detections["distance"].values
    radial_velocity = scan_detections["radial_velocity"].values
    rcs = scan_detections["rcs"].values

    x = distance * np.cos(elevation) * np.cos(azimuth)
    y = distance * np.cos(elevation) * np.sin(azimuth)
    z = distance * np.sin(elevation)

    point_cloud = np.column_stack([x, y, z, radial_velocity, rcs]).astype(np.float32)

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="radial_velocity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="rcs", offset=16, datatype=PointField.FLOAT32, count=1),
    ]

    header = Header(frame_id="radar_front", stamp=stamp_msg)
    return create_cloud(header, fields, point_cloud)
