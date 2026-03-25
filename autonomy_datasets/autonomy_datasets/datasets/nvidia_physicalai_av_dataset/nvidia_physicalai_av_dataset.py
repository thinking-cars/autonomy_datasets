from typing import Any, Dict, Iterator, List, Tuple, Optional

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
import physical_ai_av
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion, TransformStamped, Transform, Vector3
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header
from perception_msgs.msg import ObjectList, Object, ObjectClassification, HEXAMOTION
import perception_msgs_utils as pmu

try:
    import DracoPy
except ImportError:
    DracoPy = None

# Mapping from dataset class names to ROS ObjectClassification types
_CLASS_MAPPING: Dict[str, List[int]] = {
    "Automobile": [ObjectClassification.CAR],
    "Heavy_truck": [ObjectClassification.UTILITY],
    "Bus": [ObjectClassification.BUS],
    "Train_or_tram_car": [ObjectClassification.UTILITY],
    "Trolley_bus": [ObjectClassification.BUS],
    "Other_vehicle": [ObjectClassification.UNKNOWN, ObjectClassification.MOTORCYCLE],
    "Trailer": [ObjectClassification.UTILITY],
    "Person": [ObjectClassification.PEDESTRIAN, ObjectClassification.VRU],
    "Stroller": [ObjectClassification.VRU],
    "Rider": [ObjectClassification.BICYCLE],
    "Animal": [ObjectClassification.ANIMAL],
    "Protruding_object": [ObjectClassification.UNKNOWN],
}


class NvidiaPhysicalAiAvDatasetAdapter:
    """Converts NVIDIA Physical AI AV Dataset to ROS 2 messages."""

    CAMERA_FEATURE_NAME = "camera_front_tele_30fov"
    CAMERA_FRAME_ID = "cam_front_tele_30fov"

    def __init__(
        self,
        split: str = "camera",
    ) -> None:

        self.version = "0.1.0"
        self.release_notes = {
            "0.1.0": "Initial integration into Autonomy.Datasets",
        }

        self.split = split

        self.avdi = physical_ai_av.PhysicalAIAVDatasetInterface()


    def generate_samples(self) -> Iterator[Tuple[int, Dict[str, Any]]]:
        """Generate samples as ROS messages from NVIDIA Physical AI AV Dataset files.

        Yields:
            Tuple of (example_id, example_dict) containing ROS messages for each sample.
        """
        i = -1

        # Build clip selection mask based on split
        if self.split == "camera":
            print("Using all samples with camera images and obstacle labels")
            mask = (
                self.avdi.feature_presence[self.avdi.features.CAMERA.CAMERA_FRONT_TELE_30FOV]
                & self.avdi.feature_presence[self.avdi.features.LABELS.OBSTACLE_OFFLINE]
            )
        elif self.split == "camera_lidar":
            print("Using all samples with camera images, LiDAR point clouds, and obstacle labels")
            mask = (
                self.avdi.feature_presence[self.avdi.features.CAMERA.CAMERA_FRONT_TELE_30FOV]
                & self.avdi.feature_presence[self.avdi.features.LABELS.OBSTACLE_OFFLINE]
                & self.avdi.feature_presence[self.avdi.features.LIDAR.LIDAR_TOP_360FOV]
            )
        elif self.split == "camera_radar":
            print("Using all samples with camera images, radar data, and obstacle labels")
            mask = (
                self.avdi.feature_presence[self.avdi.features.CAMERA.CAMERA_FRONT_TELE_30FOV]
                & self.avdi.feature_presence[self.avdi.features.LABELS.OBSTACLE_OFFLINE]
                & self.avdi.feature_presence[self.avdi.features.RADAR.RADAR_FRONT_CENTER_SRR_0]
            )
        else:
            raise ValueError(
                f"Invalid split: {self.split}. Must be one of 'camera', 'camera_lidar', or 'camera_radar'."
            )

        clip_ids = self.avdi.feature_presence.index[mask]

        for clip_id in clip_ids:
            # Load camera video (SeekVideoReader with .timestamps attribute)
            video = self.avdi.get_clip_feature(
                clip_id, feature=self.avdi.features.CAMERA.CAMERA_FRONT_TELE_30FOV, maybe_stream=True
            )

            # Load camera intrinsics
            intrinsics = self.avdi.get_clip_feature(
                clip_id, feature=self.avdi.features.CALIBRATION.CAMERA_INTRINSICS, maybe_stream=True
            )
            camera_model = intrinsics.camera_models.get(self.CAMERA_FEATURE_NAME)

            # Load sensor extrinsics and build static TF messages
            extrinsics = self.avdi.get_clip_feature(
                clip_id, feature=self.avdi.features.CALIBRATION.SENSOR_EXTRINSICS, maybe_stream=True
            )
            segment_tf_msgs = _build_tf_msgs(extrinsics, self.CAMERA_FEATURE_NAME, self.CAMERA_FRAME_ID)

            # Load obstacle auto-labels (dict of DataFrames from zip)
            labels = self.avdi.get_clip_feature(
                clip_id, feature=self.avdi.features.LABELS.OBSTACLE_OFFLINE, maybe_stream=True
            )
            labels_data = labels["obstacle.offline"]
            label_timestamps = np.sort(labels_data["timestamp_us"].unique())

            # Load LiDAR data if split requires it
            lidar_data = None
            if self.split == "camera_lidar":
                lidar_data = self.avdi.get_clip_feature(
                    clip_id, feature=self.avdi.features.LIDAR.LIDAR_TOP_360FOV, maybe_stream=True
                )

            # Load radar data if split requires it
            radar_data = None
            if self.split == "camera_radar":
                radar_data = self.avdi.get_clip_feature(
                    clip_id, feature=self.avdi.features.RADAR.RADAR_FRONT_CENTER_SRR_0, maybe_stream=True
                )

            # Iterate over label timestamps (typically ~10 Hz)
            for label_ts in label_timestamps:
                stamp_msg = _timestamp_micros_to_stamp(label_ts)

                # Decode nearest camera frame for this label timestamp
                images, _ = video.decode_images_from_timestamps(np.array([label_ts]))
                img_rgb = images[0]  # (H, W, 3) uint8

                i += 1
                sample: Dict[str, Any] = {}
                sample["stamp"] = stamp_msg
                sample["tf"] = segment_tf_msgs

                # Camera image
                sample["image"] = _image_to_ros_msg(img_rgb, stamp_msg, self.CAMERA_FRAME_ID)

                # Camera info
                if camera_model is not None:
                    sample["camera_info"] = _camera_model_to_camera_info_msg(
                        camera_model, stamp_msg, self.CAMERA_FRAME_ID
                    )

                # 3D object list from obstacle labels
                frame_labels = labels_data[labels_data["timestamp_us"] == label_ts]
                sample["object_list_3d"] = _labels_to_object_list(frame_labels, stamp_msg)

                # LiDAR point cloud
                if lidar_data is not None:
                    pc_msg = _get_lidar_point_cloud(lidar_data, label_ts, stamp_msg)
                    if pc_msg is not None:
                        sample["point_cloud"] = pc_msg

                # Radar point cloud
                if radar_data is not None:
                    radar_msg = _get_radar_point_cloud(radar_data, label_ts, stamp_msg)
                    if radar_msg is not None:
                        sample["radar_point_cloud"] = radar_msg

                yield i, sample

            video.close()


def _build_tf_msgs(extrinsics, camera_feature_name: str, camera_frame_id: str) -> List[TransformStamped]:
    """Build static TF messages from sensor extrinsics.

    Args:
        extrinsics: Either a SensorExtrinsics object (with sensor_poses dict of RigidTransforms)
            or a raw DataFrame with sensor names as index and qx/qy/qz/qw/x/y/z columns.
        camera_feature_name: The sensor name for the camera in the extrinsics data.
        camera_frame_id: The ROS frame_id to use for the camera child frame.
    """
    tf_msgs = []

    sensor_frame_map = {
        camera_feature_name: camera_frame_id,
        "lidar_top_360fov": "lidar_top",
        "radar_front_center_srr_0": "radar_front",
    }

    if hasattr(extrinsics, "sensor_poses"):
        # SensorExtrinsics object with RigidTransform values
        for sensor_name, child_frame_id in sensor_frame_map.items():
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
                        translation=Vector3(x=float(translation[0]), y=float(translation[1]), z=float(translation[2])),
                        rotation=Quaternion(x=float(quat[0]), y=float(quat[1]), z=float(quat[2]), w=float(quat[3])),
                    ),
                )
            )
    else:
        # Raw DataFrame fallback
        for sensor_name, child_frame_id in sensor_frame_map.items():
            if sensor_name not in extrinsics.index:
                continue
            row = extrinsics.loc[sensor_name]
            tf_msgs.append(
                TransformStamped(
                    header=Header(frame_id="base_link"),
                    child_frame_id=child_frame_id,
                    transform=Transform(
                        translation=Vector3(x=float(row["x"]), y=float(row["y"]), z=float(row["z"])),
                        rotation=Quaternion(x=float(row["qx"]), y=float(row["qy"]), z=float(row["qz"]), w=float(row["qw"])),
                    ),
                )
            )

    return tf_msgs


def _timestamp_micros_to_stamp(timestamp_micros: int) -> Time:
    """Convert microsecond timestamp to ROS Time message."""
    sec = int(timestamp_micros // 1_000_000)
    nanosec = int((timestamp_micros % 1_000_000) * 1_000)
    return Time(sec=sec, nanosec=nanosec)


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
        f,   0.0, cx,
        0.0, f,   cy,
        0.0, 0.0, 1.0,
    ]
    camera_info_msg.r = [
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    ]
    camera_info_msg.p = [
        f,   0.0, cx,  0.0,
        0.0, f,   cy,  0.0,
        0.0, 0.0, 1.0, 0.0,
    ]
    return camera_info_msg


def _labels_to_object_list(labels_df: pd.DataFrame, stamp_msg: Time) -> ObjectList:
    """Convert obstacle label rows to a ROS ObjectList message."""
    object_list_msg = ObjectList()
    object_list_msg.header.frame_id = "base_link"
    object_list_msg.header.stamp = stamp_msg

    for idx, (_, row) in enumerate(labels_df.iterrows()):
        obj_msg = Object()
        obj_msg.id = idx
        obj_msg.existence_probability = 1.0

        pmu.initialize_state(obj_msg.state, HEXAMOTION.MODEL_ID)

        # Position
        obj_msg.state.continuous_state[HEXAMOTION.X] = float(row["center_x"])
        obj_msg.state.continuous_state[HEXAMOTION.Y] = float(row["center_y"])
        obj_msg.state.continuous_state[HEXAMOTION.Z] = float(row["center_z"])

        # Orientation: extract roll, pitch, yaw from quaternion
        rot = Rotation.from_quat([row["orientation_x"], row["orientation_y"], row["orientation_z"], row["orientation_w"]])
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
        obj_msg.state.classifications = [
            ObjectClassification(type=ct, probability=1.0) for ct in class_types
        ]

        object_list_msg.objects.append(obj_msg)

    return object_list_msg


def _get_lidar_point_cloud(lidar_data, label_ts: int, stamp_msg: Time) -> Optional[PointCloud2]:
    """Extract the LiDAR point cloud closest to the label timestamp.

    Args:
        lidar_data: DataFrame or dict of DataFrames from get_clip_feature for LiDAR.
        label_ts: Label timestamp in microseconds.
        stamp_msg: ROS Time message.

    Returns:
        PointCloud2 message or None if decoding is unavailable.
    """
    if DracoPy is None:
        return None

    # Handle both dict (from zip) and direct DataFrame returns
    if isinstance(lidar_data, dict):
        lidar_df = next(
            (v for v in lidar_data.values() if isinstance(v, pd.DataFrame)), None
        )
        if lidar_df is None:
            return None
    else:
        lidar_df = lidar_data

    if "reference_timestamp" not in lidar_df.columns:
        return None

    # Find the scan with the closest timestamp
    scan_timestamps = lidar_df["reference_timestamp"].values
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
            intensity = np.array(decoded.attributes[0], dtype=np.float32).reshape(-1, 1)
            points = np.hstack([points, intensity])
            fields.append(
                PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1)
            )

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
        radar_df = next(
            (v for v in radar_data.values() if isinstance(v, pd.DataFrame)), None
        )
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
