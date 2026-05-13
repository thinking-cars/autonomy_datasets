# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

import pathlib
from typing import Any, Dict, Iterator, List, Optional, Tuple

import cv2
import numpy as np
import pandas as pd
import perception_msgs_utils as pmu
from autonomy_datasets.datasets.dataset import DatasetAdapter
from autonomy_datasets.datasets.utils import timestamp_micros_to_clock
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from perception_msgs.msg import CAMERA2D, EGO, EgoData, HEXAMOTION, Object, ObjectClassification, ObjectList
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

# Waymo camera_name to ROS topic/frame_id mapping
_WAYMO_CAMERA_NAME_TO_TOPIC = {
    1: "camera_01",
    2: "camera_02",
    3: "camera_03",
    4: "camera_04",
    5: "camera_05",
}

_WAYMO_CAMERA_NAME_TO_FRAME_ID = {
    1: "cam_front",
    2: "cam_front_left",
    3: "cam_front_right",
    4: "cam_side_left",
    5: "cam_side_right",
}


class WaymoOpenDatasetAdapter(DatasetAdapter):
    """Converts Waymo Open Dataset parquet files to ROS 2 messages."""

    def __init__(
        self,
        data_publishers: Dict[str, Any],
        dataset_path: str,
        split: str,
        object_model: str = "HEXAMOTION",
        use_camera: bool = False,
        use_lidar: bool = False,
        lidar_min_points_in_bbox: int = 1,
        lidar_object_list_filter_cam_front: bool = False,
    ) -> None:
        """Initialize the Waymo Open Dataset adapter and configure enabled publishers.

        Args:
            data_publishers: Mapping of topic names to publisher instances.
            dataset_path: Root path to the Waymo Open Dataset parquet files.
            split: Dataset split selector (e.g. training, validation, all, mini variants).
            object_model: Output object representation, either "HEXAMOTION" or "CAMERA2D".
            use_camera: Whether to load and publish camera image and calibration data.
            use_lidar: Whether to load and publish LiDAR point cloud data.
            lidar_min_points_in_bbox: Minimum top-LiDAR points required to keep a 3D box.
            lidar_object_list_filter_cam_front: Whether to keep only 3D boxes visible in front camera.
        """
        super().__init__(
            data_publishers=data_publishers,
            version="0.1.0",
            release_notes={
                "0.1.0": "Initial integration into Autonomy.Datasets",
            },
        )

        self.dataset_path = pathlib.PosixPath(dataset_path)
        self.split = split

        self.object_model = object_model
        self.use_camera = use_camera
        self.use_lidar = use_lidar

        self.lidar_min_points_in_bbox = lidar_min_points_in_bbox
        self.lidar_object_list_filter_cam_front = lidar_object_list_filter_cam_front

        # add publishers for outgoing messages, actual publisher will be created in AutonomyDatasets node
        self.data_publishers["ego_data"] = None
        if self.object_model == "CAMERA2D":
            self.data_publishers["object_list_2d"] = None
        elif self.object_model == "HEXAMOTION":
            self.data_publishers["object_list_3d"] = None
        else:
            raise ValueError(f"Unsupported object model: {self.object_model}")
        if self.use_camera:
            self.data_publishers["camera_01/image_raw"] = None
            self.data_publishers["camera_01/camera_info"] = None
            self.data_publishers["camera_02/image_raw"] = None
            self.data_publishers["camera_02/camera_info"] = None
            self.data_publishers["camera_03/image_raw"] = None
            self.data_publishers["camera_03/camera_info"] = None
            self.data_publishers["camera_04/image_raw"] = None
            self.data_publishers["camera_04/camera_info"] = None
            self.data_publishers["camera_05/image_raw"] = None
            self.data_publishers["camera_05/camera_info"] = None
        if self.use_lidar:
            self.data_publishers["lidar_01/point_cloud"] = None

    def generate_samples(self) -> Iterator[Tuple[int, Dict[str, Any]]]:
        """Generate samples as ROS messages from Waymo Open Dataset parquet files.

        Yields:
            Tuple of (example_id, example_dict) containing ROS messages for each sample.
        """
        i = -1

        files = _load_files(self.dataset_path, self.split, self.use_lidar, self.use_camera)

        for (
            lidar_file,
            lidar_calibration_file,
            lidar_box_file,
            camera_file,
            camera_box_file,
            camera_calibration_file,
            vehicle_pose_file,
        ) in zip(*files):
            # load all relevant data from current files into pandas dataframes
            (
                lidar_objects_pandas,
                lidar_pandas,
                lidar_calibration_pandas,
                camera_pandas,
                camera_objects_pandas,
                camera_calibration_pandas,
                vehicle_pose_pandas,
            ) = _load_pandas_data(
                lidar_box_file,
                self.lidar_min_points_in_bbox,
                lidar_file,
                lidar_calibration_file,
                camera_file,
                camera_box_file,
                camera_calibration_file,
                vehicle_pose_file,
            )

            # iterate over all segments in the current files
            for segment_context_key in lidar_objects_pandas["key.segment_context_name"].unique():
                (
                    segment_lidar_objects,
                    segment_lidar_range_images,
                    segment_beam_inclinations,
                    segment_camera_images,
                    segment_camera_objects,
                    segment_camera_calibrations_dict,
                    segment_camera_extrinsic_inv,
                    segment_camera_intrinsic,
                    segment_tf_msgs,
                    segment_vehicle_poses,
                ) = _get_segment_data(
                    lidar_objects_pandas,
                    lidar_pandas,
                    lidar_calibration_pandas,
                    camera_pandas,
                    camera_objects_pandas,
                    camera_calibration_pandas,
                    vehicle_pose_pandas,
                    segment_context_key,
                )

                # Build a lookup of vehicle pose 4x4 matrices by timestamp
                pose_by_timestamp = {}
                if segment_vehicle_poses is not None:
                    for _, pose_row in segment_vehicle_poses.iterrows():
                        ts = pose_row["key.frame_timestamp_micros"]
                        pose_by_timestamp[ts] = np.array(pose_row["[VehiclePoseComponent].world_from_vehicle.transform"]).reshape(
                            4, 4
                        )

                # iterate over all frames in the current segment (identified by unique timestamps)
                prev_pose = None
                prev_timestamp = None
                for timestamp_key in segment_lidar_objects["key.frame_timestamp_micros"].unique():
                    clock_msg = timestamp_micros_to_clock(timestamp_key)

                    # 3D Lidar Object List in Vehicle Frame #
                    if self.object_model == "HEXAMOTION":
                        lidar_objects = segment_lidar_objects[
                            segment_lidar_objects["key.frame_timestamp_micros"] == timestamp_key
                        ]

                        # keep only objects visible in front camera, if filter specified
                        if len(lidar_objects) > 0 and self.lidar_object_list_filter_cam_front:
                            front_camera_calibration = segment_camera_calibrations_dict.get(1)
                            lidar_objects = _filter_objects_by_visibility(
                                lidar_objects,
                                front_camera_calibration,
                                segment_camera_extrinsic_inv,
                                segment_camera_intrinsic,
                            )

                        object_list_3d_msg = _lidar_object_list_to_ros_msg(lidar_objects, clock_msg.clock)

                    else:
                        object_list_3d_msg = None

                    # 2D Camera Object List in Image Frame #
                    if self.object_model == "CAMERA2D":
                        if segment_camera_objects is None:
                            raise ValueError(
                                "Camera object data is required for generating 2D object list. "
                                "Please provide camera box files and set use_camera=True."
                            )
                        camera_objects = segment_camera_objects[
                            segment_camera_objects["key.frame_timestamp_micros"] == timestamp_key
                        ]

                        object_list_2d_msg = _camera_object_list_to_ros_msg(camera_objects, clock_msg.clock)

                    else:
                        object_list_2d_msg = None

                    # Lidar Point Cloud #
                    if segment_lidar_range_images is not None and segment_beam_inclinations is not None:
                        range_image = segment_lidar_range_images[
                            segment_lidar_range_images["key.frame_timestamp_micros"] == timestamp_key
                        ].iloc[0]
                        range_values = range_image["[LiDARComponent].range_image_return1.values"]
                        range_shape = range_image["[LiDARComponent].range_image_return1.shape"]
                        point_cloud = _convert_range_image_to_point_cloud(range_values, range_shape, segment_beam_inclinations)
                        point_cloud_msg = _point_cloud_to_ros_msg(point_cloud, clock_msg.clock)

                    else:
                        point_cloud_msg = None

                    # Camera Images and Camera Info for all cameras #
                    camera_msgs = {}  # topic -> (image_msg, camera_info_msg)
                    if segment_camera_images is not None:
                        frame_camera_rows = segment_camera_images[
                            segment_camera_images["key.frame_timestamp_micros"] == timestamp_key
                        ]
                        for _, cam_row in frame_camera_rows.iterrows():
                            cam_name = cam_row["key.camera_name"]
                            topic = _WAYMO_CAMERA_NAME_TO_TOPIC.get(cam_name)
                            frame_id = _WAYMO_CAMERA_NAME_TO_FRAME_ID.get(cam_name)
                            if topic is None or frame_id is None:
                                continue
                            img_msg = _jpeg_bytes_to_ros_msg(cam_row["[CameraImageComponent].image"], clock_msg.clock, frame_id)
                            cam_calib = segment_camera_calibrations_dict.get(cam_name)
                            info_msg = None
                            if cam_calib is not None:
                                info_msg = _camera_calibration_to_camera_info_msg(cam_calib, clock_msg.clock, frame_id)
                            camera_msgs[topic] = (img_msg, info_msg)

                    # Ego Data and TF from vehicle pose
                    current_pose = pose_by_timestamp.get(timestamp_key)
                    if current_pose is not None:
                        # Compute velocity from consecutive poses via finite difference
                        velocity = None
                        if prev_pose is not None and prev_timestamp is not None:
                            dt = (timestamp_key - prev_timestamp) / 1e6  # seconds
                            if dt > 0:
                                velocity = (current_pose[:3, 3] - prev_pose[:3, 3]) / dt
                        prev_pose = current_pose
                        prev_timestamp = timestamp_key
                        ego_data_msg, tf_msg = _egomotion_to_ego_data(current_pose, clock_msg.clock, velocity)
                    else:
                        ego_data_msg = EgoData()
                        ego_data_msg.header.stamp = clock_msg.clock
                        tf_msg = TFMessage()

                    i += 1
                    sample = {}
                    sample["scene_id"] = segment_context_key
                    sample["/clock"] = clock_msg
                    sample["ego_data"] = ego_data_msg
                    sample["/tf"] = tf_msg
                    sample["/tf_static"] = TFMessage(transforms=segment_tf_msgs)
                    if object_list_2d_msg is not None:
                        sample["object_list_2d"] = object_list_2d_msg
                    if object_list_3d_msg is not None:
                        sample["object_list_3d"] = object_list_3d_msg
                    if point_cloud_msg is not None:
                        sample["lidar_01/point_cloud"] = point_cloud_msg
                    for topic, (img_msg, info_msg) in camera_msgs.items():
                        if img_msg is not None:
                            sample[f"{topic}/image_raw"] = img_msg
                        if info_msg is not None:
                            sample[f"{topic}/camera_info"] = info_msg
                    yield (i, sample)


def _load_files(
    dataset_root_dir: pathlib.PosixPath, split: str, use_lidar: bool, use_camera: bool
) -> Tuple[List, List, List, List, List, List, List]:
    """Load all necessary files for the selected split and data."""

    if split == "all":
        split_paths = [dataset_root_dir / "training", dataset_root_dir / "validation"]
    elif "training" in split:
        split_paths = [dataset_root_dir / "training"]
    elif "validation" in split:
        split_paths = [dataset_root_dir / "validation"]
    else:
        raise ValueError(f"Unknown split: {split}")

    lidar_box_files = []
    lidar_files = []
    lidar_calibration_files = []
    camera_files = []
    camera_box_files = []
    camera_calibration_files = []
    vehicle_pose_files = []

    for split_path in split_paths:
        # lidar boxes are always required for generating samples
        split_lidar_box_files = sorted((split_path / "lidar_box").glob("*.parquet"))
        if "mini" in split:
            split_lidar_box_files = split_lidar_box_files[:1]
        lidar_box_files.extend(split_lidar_box_files)

        # raw lidar data and calibrations, if requested
        if use_lidar:
            split_lidar_files = sorted((split_path / "lidar").glob("*.parquet"))
            split_lidar_calibration_files = sorted((split_path / "lidar_calibration").glob("*.parquet"))
            if "mini" in split:
                split_lidar_files = split_lidar_files[:1]
                split_lidar_calibration_files = split_lidar_calibration_files[:1]
            lidar_files.extend(split_lidar_files)
            lidar_calibration_files.extend(split_lidar_calibration_files)
        else:
            lidar_files.extend([None] * len(split_lidar_box_files))
            lidar_calibration_files.extend([None] * len(split_lidar_box_files))

        # raw camera data, camera boxes and calibrations, if requested
        if use_camera:
            split_camera_files = sorted((split_path / "camera_image").glob("*.parquet"))
            split_camera_box_files = sorted((split_path / "camera_box").glob("*.parquet"))
            split_camera_calibration_files = sorted((split_path / "camera_calibration").glob("*.parquet"))
            if "mini" in split:
                split_camera_files = split_camera_files[:1]
                split_camera_box_files = split_camera_box_files[:1]
                split_camera_calibration_files = split_camera_calibration_files[:1]
            camera_files.extend(split_camera_files)
            camera_box_files.extend(split_camera_box_files)
            camera_calibration_files.extend(split_camera_calibration_files)
        else:
            camera_files.extend([None] * len(split_lidar_box_files))
            camera_box_files.extend([None] * len(split_lidar_box_files))
            camera_calibration_files.extend([None] * len(split_lidar_box_files))

        # vehicle pose is always required for ego data
        split_vehicle_pose_files = sorted((split_path / "vehicle_pose").glob("*.parquet"))
        if "mini" in split:
            split_vehicle_pose_files = split_vehicle_pose_files[:1]
        vehicle_pose_files.extend(split_vehicle_pose_files)

    return (
        lidar_files,
        lidar_calibration_files,
        lidar_box_files,
        camera_files,
        camera_box_files,
        camera_calibration_files,
        vehicle_pose_files,
    )


def _load_pandas_data(
    lidar_box_file: pathlib.PosixPath,
    lidar_box_min_points_in_bbox: int,
    lidar_file: pathlib.PosixPath,
    lidar_calibration_file: pathlib.PosixPath,
    camera_file: pathlib.PosixPath,
    camera_box_file: pathlib.PosixPath,
    camera_calibration_file: pathlib.PosixPath,
    vehicle_pose_file: pathlib.PosixPath,
):
    # load lidar boxes
    lidar_objects_pandas = pd.read_parquet(
        lidar_box_file,
        engine="auto",
        columns=[
            "key.segment_context_name",
            "key.frame_timestamp_micros",
            "[LiDARBoxComponent].type",
            "[LiDARBoxComponent].box.center.x",
            "[LiDARBoxComponent].box.center.y",
            "[LiDARBoxComponent].box.center.z",
            "[LiDARBoxComponent].box.heading",
            "[LiDARBoxComponent].box.size.x",
            "[LiDARBoxComponent].box.size.y",
            "[LiDARBoxComponent].box.size.z",
            "[LiDARBoxComponent].num_top_lidar_points_in_box",
            "[LiDARBoxComponent].difficulty_level.detection",
        ],
    )
    # drop all objects not covered by top lidar with at least min_lidar_points_in_bbox
    lidar_objects_pandas = lidar_objects_pandas[
        lidar_objects_pandas["[LiDARBoxComponent].num_top_lidar_points_in_box"] >= lidar_box_min_points_in_bbox
    ].copy()

    # load raw sensor data and calibrations from TOP lidar, if requested
    if lidar_file is not None and lidar_calibration_file is not None:
        lidar_pandas = pd.read_parquet(
            lidar_file,
            engine="auto",
            columns=[
                "key.laser_name",
                "key.segment_context_name",
                "key.frame_timestamp_micros",
                "[LiDARComponent].range_image_return1.values",
                "[LiDARComponent].range_image_return1.shape",
            ],
        )
        lidar_calibration_pandas = pd.read_parquet(
            lidar_calibration_file,
            engine="auto",
            columns=[
                "key.laser_name",
                "key.segment_context_name",
                "[LiDARCalibrationComponent].extrinsic.transform",
                "[LiDARCalibrationComponent].beam_inclination.values",
            ],
        )
        # use only data from TOP lidar (laser_name == 1)
        lidar_pandas = lidar_pandas[lidar_pandas["key.laser_name"] == 1].copy()
        lidar_calibration_pandas = lidar_calibration_pandas[lidar_calibration_pandas["key.laser_name"] == 1].copy()
    else:
        lidar_pandas = None
        lidar_calibration_pandas = None

    # load camera data and calibrations for all cameras, if requested
    if camera_file is not None and camera_calibration_file is not None:
        camera_pandas = pd.read_parquet(
            camera_file,
            engine="auto",
            columns=[
                "key.camera_name",
                "key.segment_context_name",
                "key.frame_timestamp_micros",
                "[CameraImageComponent].image",
            ],
        )
        camera_objects_pandas = pd.read_parquet(
            camera_box_file,
            engine="auto",
            columns=[
                "key.camera_name",
                "key.segment_context_name",
                "key.frame_timestamp_micros",
                "[CameraBoxComponent].type",
                "[CameraBoxComponent].box.center.x",
                "[CameraBoxComponent].box.center.y",
                "[CameraBoxComponent].box.size.x",
                "[CameraBoxComponent].box.size.y",
                "[CameraBoxComponent].difficulty_level.detection",
            ],
        )
        camera_calibration_pandas = pd.read_parquet(
            camera_calibration_file,
            engine="auto",
            columns=[
                "key.camera_name",
                "key.segment_context_name",
                "[CameraCalibrationComponent].extrinsic.transform",
                "[CameraCalibrationComponent].intrinsic.f_u",
                "[CameraCalibrationComponent].intrinsic.f_v",
                "[CameraCalibrationComponent].intrinsic.c_u",
                "[CameraCalibrationComponent].intrinsic.c_v",
                "[CameraCalibrationComponent].width",
                "[CameraCalibrationComponent].height",
            ],
        )

    else:
        camera_pandas = None
        camera_calibration_pandas = None
        camera_objects_pandas = None

    # load vehicle pose data
    if vehicle_pose_file is not None:
        vehicle_pose_pandas = pd.read_parquet(
            vehicle_pose_file,
            engine="auto",
            columns=[
                "key.segment_context_name",
                "key.frame_timestamp_micros",
                "[VehiclePoseComponent].world_from_vehicle.transform",
            ],
        )
    else:
        vehicle_pose_pandas = None

    return (
        lidar_objects_pandas,
        lidar_pandas,
        lidar_calibration_pandas,
        camera_pandas,
        camera_objects_pandas,
        camera_calibration_pandas,
        vehicle_pose_pandas,
    )


def _get_segment_data(
    lidar_objects_pandas: pd.DataFrame,
    lidar_pandas: Optional[pd.DataFrame],
    lidar_calibration_pandas: Optional[pd.DataFrame],
    camera_pandas: Optional[pd.DataFrame],
    camera_objects_pandas: Optional[pd.DataFrame],
    camera_calibration_pandas: Optional[pd.DataFrame],
    vehicle_pose_pandas: Optional[pd.DataFrame],
    segment_context_key: str,
):

    segment_tf_msgs = []  # list to collect static transforms for current segment

    # get lidar objects for current segment
    segment_lidar_objects = lidar_objects_pandas[lidar_objects_pandas["key.segment_context_name"] == segment_context_key]

    # get lidar range images and calibration for current segment, if requested
    if lidar_pandas is not None and lidar_calibration_pandas is not None:
        segment_lidar_range_images = lidar_pandas[lidar_pandas["key.segment_context_name"] == segment_context_key]
        segment_lidar_calibrations = lidar_calibration_pandas[
            lidar_calibration_pandas["key.segment_context_name"] == segment_context_key
        ]
        assert len(segment_lidar_calibrations) == 1, "Expected exactly one calibration per frame"
        segment_lidar_calibration = segment_lidar_calibrations.iloc[0]

        # Pre-compute extrinsic and beam inclinations once per segment
        segment_lidar_extrinsic = np.array(segment_lidar_calibration["[LiDARCalibrationComponent].extrinsic.transform"]).reshape(
            4, 4
        )
        segment_beam_inclinations = np.array(segment_lidar_calibration["[LiDARCalibrationComponent].beam_inclination.values"])

        # Build static transform: base_link -> lidar_top
        segment_tf_msgs.append(
            TransformStamped(
                header=Header(frame_id="base_link"),
                child_frame_id="lidar_top",
                transform=Transform(
                    translation=Vector3(
                        x=float(segment_lidar_extrinsic[0, 3]),
                        y=float(segment_lidar_extrinsic[1, 3]),
                        z=float(segment_lidar_extrinsic[2, 3]),
                    )
                ),
            )
        )
    else:
        segment_lidar_range_images = None
        segment_beam_inclinations = None

    # get camera images, objects and calibration for current segment, if requested
    # segment_camera_calibrations_dict: camera_name -> calibration row
    # segment_camera_extrinsic_inv / segment_camera_intrinsic: kept for front camera (visibility filter)
    segment_camera_calibrations_dict = {}
    if camera_pandas is not None and camera_objects_pandas is not None and camera_calibration_pandas is not None:
        segment_camera_objects = camera_objects_pandas[camera_objects_pandas["key.segment_context_name"] == segment_context_key]
        segment_camera_images = camera_pandas[camera_pandas["key.segment_context_name"] == segment_context_key]
        segment_all_camera_calibrations = camera_calibration_pandas[
            camera_calibration_pandas["key.segment_context_name"] == segment_context_key
        ]

        # Waymo sensor frame is x=front, y=left, z=up; compose with rotation to optical frame
        R_sensor_from_optical = np.array(
            [
                [0, 0, 1],
                [-1, 0, 0],
                [0, -1, 0],
            ],
            dtype=np.float64,
        )

        segment_camera_extrinsic_inv = None
        segment_camera_intrinsic = None

        for _, cam_calib_row in segment_all_camera_calibrations.iterrows():
            cam_name = cam_calib_row["key.camera_name"]
            segment_camera_calibrations_dict[cam_name] = cam_calib_row

            frame_id = _WAYMO_CAMERA_NAME_TO_FRAME_ID.get(cam_name)
            if frame_id is None:
                continue

            cam_extrinsic = np.array(cam_calib_row["[CameraCalibrationComponent].extrinsic.transform"]).reshape(4, 4)

            # Build static transform: base_link -> camera frame (optical convention)
            cam_rotation = cam_extrinsic[:3, :3] @ R_sensor_from_optical
            cam_quat = R.from_matrix(cam_rotation).as_quat(canonical=False)  # [x, y, z, w]
            segment_tf_msgs.append(
                TransformStamped(
                    header=Header(frame_id="base_link"),
                    child_frame_id=frame_id,
                    transform=Transform(
                        translation=Vector3(
                            x=float(cam_extrinsic[0, 3]),
                            y=float(cam_extrinsic[1, 3]),
                            z=float(cam_extrinsic[2, 3]),
                        ),
                        rotation=Quaternion(
                            x=float(cam_quat[0]),
                            y=float(cam_quat[1]),
                            z=float(cam_quat[2]),
                            w=float(cam_quat[3]),
                        ),
                    ),
                )
            )

            # Keep front camera extrinsic inverse and intrinsic for visibility filtering
            if cam_name == 1:
                segment_camera_extrinsic_inv = np.linalg.inv(cam_extrinsic)
                segment_camera_intrinsic = np.array(
                    [
                        [
                            cam_calib_row["[CameraCalibrationComponent].intrinsic.f_u"],
                            0,
                            cam_calib_row["[CameraCalibrationComponent].intrinsic.c_u"],
                        ],
                        [
                            0,
                            cam_calib_row["[CameraCalibrationComponent].intrinsic.f_v"],
                            cam_calib_row["[CameraCalibrationComponent].intrinsic.c_v"],
                        ],
                        [0, 0, 1],
                    ],
                    dtype=np.float32,
                )
    else:
        segment_camera_images = None
        segment_camera_objects = None
        segment_camera_extrinsic_inv = None
        segment_camera_intrinsic = None

    # get vehicle poses for current segment
    if vehicle_pose_pandas is not None:
        segment_vehicle_poses = vehicle_pose_pandas[
            vehicle_pose_pandas["key.segment_context_name"] == segment_context_key
        ].sort_values("key.frame_timestamp_micros")
    else:
        segment_vehicle_poses = None

    return (
        segment_lidar_objects,
        segment_lidar_range_images,
        segment_beam_inclinations,
        segment_camera_images,
        segment_camera_objects,
        segment_camera_calibrations_dict,
        segment_camera_extrinsic_inv,
        segment_camera_intrinsic,
        segment_tf_msgs,
        segment_vehicle_poses,
    )


def _filter_objects_by_visibility(
    lidar_objects,
    segment_camera_calibration,
    segment_camera_extrinsic_inv,
    segment_camera_intrinsic,
):
    if segment_camera_calibration is None or segment_camera_extrinsic_inv is None or segment_camera_intrinsic is None:
        raise ValueError(
            "Camera calibration data is required for filtering objects by camera visibility. "
            "Please provide camera calibration files and set use_camera=True."
        )

    # Get centers of lidar boxes in vehicle frame
    n_objs = len(lidar_objects)
    centers_vehicle = np.empty((n_objs, 3), dtype=np.float32)
    centers_vehicle[:, 0] = lidar_objects["[LiDARBoxComponent].box.center.x"].values
    centers_vehicle[:, 1] = lidar_objects["[LiDARBoxComponent].box.center.y"].values
    centers_vehicle[:, 2] = lidar_objects["[LiDARBoxComponent].box.center.z"].values

    # Transform centers to camera frame for visibility check
    centers_homogeneous = np.empty((n_objs, 4), dtype=np.float32)
    centers_homogeneous[:, :3] = centers_vehicle
    centers_homogeneous[:, 3] = 1
    centers_camera_origin = (segment_camera_extrinsic_inv @ centers_homogeneous.T).T[:, :3]

    # Build camera frame coordinates for projection
    centers_camera_frame = np.empty((n_objs, 3), dtype=np.float32)
    centers_camera_frame[:, 0] = -centers_camera_origin[:, 1]  # cam_x (right) = -y_vehicle
    centers_camera_frame[:, 1] = centers_camera_origin[:, 2]  # cam_y (down) = -z_vehicle
    centers_camera_frame[:, 2] = centers_camera_origin[:, 0]  # cam_z (depth) = x_vehicle

    # Project to image plane
    projected = (segment_camera_intrinsic @ centers_camera_frame.T).T
    projected_2d = projected[:, :2] / projected[:, 2:3]

    # Check visibility
    visibility_mask = (
        (centers_camera_frame[:, 2] > 0)
        & (projected_2d[:, 0] >= 0)
        & (projected_2d[:, 0] < segment_camera_calibration["[CameraCalibrationComponent].width"])
        & (projected_2d[:, 1] >= 0)
        & (projected_2d[:, 1] < segment_camera_calibration["[CameraCalibrationComponent].height"])
    )

    # filter objects by visibility
    lidar_objects = lidar_objects[visibility_mask]

    return lidar_objects


def _lidar_object_list_to_ros_msg(lidar_objects, stamp_msg) -> ObjectList:
    object_list_3d_msg = ObjectList()
    object_list_3d_msg.header.frame_id = "base_link"
    object_list_3d_msg.header.stamp = stamp_msg

    if len(lidar_objects) > 0:
        n_objects = len(lidar_objects)
        lidar_object_list = np.empty((n_objects, 10), dtype=np.float32)
        lidar_object_list[:, 0] = lidar_objects["[LiDARBoxComponent].type"].values
        lidar_object_list[:, 1] = lidar_objects["[LiDARBoxComponent].box.center.x"].values
        lidar_object_list[:, 2] = lidar_objects["[LiDARBoxComponent].box.center.y"].values
        lidar_object_list[:, 3] = lidar_objects["[LiDARBoxComponent].box.center.z"].values
        lidar_object_list[:, 4] = lidar_objects["[LiDARBoxComponent].box.heading"].values
        lidar_object_list[:, 5] = lidar_objects["[LiDARBoxComponent].box.size.x"].values
        lidar_object_list[:, 6] = lidar_objects["[LiDARBoxComponent].box.size.y"].values
        lidar_object_list[:, 7] = lidar_objects["[LiDARBoxComponent].box.size.z"].values
        lidar_object_list[:, 8] = lidar_objects["[LiDARBoxComponent].num_top_lidar_points_in_box"].values
        lidar_object_list[:, 9] = lidar_objects["[LiDARBoxComponent].difficulty_level.detection"].values

        for i, obj in enumerate(lidar_object_list):
            lidar_obj_msg = Object()
            lidar_obj_msg.id = i
            lidar_obj_msg.existence_probability = 1.0

            # fill continuous state with position, orientation and size
            pmu.initialize_state(lidar_obj_msg.state, HEXAMOTION.MODEL_ID)
            lidar_obj_msg.state.continuous_state[HEXAMOTION.X] = obj[1]
            lidar_obj_msg.state.continuous_state[HEXAMOTION.Y] = obj[2]
            lidar_obj_msg.state.continuous_state[HEXAMOTION.Z] = obj[3]
            lidar_obj_msg.state.continuous_state[HEXAMOTION.ROLL] = 0.0  # not provided
            lidar_obj_msg.state.continuous_state[HEXAMOTION.PITCH] = 0.0  # not provided
            lidar_obj_msg.state.continuous_state[HEXAMOTION.YAW] = obj[4]
            lidar_obj_msg.state.continuous_state[HEXAMOTION.LENGTH] = obj[5]
            lidar_obj_msg.state.continuous_state[HEXAMOTION.WIDTH] = obj[6]
            lidar_obj_msg.state.continuous_state[HEXAMOTION.HEIGHT] = obj[7]

            # fill discrete state and append additional attributes at the end
            lidar_obj_msg.state.discrete_state[HEXAMOTION.TURN_INDICATOR] = HEXAMOTION.TURN_INDICATOR_UNKNOWN
            lidar_obj_msg.state.discrete_state[HEXAMOTION.BRAKE_LIGHT] = HEXAMOTION.LIGHT_UNKNOWN
            lidar_obj_msg.state.discrete_state[HEXAMOTION.REVERSE_LIGHT] = HEXAMOTION.LIGHT_UNKNOWN

            # fill object classification
            if obj[0] == 0:  # UNKNOWN
                lidar_obj_msg.state.classifications = [
                    ObjectClassification(
                        type=ObjectClassification.UNKNOWN,
                        probability=1.0,
                    ),
                    ObjectClassification(
                        type=ObjectClassification.ANIMAL,
                        probability=1.0,
                    ),
                ]
            elif obj[0] == 1:  # VEHICLE
                lidar_obj_msg.state.classifications = [
                    ObjectClassification(
                        type=ObjectClassification.CAR,
                        probability=1.0,
                    ),
                    ObjectClassification(
                        type=ObjectClassification.MOTORCYCLE,
                        probability=1.0,
                    ),
                    ObjectClassification(
                        type=ObjectClassification.UTILITY,
                        probability=1.0,
                    ),
                    ObjectClassification(
                        type=ObjectClassification.BUS,
                        probability=1.0,
                    ),
                    ObjectClassification(
                        type=ObjectClassification.MICRO,
                        probability=1.0,
                    ),
                ]
            elif obj[0] == 2:  # PEDESTRIAN
                lidar_obj_msg.state.classifications = [
                    ObjectClassification(
                        type=ObjectClassification.PEDESTRIAN,
                        probability=1.0,
                    ),
                    ObjectClassification(
                        type=ObjectClassification.VRU,
                        probability=1.0,
                    ),
                ]
            elif obj[0] == 3:  # SIGN
                lidar_obj_msg.state.classifications = [
                    ObjectClassification(type=20, probability=1.0)  # TODO: add to perception_msgs
                ]
            elif obj[0] == 4:  # CYCLIST
                lidar_obj_msg.state.classifications = [
                    ObjectClassification(
                        type=ObjectClassification.BICYCLE,
                        probability=1.0,
                    )
                ]
            else:
                raise ValueError(f"Unknown class ID: {obj[0]}")

            # meta info for evaluation
            lidar_obj_msg.meta_info.append(f"original_class:{int(obj[0])}")
            lidar_obj_msg.meta_info.append(f"num_lidar_pts:{int(obj[8])}")
            lidar_obj_msg.meta_info.append(f"difficulty_level:{-1 if np.isnan(obj[9]) else int(obj[9])}")

            object_list_3d_msg.objects.append(lidar_obj_msg)  # type: ignore[attr-defined]

    return object_list_3d_msg


def _camera_object_list_to_ros_msg(camera_objects, stamp_msg) -> ObjectList:
    object_list_2d_msg = ObjectList()
    object_list_2d_msg.header.frame_id = "cam_front"
    object_list_2d_msg.header.stamp = stamp_msg

    if len(camera_objects) > 0:
        # Extract values once and compute bounding boxes
        center_x = camera_objects["[CameraBoxComponent].box.center.x"].values
        center_y = camera_objects["[CameraBoxComponent].box.center.y"].values
        half_size_x = camera_objects["[CameraBoxComponent].box.size.x"].values / 2
        half_size_y = camera_objects["[CameraBoxComponent].box.size.y"].values / 2

        n_objects = len(camera_objects)
        camera_object_list = np.empty((n_objects, 6), dtype=np.float64)
        camera_object_list[:, 0] = camera_objects["[CameraBoxComponent].type"].values
        camera_object_list[:, 1] = center_x - half_size_x
        camera_object_list[:, 2] = center_y - half_size_y
        camera_object_list[:, 3] = center_x + half_size_x
        camera_object_list[:, 4] = center_y + half_size_y
        camera_object_list[:, 5] = camera_objects["[CameraBoxComponent].difficulty_level.detection"].values

        for i, obj in enumerate(camera_object_list):
            camera_obj_msg = Object()
            camera_obj_msg.id = i
            camera_obj_msg.existence_probability = 1.0

            # fill continuous state with position, orientation and size
            pmu.initialize_state(camera_obj_msg.state, CAMERA2D.MODEL_ID)
            camera_obj_msg.state.continuous_state[CAMERA2D.U] = obj[1]
            camera_obj_msg.state.continuous_state[CAMERA2D.V] = obj[2]
            camera_obj_msg.state.continuous_state[CAMERA2D.WIDTH] = obj[3] - obj[1]
            camera_obj_msg.state.continuous_state[CAMERA2D.HEIGHT] = obj[4] - obj[2]

            # fill object classification
            if obj[0] == 0:  # UNKNOWN
                camera_obj_msg.state.classifications = [
                    ObjectClassification(
                        type=ObjectClassification.UNKNOWN,
                        probability=1.0,
                    ),
                    ObjectClassification(
                        type=ObjectClassification.ANIMAL,
                        probability=1.0,
                    ),
                ]
            elif obj[0] == 1:  # VEHICLE
                camera_obj_msg.state.classifications = [
                    ObjectClassification(
                        type=ObjectClassification.MOTORCYCLE,
                        probability=1.0,
                    ),
                    ObjectClassification(
                        type=ObjectClassification.CAR,
                        probability=1.0,
                    ),
                    ObjectClassification(
                        type=ObjectClassification.UTILITY,
                        probability=1.0,
                    ),
                    ObjectClassification(
                        type=ObjectClassification.BUS,
                        probability=1.0,
                    ),
                    ObjectClassification(
                        type=ObjectClassification.MICRO,
                        probability=1.0,
                    ),
                ]
            elif obj[0] == 2:  # PEDESTRIAN
                camera_obj_msg.state.classifications = [
                    ObjectClassification(
                        type=ObjectClassification.PEDESTRIAN,
                        probability=1.0,
                    ),
                    ObjectClassification(
                        type=ObjectClassification.VRU,
                        probability=1.0,
                    ),
                ]
            elif obj[0] == 3:  # SIGN
                camera_obj_msg.state.classifications = [
                    ObjectClassification(type=20, probability=1.0)  # TODO: add to perception_msgs
                ]
            elif obj[0] == 4:  # CYCLIST
                camera_obj_msg.state.classifications = [
                    ObjectClassification(
                        type=ObjectClassification.BICYCLE,
                        probability=1.0,
                    )
                ]
            else:
                raise ValueError(f"Unknown class ID: {obj[0]}")

            # meta info for evaluation
            camera_obj_msg.meta_info.append(f"original_class:{int(obj[0])}")
            camera_obj_msg.meta_info.append(f"difficulty_level:{-1 if np.isnan(obj[5]) else int(obj[5])}")

            object_list_2d_msg.objects.append(camera_obj_msg)  # type: ignore[attr-defined]

    return object_list_2d_msg


def _camera_calibration_to_camera_info_msg(camera_calibration, stamp_msg, frame_id: str) -> CameraInfo:
    f_u = float(camera_calibration["[CameraCalibrationComponent].intrinsic.f_u"])
    f_v = float(camera_calibration["[CameraCalibrationComponent].intrinsic.f_v"])
    c_u = float(camera_calibration["[CameraCalibrationComponent].intrinsic.c_u"])
    c_v = float(camera_calibration["[CameraCalibrationComponent].intrinsic.c_v"])
    width = int(camera_calibration["[CameraCalibrationComponent].width"])
    height = int(camera_calibration["[CameraCalibrationComponent].height"])

    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = frame_id
    camera_info_msg.header.stamp = stamp_msg
    camera_info_msg.width = width
    camera_info_msg.height = height
    # camera_info_msg.distortion_model = "plumb_bob"
    # camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    camera_info_msg.k = [
        f_u,
        0.0,
        c_u,
        0.0,
        f_v,
        c_v,
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
        f_u,
        0.0,
        c_u,
        0.0,
        0.0,
        f_v,
        c_v,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
    ]

    return camera_info_msg


def _convert_range_image_to_point_cloud(range_image_values, range_image_shape, beam_inclinations):
    """Convert range image to point cloud.

    Args:
        range_image_values: Flattened range image values
        range_image_shape: Shape of the range image [H, W, C]
        beam_inclinations: Array of beam inclination values (in radians) for each row

    Returns:
        Point cloud array with shape [N, 5] containing (x, y, z, intensity, elongation)
        in sensor frame with x=front, y=left, z=up
    """
    range_image = np.array(range_image_values).reshape(range_image_shape)

    range_values = range_image[..., 0]
    intensity_values = range_image[..., 1]
    elongation_values = range_image[..., 2]

    valid_mask = range_values > 0
    valid_indices = np.where(valid_mask)
    ranges = range_values[valid_indices]
    intensities = intensity_values[valid_indices]
    elongations = elongation_values[valid_indices]

    _, width = range_image_shape[0], range_image_shape[1]

    inclinations_per_row = np.array(beam_inclinations)[::-1]
    inclination = inclinations_per_row[valid_indices[0]]

    ratios = (width - valid_indices[1] - 0.5) / width
    azimuth = (ratios * 2.0 - 1.0) * np.pi

    cos_azimuth = np.cos(azimuth)
    sin_azimuth = np.sin(azimuth)
    cos_incl = np.cos(inclination)
    sin_incl = np.sin(inclination)

    x = cos_azimuth * cos_incl * ranges
    y = sin_azimuth * cos_incl * ranges
    z = sin_incl * ranges

    point_cloud = np.column_stack([x, y, z, intensities, elongations])

    return point_cloud.astype(np.float32)


def _point_cloud_to_ros_msg(point_cloud, stamp_msg) -> PointCloud2:
    header = Header(frame_id="lidar_top", stamp=stamp_msg)
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(
            name="intensity",
            offset=12,
            datatype=PointField.FLOAT32,
            count=1,
        ),
        PointField(
            name="elongation",
            offset=16,
            datatype=PointField.FLOAT32,
            count=1,
        ),
    ]
    point_cloud_msg = create_cloud(header, fields, point_cloud)

    return point_cloud_msg


def _jpeg_bytes_to_ros_msg(jpeg_bytes, stamp_msg, frame_id: str) -> Image:
    img_array = cv2.imdecode(np.frombuffer(jpeg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
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


def _egomotion_to_ego_data(
    world_from_vehicle: np.ndarray,
    stamp_msg: Time,
    velocity: Optional[np.ndarray] = None,
) -> Tuple[EgoData, TFMessage]:
    """Convert a Waymo vehicle pose (4x4 world_from_vehicle) to EgoData and TF.

    Args:
        world_from_vehicle: 4x4 transformation matrix (world <- vehicle).
        stamp_msg: ROS Time message.
        velocity: Optional (3,) global velocity vector [vx, vy, vz] from finite differencing.

    Returns:
        Tuple of (EgoData, TFMessage).
    """
    ego_data_msg = EgoData()
    ego_data_msg.header.frame_id = "map"
    ego_data_msg.header.stamp = stamp_msg
    pmu.initialize_state(ego_data_msg.state, EGO.MODEL_ID)

    # Position
    x, y, z = world_from_vehicle[:3, 3]
    ego_data_msg.state.continuous_state[EGO.X] = float(x)
    ego_data_msg.state.continuous_state[EGO.Y] = float(y)
    ego_data_msg.state.continuous_state[EGO.Z] = float(z)

    # Orientation: extract roll, pitch, yaw from rotation matrix
    rot = R.from_matrix(world_from_vehicle[:3, :3])
    roll, pitch, yaw = rot.as_euler("xyz")
    ego_data_msg.state.continuous_state[EGO.ROLL] = float(roll)
    ego_data_msg.state.continuous_state[EGO.PITCH] = float(pitch)
    ego_data_msg.state.continuous_state[EGO.YAW] = float(yaw)

    # Dimensions from egomotion data (Jaguar I-PACE)
    ego_data_msg.length = 4.68
    ego_data_msg.width = 1.89
    ego_data_msg.height = 1.56

    # Velocity: transform from global frame to ego-local (longitudinal/lateral)
    if velocity is not None:
        vx, vy = velocity[0], velocity[1]
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        ego_data_msg.state.continuous_state[EGO.VEL_LON] = float(cos_yaw * vx + sin_yaw * vy)
        ego_data_msg.state.continuous_state[EGO.VEL_LAT] = float(-sin_yaw * vx + cos_yaw * vy)

    # Create TFMessage for ego pose in map frame
    quat = rot.as_quat()  # [x, y, z, w]
    tf_msg = TFMessage(
        transforms=[
            TransformStamped(
                header=Header(frame_id="map", stamp=stamp_msg),
                child_frame_id="base_link",
                transform=Transform(
                    translation=Vector3(x=float(x), y=float(y), z=float(z)),
                    rotation=Quaternion(
                        x=float(quat[0]),
                        y=float(quat[1]),
                        z=float(quat[2]),
                        w=float(quat[3]),
                    ),
                ),
            )
        ]
    )

    return ego_data_msg, tf_msg
