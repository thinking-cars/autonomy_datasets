import cv2
import pathlib
from typing import Any, Dict, Iterator, List, Tuple, Optional

import numpy as np
import pandas as pd
from geometry_msgs.msg import TransformStamped, Transform, Vector3
from perception_msgs.msg import (
    ObjectList,
    Object,
    ObjectClassification,
    CAMERA2D,
    HEXAMOTION,
)
import perception_msgs_utils as pmu
from sensor_msgs.msg import PointCloud2, PointField, Image
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header


class WaymoOpenDatasetAdapter:
    """Converts Waymo Open Dataset parquet files to ROS 2 messages."""

    def __init__(
        self,
        dataset_root_dir: str,
        split: str,
        use_lidar: bool = False,
        use_camera: bool = False,
        use_camera_object_list: bool = False,
        use_lidar_object_list: bool = True,
        lidar_min_points_in_bbox: int = 1,
        lidar_object_list_filter_cam_front: bool = False,
    ) -> None:

        self.version = "0.1.0"
        self.release_notes = {
            "0.1.0": "Initial integration into Autonomy.Datasets",
        }

        self.dataset_root_dir = pathlib.PosixPath(dataset_root_dir)
        self.split = split

        self.use_lidar = use_lidar
        self.use_camera = use_camera

        self.use_camera_object_list = use_camera_object_list
        self.use_lidar_object_list = use_lidar_object_list

        self.lidar_min_points_in_bbox = lidar_min_points_in_bbox
        self.lidar_object_list_filter_cam_front = lidar_object_list_filter_cam_front

    def generate_samples(self) -> Iterator[Tuple[int, Dict[str, Any]]]:
        """Generate samples as ROS messages from Waymo Open Dataset parquet files.

        Yields:
            Tuple of (example_id, example_dict) containing ROS messages for each sample.
        """
        i = -1

        files = _load_files(
            self.dataset_root_dir, self.split, self.use_lidar, self.use_camera
        )

        for (
            lidar_file,
            lidar_calibration_file,
            lidar_box_file,
            camera_file,
            camera_box_file,
            camera_calibration_file,
        ) in zip(*files):
            # load all relevant data from current files into pandas dataframes
            (
                lidar_objects_pandas,
                lidar_pandas,
                lidar_calibration_pandas,
                camera_pandas,
                camera_objects_pandas,
                camera_calibration_pandas,
            ) = _load_pandas_data(
                lidar_box_file,
                self.lidar_min_points_in_bbox,
                lidar_file,
                lidar_calibration_file,
                camera_file,
                camera_box_file,
                camera_calibration_file,
            )

            # iterate over all segments in the current files
            for segment_context_key in lidar_objects_pandas[
                "key.segment_context_name"
            ].unique():
                (
                    segment_lidar_objects,
                    segment_lidar_range_images,
                    segment_beam_inclinations,
                    segment_camera_images,
                    segment_camera_objects,
                    segment_camera_calibration,
                    segment_camera_extrinsic_inv,
                    segment_camera_intrinsic,
                    segment_tf_msgs,
                ) = _get_segment_data(
                    lidar_objects_pandas,
                    lidar_pandas,
                    lidar_calibration_pandas,
                    camera_pandas,
                    camera_objects_pandas,
                    camera_calibration_pandas,
                    segment_context_key,
                )

                # iterate over all frames in the current segment (identified by unique timestamps)
                for timestamp_key in segment_lidar_objects[
                    "key.frame_timestamp_micros"
                ].unique():
                    ## 3D Lidar Object List in Vehicle Frame ##
                    if self.use_lidar_object_list:
                        lidar_objects = segment_lidar_objects[
                            segment_lidar_objects["key.frame_timestamp_micros"]
                            == timestamp_key
                        ]

                        # keep only objects visible in front camera, if filter specified
                        if (
                            len(lidar_objects) > 0
                            and self.lidar_object_list_filter_cam_front
                        ):
                            lidar_objects = _filter_objects_by_visibility(
                                lidar_objects,
                                segment_camera_calibration,
                                segment_camera_extrinsic_inv,
                                segment_camera_intrinsic,
                            )

                        object_list_3d_msg = _lidar_object_list_to_ros_msg(
                            lidar_objects
                        )

                    else:
                        object_list_3d_msg = None

                    ## 2D Camera Object List in Image Frame ##
                    if self.use_camera_object_list:
                        if segment_camera_objects is None:
                            raise ValueError(
                                "Camera object data is required for generating 2D object list. Please provide camera box files and set use_camera=True."
                            )
                        camera_objects = segment_camera_objects[
                            segment_camera_objects["key.frame_timestamp_micros"]
                            == timestamp_key
                        ]

                        object_list_2d_msg = _camera_object_list_to_ros_msg(
                            camera_objects
                        )

                    else:
                        object_list_2d_msg = None

                    ## Lidar Point Cloud ##
                    if (
                        segment_lidar_range_images is not None
                        and segment_beam_inclinations is not None
                    ):
                        range_image = segment_lidar_range_images[
                            segment_lidar_range_images["key.frame_timestamp_micros"]
                            == timestamp_key
                        ].iloc[0]
                        range_values = range_image[
                            "[LiDARComponent].range_image_return1.values"
                        ]
                        range_shape = range_image[
                            "[LiDARComponent].range_image_return1.shape"
                        ]
                        point_cloud = _convert_range_image_to_point_cloud(
                            range_values, range_shape, segment_beam_inclinations
                        )
                        point_cloud_msg = _point_cloud_to_ros_msg(point_cloud)

                    else:
                        point_cloud_msg = None

                    ## Camera Image ##
                    if segment_camera_images is not None:
                        image_row = segment_camera_images[
                            segment_camera_images["key.frame_timestamp_micros"]
                            == timestamp_key
                        ].iloc[0]
                        image_msg = _jpeg_bytes_to_ros_msg(
                            image_row["[CameraImageComponent].image"]
                        )

                    else:
                        image_msg = None

                    i += 1
                    sample = {}
                    sample["tf"] = segment_tf_msgs
                    if object_list_2d_msg is not None:
                        sample["object_list_2d"] = object_list_2d_msg
                    if object_list_3d_msg is not None:
                        sample["object_list_3d"] = object_list_3d_msg
                    if point_cloud_msg is not None:
                        sample["point_cloud"] = point_cloud_msg
                    if image_msg is not None:
                        sample["image"] = image_msg
                    yield (i, sample)


def _load_files(
    dataset_root_dir: pathlib.PosixPath, split: str, use_lidar: bool, use_camera: bool
) -> Tuple[List, List, List, List, List, List]:
    """Load all necessary files for the selected split and data."""

    if "training" in split:
        split_path = dataset_root_dir / "training"
    elif "validation" in split:
        split_path = dataset_root_dir / "validation"
    else:
        raise ValueError(f"Unknown split: {split}")

    # lidar boxes are always required for generating samples
    lidar_box_path = split_path / "lidar_box"
    lidar_box_files = sorted(lidar_box_path.glob("*.parquet"))
    if "mini" in split:
        lidar_box_files = lidar_box_files[:1]

    # raw lidar data and calibrations, if requested
    if use_lidar:
        lidar_path = split_path / "lidar"
        lidar_files = sorted(lidar_path.glob("*.parquet"))
        lidar_calibration_path = split_path / "lidar_calibration"
        lidar_calibration_files = sorted(lidar_calibration_path.glob("*.parquet"))
        if "mini" in split:
            lidar_files = lidar_files[:1]
            lidar_calibration_files = lidar_calibration_files[:1]
    else:
        lidar_files = [None] * len(lidar_box_files)
        lidar_calibration_files = [None] * len(lidar_box_files)

    # raw camera data, camera boxes and calibrations, if requested
    if use_camera:
        camera_path = split_path / "camera_image"
        camera_files = sorted(camera_path.glob("*.parquet"))
        camera_box_path = split_path / "camera_box"
        camera_box_files = sorted(camera_box_path.glob("*.parquet"))
        camera_calibration_path = split_path / "camera_calibration"
        camera_calibration_files = sorted(camera_calibration_path.glob("*.parquet"))
        if "mini" in split:
            camera_files = camera_files[:1]
            camera_box_files = camera_box_files[:1]
            camera_calibration_files = camera_calibration_files[:1]
    else:
        camera_files = [None] * len(lidar_box_files)
        camera_box_files = [None] * len(lidar_box_files)
        camera_calibration_files = [None] * len(lidar_box_files)

    return (
        lidar_files,
        lidar_calibration_files,
        lidar_box_files,
        camera_files,
        camera_box_files,
        camera_calibration_files,
    )


def _load_pandas_data(
    lidar_box_file: pathlib.PosixPath,
    lidar_box_min_points_in_bbox: int,
    lidar_file: pathlib.PosixPath,
    lidar_calibration_file: pathlib.PosixPath,
    camera_file: pathlib.PosixPath,
    camera_box_file: pathlib.PosixPath,
    camera_calibration_file: pathlib.PosixPath,
):
    # load lidar boxes
    lidar_objects_pandas = pd.read_parquet(
        lidar_box_file,
        engine="pyarrow",
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
        lidar_objects_pandas["[LiDARBoxComponent].num_top_lidar_points_in_box"]
        >= lidar_box_min_points_in_bbox
    ].copy()

    # load raw sensor data and calibrations from TOP lidar, if requested
    if lidar_file is not None and lidar_calibration_file is not None:
        lidar_pandas = pd.read_parquet(
            lidar_file,
            engine="pyarrow",
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
            engine="pyarrow",
            columns=[
                "key.laser_name",
                "key.segment_context_name",
                "[LiDARCalibrationComponent].extrinsic.transform",
                "[LiDARCalibrationComponent].beam_inclination.values",
            ],
        )
        # use only data from TOP lidar (laser_name == 1)
        lidar_pandas = lidar_pandas[lidar_pandas["key.laser_name"] == 1].copy()
        lidar_calibration_pandas = lidar_calibration_pandas[
            lidar_calibration_pandas["key.laser_name"] == 1
        ].copy()
    else:
        lidar_pandas = None
        lidar_calibration_pandas = None

    # load camera data and calibrations for FRONT camera, if requested
    if camera_file is not None and camera_calibration_file is not None:
        camera_pandas = pd.read_parquet(
            camera_file,
            engine="pyarrow",
            columns=[
                "key.camera_name",
                "key.segment_context_name",
                "key.frame_timestamp_micros",
                "[CameraImageComponent].image",
            ],
        )
        camera_objects_pandas = pd.read_parquet(
            camera_box_file,
            engine="pyarrow",
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
            engine="pyarrow",
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
        # use only data from FRONT camera (camera_name == 1)
        camera_pandas = camera_pandas[camera_pandas["key.camera_name"] == 1].copy()
        camera_objects_pandas = camera_objects_pandas[
            camera_objects_pandas["key.camera_name"] == 1
        ].copy()
        camera_calibration_pandas = camera_calibration_pandas[
            camera_calibration_pandas["key.camera_name"] == 1
        ].copy()

    else:
        camera_pandas = None
        camera_calibration_pandas = None
        camera_objects_pandas = None

    return (
        lidar_objects_pandas,
        lidar_pandas,
        lidar_calibration_pandas,
        camera_pandas,
        camera_objects_pandas,
        camera_calibration_pandas,
    )


def _get_segment_data(
    lidar_objects_pandas: pd.DataFrame,
    lidar_pandas: Optional[pd.DataFrame],
    lidar_calibration_pandas: Optional[pd.DataFrame],
    camera_pandas: Optional[pd.DataFrame],
    camera_objects_pandas: Optional[pd.DataFrame],
    camera_calibration_pandas: Optional[pd.DataFrame],
    segment_context_key: str,
):

    segment_tf_msgs = []  # list to collect static transforms for current segment

    # get lidar objects for current segment
    segment_lidar_objects = lidar_objects_pandas[
        lidar_objects_pandas["key.segment_context_name"] == segment_context_key
    ]

    # get lidar range images and calibration for current segment, if requested
    if lidar_pandas is not None and lidar_calibration_pandas is not None:
        segment_lidar_range_images = lidar_pandas[
            lidar_pandas["key.segment_context_name"] == segment_context_key
        ]
        segment_lidar_calibrations = lidar_calibration_pandas[
            lidar_calibration_pandas["key.segment_context_name"] == segment_context_key
        ]
        assert len(segment_lidar_calibrations) == 1, (
            "Expected exactly one calibration per frame"
        )
        segment_lidar_calibration = segment_lidar_calibrations.iloc[0]

        # Pre-compute extrinsic and beam inclinations once per segment
        segment_lidar_extrinsic = np.array(
            segment_lidar_calibration["[LiDARCalibrationComponent].extrinsic.transform"]
        ).reshape(4, 4)
        segment_beam_inclinations = np.array(
            segment_lidar_calibration[
                "[LiDARCalibrationComponent].beam_inclination.values"
            ]
        )

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
    if (
        camera_pandas is not None
        and camera_objects_pandas is not None
        and camera_calibration_pandas is not None
    ):
        segment_camera_objects = camera_objects_pandas[
            camera_objects_pandas["key.segment_context_name"] == segment_context_key
        ]
        segment_camera_images = camera_pandas[
            camera_pandas["key.segment_context_name"] == segment_context_key
        ]
        segment_camera_calibrations = camera_calibration_pandas[
            camera_calibration_pandas["key.segment_context_name"] == segment_context_key
        ]
        assert len(segment_camera_calibrations) == 1, (
            "Expected exactly one calibration per frame"
        )
        segment_camera_calibration = segment_camera_calibrations.iloc[0]

        # Get camera extrinsic and compute inverse once
        segment_camera_extrinsic = np.array(
            segment_camera_calibration[
                "[CameraCalibrationComponent].extrinsic.transform"
            ]
        ).reshape(4, 4)
        segment_camera_extrinsic_inv = np.linalg.inv(segment_camera_extrinsic)

        # Build static transform: base_link -> cam_front
        segment_tf_msgs.append(
            TransformStamped(
                header=Header(frame_id="base_link"),
                child_frame_id="cam_front",
                transform=Transform(
                    translation=Vector3(
                        x=float(segment_camera_extrinsic[0, 3]),
                        y=float(segment_camera_extrinsic[1, 3]),
                        z=float(segment_camera_extrinsic[2, 3]),
                    )
                ),
            )
        )

        # Build intrinsic matrix once
        segment_camera_intrinsic = np.array(
            [
                [
                    segment_camera_calibration[
                        "[CameraCalibrationComponent].intrinsic.f_u"
                    ],
                    0,
                    segment_camera_calibration[
                        "[CameraCalibrationComponent].intrinsic.c_u"
                    ],
                ],
                [
                    0,
                    segment_camera_calibration[
                        "[CameraCalibrationComponent].intrinsic.f_v"
                    ],
                    segment_camera_calibration[
                        "[CameraCalibrationComponent].intrinsic.c_v"
                    ],
                ],
                [0, 0, 1],
            ],
            dtype=np.float32,
        )
    else:
        segment_camera_images = None
        segment_camera_objects = None
        segment_camera_calibration = None
        segment_camera_extrinsic_inv = None
        segment_camera_intrinsic = None

    return (
        segment_lidar_objects,
        segment_lidar_range_images,
        segment_beam_inclinations,
        segment_camera_images,
        segment_camera_objects,
        segment_camera_calibration,
        segment_camera_extrinsic_inv,
        segment_camera_intrinsic,
        segment_tf_msgs,
    )


def _filter_objects_by_visibility(
    lidar_objects,
    segment_camera_calibration,
    segment_camera_extrinsic_inv,
    segment_camera_intrinsic,
):
    if (
        segment_camera_calibration is None
        or segment_camera_extrinsic_inv is None
        or segment_camera_intrinsic is None
    ):
        raise ValueError(
            "Camera calibration data is required for filtering objects by camera visibility. Please provide camera calibration files and set use_camera=True."
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
    centers_camera_origin = (segment_camera_extrinsic_inv @ centers_homogeneous.T).T[
        :, :3
    ]

    # Build camera frame coordinates for projection
    centers_camera_frame = np.empty((n_objs, 3), dtype=np.float32)
    centers_camera_frame[:, 0] = -centers_camera_origin[
        :, 1
    ]  # cam_x (right) = -y_vehicle
    centers_camera_frame[:, 1] = centers_camera_origin[
        :, 2
    ]  # cam_y (down) = -z_vehicle
    centers_camera_frame[:, 2] = centers_camera_origin[
        :, 0
    ]  # cam_z (depth) = x_vehicle

    # Project to image plane
    projected = (segment_camera_intrinsic @ centers_camera_frame.T).T
    projected_2d = projected[:, :2] / projected[:, 2:3]

    # Check visibility
    visibility_mask = (
        (centers_camera_frame[:, 2] > 0)
        & (projected_2d[:, 0] >= 0)
        & (
            projected_2d[:, 0]
            < segment_camera_calibration["[CameraCalibrationComponent].width"]
        )
        & (projected_2d[:, 1] >= 0)
        & (
            projected_2d[:, 1]
            < segment_camera_calibration["[CameraCalibrationComponent].height"]
        )
    )

    # filter objects by visibility
    lidar_objects = lidar_objects[visibility_mask]

    return lidar_objects


def _lidar_object_list_to_ros_msg(lidar_objects) -> ObjectList:
    object_list_3d_msg = ObjectList()
    object_list_3d_msg.header.frame_id = "base_link"

    if len(lidar_objects) > 0:
        n_objects = len(lidar_objects)
        lidar_object_list = np.empty((n_objects, 10), dtype=np.float32)
        lidar_object_list[:, 0] = lidar_objects["[LiDARBoxComponent].type"].values
        lidar_object_list[:, 1] = lidar_objects[
            "[LiDARBoxComponent].box.center.x"
        ].values
        lidar_object_list[:, 2] = lidar_objects[
            "[LiDARBoxComponent].box.center.y"
        ].values
        lidar_object_list[:, 3] = lidar_objects[
            "[LiDARBoxComponent].box.center.z"
        ].values
        lidar_object_list[:, 4] = lidar_objects[
            "[LiDARBoxComponent].box.heading"
        ].values
        lidar_object_list[:, 5] = lidar_objects["[LiDARBoxComponent].box.size.x"].values
        lidar_object_list[:, 6] = lidar_objects["[LiDARBoxComponent].box.size.y"].values
        lidar_object_list[:, 7] = lidar_objects["[LiDARBoxComponent].box.size.z"].values
        lidar_object_list[:, 8] = lidar_objects[
            "[LiDARBoxComponent].num_top_lidar_points_in_box"
        ].values
        lidar_object_list[:, 9] = lidar_objects[
            "[LiDARBoxComponent].difficulty_level.detection"
        ].values

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
            lidar_obj_msg.state.discrete_state[HEXAMOTION.TURN_INDICATOR] = (
                HEXAMOTION.TURN_INDICATOR_UNKNOWN
            )
            lidar_obj_msg.state.discrete_state[HEXAMOTION.BRAKE_LIGHT] = (
                HEXAMOTION.LIGHT_UNKNOWN
            )
            lidar_obj_msg.state.discrete_state[HEXAMOTION.REVERSE_LIGHT] = (
                HEXAMOTION.LIGHT_UNKNOWN
            )
            lidar_obj_msg.state.discrete_state.append(int(obj[8]))  # num_points_in_box
            lidar_obj_msg.state.discrete_state.append(
                -1 if np.isnan(obj[9]) else int(obj[9])
            )  # difficulty_level

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
                    ObjectClassification(
                        type=20, probability=1.0
                    )  # TODO: add to perception_msgs
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

            object_list_3d_msg.objects.append(lidar_obj_msg)  # type: ignore[attr-defined]

    return object_list_3d_msg


def _camera_object_list_to_ros_msg(camera_objects) -> ObjectList:
    object_list_2d_msg = ObjectList()
    object_list_2d_msg.header.frame_id = "cam_front"

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
        camera_object_list[:, 5] = camera_objects[
            "[CameraBoxComponent].difficulty_level.detection"
        ].values

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

            # fill discrete state and append additional attributes at the end
            camera_obj_msg.state.discrete_state.append(
                -1 if np.isnan(obj[5]) else int(obj[5])
            )  # difficulty_level

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
                    ObjectClassification(
                        type=20, probability=1.0
                    )  # TODO: add to perception_msgs
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

            object_list_2d_msg.objects.append(camera_obj_msg)  # type: ignore[attr-defined]

    return object_list_2d_msg


def _convert_range_image_to_point_cloud(
    range_image_values, range_image_shape, beam_inclinations
):
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


def _point_cloud_to_ros_msg(point_cloud) -> PointCloud2:
    header = Header(frame_id="lidar_top")
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


def _jpeg_bytes_to_ros_msg(jpeg_bytes) -> Image:
    img_array = cv2.imdecode(
        np.frombuffer(jpeg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR
    )
    img_rgb = cv2.cvtColor(img_array, cv2.COLOR_BGR2RGB)

    image_msg = Image()
    image_msg.header.frame_id = "cam_front"
    image_msg.height = img_rgb.shape[0]
    image_msg.width = img_rgb.shape[1]
    image_msg.encoding = "rgb8"
    image_msg.is_bigendian = False
    image_msg.step = img_rgb.shape[1] * 3
    image_msg.data = img_rgb.tobytes()

    return image_msg
