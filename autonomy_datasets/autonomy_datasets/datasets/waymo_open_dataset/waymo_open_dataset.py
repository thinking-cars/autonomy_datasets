import cv2
import pathlib
from typing import Any, Dict, Iterator, List, Tuple

import numpy as np
import pandas as pd
from geometry_msgs.msg import TransformStamped
from perception_msgs.msg import ObjectList, Object, ObjectClassification, CAMERA2D, HEXAMOTION
import perception_msgs_utils as pmu
from sensor_msgs.msg import PointField, Image
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header


class WaymoOpenDatasetAdapter:
    """Converts Waymo Open Dataset parquet files to ROS 2 messages.
    """

    def __init__(self, dataset_root_dir: str, split: str,
                 use_lidar: bool = False, use_camera: bool = False, 
                 use_camera_object_list: bool = False, use_lidar_object_list: bool = True,
                 lidar_object_list_filter: List[str] = ["lidar_top"], lidar_min_points_in_bbox: int = 1) -> None:

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

        self.lidar_object_list_filter = lidar_object_list_filter
        self.lidar_min_points_in_bbox = lidar_min_points_in_bbox

    def generate_samples(self) -> Iterator[Tuple[int, Dict[str, Any]]]:
        """Generate samples as ROS messages from Waymo Open Dataset parquet files.

        Yields:
            Tuple of (example_id, example_dict) containing ROS messages for each sample.
        """
        i = -1

        files = _load_files(self.dataset_root_dir, self.split, self.use_lidar, self.use_camera)

        for lidar_file, lidar_calibration_file, lidar_box_file, camera_file, camera_box_file, camera_calibration_file in zip(*files):

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
            if "lidar_top" in self.lidar_object_list_filter:
                # Drop all objects not covered by top lidar with at least min_lidar_points_in_bbox
                lidar_objects_pandas = lidar_objects_pandas[
                    lidar_objects_pandas["[LiDARBoxComponent].num_top_lidar_points_in_box"] >= self.lidar_min_points_in_bbox
                ].copy()

            if self.use_lidar and lidar_file is not None and lidar_calibration_file is not None:
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
                # Filter only TOP lidar
                lidar_pandas = lidar_pandas[lidar_pandas["key.laser_name"] == 1].copy()
                lidar_calibration_pandas = lidar_calibration_pandas[lidar_calibration_pandas["key.laser_name"] == 1].copy()
            else:
                lidar_pandas = None
                lidar_calibration_pandas = None
            
            if self.use_camera and camera_file is not None and camera_calibration_file is not None:
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
                # Filter only FRONT camera (camera_name == 1) upfront
                camera_pandas = camera_pandas[camera_pandas["key.camera_name"] == 1].copy()
                camera_objects_pandas = camera_objects_pandas[camera_objects_pandas["key.camera_name"] == 1].copy()
                camera_calibration_pandas = camera_calibration_pandas[
                    camera_calibration_pandas["key.camera_name"] == 1
                ].copy()

                # Pre-compute half sizes for bounding boxes (vectorized)
                camera_objects_pandas["half_size_x"] = camera_objects_pandas["[CameraBoxComponent].box.size.x"].values / 2
                camera_objects_pandas["half_size_y"] = camera_objects_pandas["[CameraBoxComponent].box.size.y"].values / 2

            else:
                camera_pandas = None
                camera_calibration_pandas = None
                camera_objects_pandas = None
            # collect transform messages
            tf_msgs = []

            for segment_context_key in lidar_objects_pandas["key.segment_context_name"].unique():
                
                segment_mask_obj = lidar_objects_pandas["key.segment_context_name"] == segment_context_key
                lidar_frame_objects = lidar_objects_pandas[segment_mask_obj]

                if lidar_pandas is not None and lidar_calibration_pandas is not None:
                    segment_mask_lidar = lidar_pandas["key.segment_context_name"] == segment_context_key
                    frame_range_images = lidar_pandas[segment_mask_lidar]
                    segment_mask_cal = lidar_calibration_pandas["key.segment_context_name"] == segment_context_key
                    lidar_frame_calibrations = lidar_calibration_pandas[segment_mask_cal]
                    assert len(lidar_frame_calibrations) == 1, "Expected exactly one calibration per frame"
                    lidar_calibration = lidar_frame_calibrations.iloc[0]
                    # Pre-compute extrinsic and beam inclinations once per segment
                    lidar_extrinsic = np.array(lidar_calibration["[LiDARCalibrationComponent].extrinsic.transform"]).reshape(4, 4)
                    beam_inclinations = np.array(lidar_calibration["[LiDARCalibrationComponent].beam_inclination.values"])

                    # Build static transform: base_link -> lidar_top
                    tf_msg = TransformStamped()
                    tf_msg.header.frame_id = "base_link"
                    tf_msg.child_frame_id = "lidar_top"
                    tf_msg.transform.translation.x = float(lidar_extrinsic[0, 3])
                    tf_msg.transform.translation.y = float(lidar_extrinsic[1, 3])
                    tf_msg.transform.translation.z = float(lidar_extrinsic[2, 3])
                    tf_msgs.append(tf_msg)
                else:
                    frame_range_images = None
                    beam_inclinations = None
                
                if camera_pandas is not None and camera_objects_pandas is not None and camera_calibration_pandas is not None:
                    segment_mask_img = camera_pandas["key.segment_context_name"] == segment_context_key
                    frame_camera_objects = camera_objects_pandas[segment_mask_obj]
                    segment_mask_cal = camera_calibration_pandas["key.segment_context_name"] == segment_context_key
                    frame_images = camera_pandas[segment_mask_img]
                    camera_frame_calibrations = camera_calibration_pandas[segment_mask_cal]
                    assert len(camera_frame_calibrations) == 1, "Expected exactly one calibration per frame"
                    camera_calibration = camera_frame_calibrations.iloc[0]

                    # Get camera extrinsic and compute inverse once
                    camera_extrinsic = np.array(camera_calibration["[CameraCalibrationComponent].extrinsic.transform"]).reshape(4, 4)
                    extrinsic_inv = np.linalg.inv(camera_extrinsic)

                    # Build static transform: base_link -> cam_front
                    tf_msg = TransformStamped()
                    tf_msg.header.frame_id = "base_link"
                    tf_msg.child_frame_id = "cam_front"
                    tf_msg.transform.translation.x = float(camera_extrinsic[0, 3])
                    tf_msg.transform.translation.y = float(camera_extrinsic[1, 3])
                    tf_msg.transform.translation.z = float(camera_extrinsic[2, 3])
                    tf_msgs.append(tf_msg)

                    # Build intrinsic matrix once
                    intrinsic = np.array(
                        [
                            [
                                camera_calibration["[CameraCalibrationComponent].intrinsic.f_u"],
                                0,
                                camera_calibration["[CameraCalibrationComponent].intrinsic.c_u"],
                            ],
                            [
                                0,
                                camera_calibration["[CameraCalibrationComponent].intrinsic.f_v"],
                                camera_calibration["[CameraCalibrationComponent].intrinsic.c_v"],
                            ],
                            [0, 0, 1],
                        ],
                        dtype=np.float32,
                    )
                else:
                    frame_images = None
                    frame_camera_objects = None
                    camera_calibration = None
                    extrinsic_inv = None
                    intrinsic = None

                for timestamp_key in lidar_frame_objects["key.frame_timestamp_micros"].unique():
                    
                    ## 3D Object List in Vehicle Frame ##
                    if self.use_lidar_object_list:
                        lidar_objects = lidar_frame_objects[lidar_frame_objects["key.frame_timestamp_micros"] == timestamp_key]

                        # keep only objects visible in front camera, if filter specified
                        if len(lidar_objects) > 0 and "cam_front" in self.lidar_object_list_filter:
                            if camera_calibration is None or extrinsic_inv is None or intrinsic is None:
                                raise ValueError("Camera calibration data is required for filtering objects by camera visibility. Please provide camera calibration files and set use_camera=True.")

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
                            centers_camera_origin = (extrinsic_inv @ centers_homogeneous.T).T[:, :3]

                            # Build camera frame coordinates for projection
                            centers_camera_frame = np.empty((n_objs, 3), dtype=np.float32)
                            centers_camera_frame[:, 0] = -centers_camera_origin[:, 1]  # cam_x (right) = -y_vehicle
                            centers_camera_frame[:, 1] = centers_camera_origin[:, 2]  # cam_y (down) = -z_vehicle
                            centers_camera_frame[:, 2] = centers_camera_origin[:, 0]  # cam_z (depth) = x_vehicle

                            # Project to image plane
                            projected = (intrinsic @ centers_camera_frame.T).T
                            projected_2d = projected[:, :2] / projected[:, 2:3]

                            # Check visibility
                            visibility_mask = (
                                (centers_camera_frame[:, 2] > 0)
                                & (projected_2d[:, 0] >= 0)
                                & (projected_2d[:, 0] < camera_calibration["[CameraCalibrationComponent].width"])
                                & (projected_2d[:, 1] >= 0)
                                & (projected_2d[:, 1] < camera_calibration["[CameraCalibrationComponent].height"])
                            )

                            # filter objects by visibility
                            lidar_objects = lidar_objects[visibility_mask]

                        # convert object list to ROS message
                        object_list_3d_msg = ObjectList()
                        object_list_3d_msg.header.frame_id = "base_link"

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
                                lidar_obj_msg.state.continuous_state[HEXAMOTION.ROLL] = 0.0
                                lidar_obj_msg.state.continuous_state[HEXAMOTION.PITCH] = 0.0
                                lidar_obj_msg.state.continuous_state[HEXAMOTION.YAW] = obj[4]
                                lidar_obj_msg.state.continuous_state[HEXAMOTION.LENGTH] = obj[5]
                                lidar_obj_msg.state.continuous_state[HEXAMOTION.WIDTH] = obj[6]
                                lidar_obj_msg.state.continuous_state[HEXAMOTION.HEIGHT] = obj[7]

                                # fill discrete state and append additional attributes at the end
                                lidar_obj_msg.state.discrete_state[HEXAMOTION.TURN_INDICATOR] = HEXAMOTION.TURN_INDICATOR_UNKNOWN
                                lidar_obj_msg.state.discrete_state[HEXAMOTION.BRAKE_LIGHT] = HEXAMOTION.LIGHT_UNKNOWN
                                lidar_obj_msg.state.discrete_state[HEXAMOTION.REVERSE_LIGHT] = HEXAMOTION.LIGHT_UNKNOWN
                                lidar_obj_msg.state.discrete_state.append(int(obj[8]))  # num_points_in_box
                                lidar_obj_msg.state.discrete_state.append(-1 if np.isnan(obj[9]) else int(obj[9]))  # difficulty_level

                                # fill object classification
                                if obj[0] == 0:  # UNKNOWN
                                    lidar_obj_msg.state.classifications = [
                                        ObjectClassification(type=ObjectClassification.UNKNOWN, probability=1.0),
                                        ObjectClassification(type=ObjectClassification.ANIMAL, probability=1.0),
                                    ]
                                elif obj[0] == 1:  # VEHICLE
                                    lidar_obj_msg.state.classifications = [
                                        ObjectClassification(type=ObjectClassification.MOTORCYCLE, probability=1.0),
                                        ObjectClassification(type=ObjectClassification.CAR, probability=1.0),
                                        ObjectClassification(type=ObjectClassification.UTILITY, probability=1.0),
                                        ObjectClassification(type=ObjectClassification.BUS, probability=1.0),
                                        ObjectClassification(type=ObjectClassification.MICRO, probability=1.0),
                                    ]
                                elif obj[0] == 2:  # PEDESTRIAN
                                    lidar_obj_msg.state.classifications = [
                                        ObjectClassification(type=ObjectClassification.PEDESTRIAN, probability=1.0),
                                        ObjectClassification(type=ObjectClassification.VRU, probability=1.0),
                                    ]
                                elif obj[0] == 3:  # SIGN
                                    lidar_obj_msg.state.classifications = [
                                        ObjectClassification(type=20, probability=1.0)  # TODO: add to perception_msgs
                                    ]
                                elif obj[0] == 4:  # CYCLIST
                                    lidar_obj_msg.state.classifications = [
                                        ObjectClassification(type=ObjectClassification.BICYCLE, probability=1.0)
                                    ]
                                else:
                                    raise ValueError(f"Unknown class ID: {obj[0]}")

                                object_list_3d_msg.objects.append(lidar_obj_msg)
                    else:
                        object_list_3d_msg = None

                    if self.use_camera_object_list:
                        if frame_camera_objects is None:
                            raise ValueError("Camera object data is required for generating 2D object list. Please provide camera box files and set use_camera=True.")
                        camera_objects = frame_camera_objects[frame_camera_objects["key.frame_timestamp_micros"] == timestamp_key]

                        # convert object list to ROS message
                        object_list_2d_msg = ObjectList()
                        object_list_2d_msg.header.frame_id = "cam_front"

                        if len(camera_objects) > 0:
                            # Extract values once and compute bounding boxes
                            center_x = camera_objects["[CameraBoxComponent].box.center.x"].values
                            center_y = camera_objects["[CameraBoxComponent].box.center.y"].values
                            half_size_x = camera_objects["half_size_x"].values
                            half_size_y = camera_objects["half_size_y"].values

                            n_objects = len(camera_objects)
                            camera_object_list = np.empty((n_objects, 6), dtype=np.int32)
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

                                # fill discrete state and append additional attributes at the end
                                camera_obj_msg.state.discrete_state.append(-1 if np.isnan(obj[5]) else int(obj[5]))  # difficulty_level

                                # fill object classification
                                if obj[0] == 0:  # UNKNOWN
                                    camera_obj_msg.state.classifications = [
                                        ObjectClassification(type=ObjectClassification.UNKNOWN, probability=1.0),
                                        ObjectClassification(type=ObjectClassification.ANIMAL, probability=1.0),
                                    ]
                                elif obj[0] == 1:  # VEHICLE
                                    camera_obj_msg.state.classifications = [
                                        ObjectClassification(type=ObjectClassification.MOTORCYCLE, probability=1.0),
                                        ObjectClassification(type=ObjectClassification.CAR, probability=1.0),
                                        ObjectClassification(type=ObjectClassification.UTILITY, probability=1.0),
                                        ObjectClassification(type=ObjectClassification.BUS, probability=1.0),
                                        ObjectClassification(type=ObjectClassification.MICRO, probability=1.0),
                                    ]
                                elif obj[0] == 2:  # PEDESTRIAN
                                    camera_obj_msg.state.classifications = [
                                        ObjectClassification(type=ObjectClassification.PEDESTRIAN, probability=1.0),
                                        ObjectClassification(type=ObjectClassification.VRU, probability=1.0),
                                    ]
                                elif obj[0] == 3:  # SIGN
                                    camera_obj_msg.state.classifications = [
                                        ObjectClassification(type=20, probability=1.0)  # TODO: add to perception_msgs
                                    ]
                                elif obj[0] == 4:  # CYCLIST
                                    camera_obj_msg.state.classifications = [
                                        ObjectClassification(type=ObjectClassification.BICYCLE, probability=1.0)
                                    ]
                                else:
                                    raise ValueError(f"Unknown class ID: {obj[0]}")

                                object_list_2d_msg.objects.append(camera_obj_msg)
                    else:
                        object_list_2d_msg = None

                    # convert range image to point cloud in sensor frame
                    if frame_range_images is not None and beam_inclinations is not None:
                        range_image = frame_range_images[frame_range_images["key.frame_timestamp_micros"] == timestamp_key].iloc[0]
                        range_values = range_image["[LiDARComponent].range_image_return1.values"]
                        range_shape = range_image["[LiDARComponent].range_image_return1.shape"]
                        point_cloud = _convert_range_image_to_point_cloud(range_values, range_shape, beam_inclinations)

                        # Convert point cloud to ROS PointCloud2 message (in sensor frame)
                        header = Header()
                        header.frame_id = "lidar_top"
                        fields = [
                            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
                            PointField(name='elongation', offset=16, datatype=PointField.FLOAT32, count=1),
                        ]
                        point_cloud_msg = create_cloud(header, fields, point_cloud)
                    else:
                        point_cloud_msg = None

                    if frame_images is not None:
                        image_row = frame_images[frame_images["key.frame_timestamp_micros"] == timestamp_key].iloc[0]

                        # convert JPEG image to ROS Image message
                        jpeg_bytes = image_row["[CameraImageComponent].image"]
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
                    else:
                        image_msg = None
                    
                    i += 1
                    sample = {
                        "tf": tf_msgs,
                    }
                    if object_list_2d_msg is not None:
                        sample["object_list_2d"] = object_list_2d_msg
                    if object_list_3d_msg is not None:
                        sample["object_list_3d"] = object_list_3d_msg
                    if point_cloud_msg is not None:
                        sample["point_cloud"] = point_cloud_msg
                    if image_msg is not None:
                        sample["image"] = image_msg
                    yield (i, sample)


def _load_files(dataset_root_dir: pathlib.PosixPath, split: str, use_lidar: bool, use_camera: bool) -> Tuple[List, List, List, List, List, List]:
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

    return lidar_files, lidar_calibration_files, lidar_box_files, camera_files, camera_box_files, camera_calibration_files

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

    # Clean up intermediate arrays
    del range_image, range_values, intensity_values, elongation_values
    del valid_mask, valid_indices, ranges, intensities, elongations
    del inclinations_per_row, inclination, ratios, azimuth
    del cos_azimuth, sin_azimuth, cos_incl, sin_incl, x, y, z

    return point_cloud.astype(np.float32)
