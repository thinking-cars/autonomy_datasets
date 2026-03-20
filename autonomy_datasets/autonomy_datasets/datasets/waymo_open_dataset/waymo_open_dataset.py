import gc
import pathlib
from typing import Any, Dict, Iterator, Tuple

import numpy as np
import pandas as pd


class WaymoOpenDatasetAdapter:
    """Converts Waymo Open Dataset parquet files to ROS 2 messages.
    """

    def __init__(self, dataset_root_dir: str, min_lidar_points_in_bbox: int = 1) -> None:

        self.version = "0.1.0"
        self.release_notes = {
            "0.1.0": "Initial integration into Autonomy.Datasets",
        }

        # Root directory of the Waymo Open Dataset
        self.dataset_root_dir = pathlib.PosixPath(dataset_root_dir)

        # Minimum number of lidar points in bounding box to be considered in "lidar_objects" datasets
        self.min_lidar_points_in_bbox = min_lidar_points_in_bbox

    def generate_samples(self, config: str, split: str):
        
        if config == "lidar_objects":
            return self._generate_lidar_objects(split)
        elif config == "camera_objects_2d":
            return self._generate_camera_objects_2d(split)
        elif config == "camera_objects_3d":
            return self._generate_camera_objects_3d(split)
        else:
            raise ValueError(f"Unknown config: {config}")

    def _generate_lidar_objects(self, split) -> Iterator[Tuple[int, Dict[str, Any]]]:
        """Generate samples containing lidar point clouds and 3D bounding boxes from Waymo Open Dataset parquet files.

        Args:
            split: Split name (e.g., 'training', 'validation', 'training_mini', 'validation_mini').

        Yields:
            Tuple of (example_id, example_dict) containing point clouds and objects.
        """
        i = -1

        if "training" in split:
            split_path = self.dataset_root_dir / "training"
        elif "validation" in split:
            split_path = self.dataset_root_dir / "validation"
        else:
            raise ValueError(f"Unknown split: {split}")

        lidar_path = split_path / "lidar"
        lidar_files = sorted(lidar_path.glob("*.parquet"))
        lidar_calibration_path = split_path / "lidar_calibration"
        lidar_calibration_files = sorted(lidar_calibration_path.glob("*.parquet"))
        lidar_box_path = split_path / "lidar_box"
        lidar_box_files = sorted(lidar_box_path.glob("*.parquet"))

        if "mini" in split:
            lidar_files = lidar_files[:1]
            lidar_box_files = lidar_box_files[:1]
            lidar_calibration_files = lidar_calibration_files[:1]
            process_every_nth_frame = 10
        else:
            process_every_nth_frame = 1

        for lidar_file, lidar_box_file, lidar_calibration_file in zip(
            lidar_files, lidar_box_files, lidar_calibration_files
        ):
            # Read parquet files with only needed columns
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

            # Filter only TOP lidar upfront
            lidar_pandas = lidar_pandas[lidar_pandas["key.laser_name"] == 1].copy()
            lidar_objects_pandas = lidar_objects_pandas[
                lidar_objects_pandas["[LiDARBoxComponent].num_top_lidar_points_in_box"] >= MIN_LIDAR_POINTS_IN_BBOX
            ].copy()
            lidar_calibration_pandas = lidar_calibration_pandas[lidar_calibration_pandas["key.laser_name"] == 1].copy()

            for segment_context_key in lidar_pandas["key.segment_context_name"].unique():
                # Use boolean masks for filtering
                segment_mask_lidar = lidar_pandas["key.segment_context_name"] == segment_context_key
                segment_mask_obj = lidar_objects_pandas["key.segment_context_name"] == segment_context_key
                segment_mask_cal = lidar_calibration_pandas["key.segment_context_name"] == segment_context_key

                frame_range_images = lidar_pandas[segment_mask_lidar]
                frame_objects = lidar_objects_pandas[segment_mask_obj]
                frame_calibrations = lidar_calibration_pandas[segment_mask_cal]

                assert len(frame_calibrations) == 1, "Expected exactly one calibration per frame"
                calibration = frame_calibrations.iloc[0]

                # Pre-compute extrinsic and beam inclinations once per segment
                extrinsic = np.array(calibration["[LiDARCalibrationComponent].extrinsic.transform"]).reshape(4, 4)
                translation = extrinsic[:3, 3]
                beam_inclinations = np.array(calibration["[LiDARCalibrationComponent].beam_inclination.values"])

                for timestamp_key in frame_range_images["key.frame_timestamp_micros"].unique():
                    if (i + 1) % process_every_nth_frame != 0:
                        i += 1
                        continue

                    # Use boolean masks for timestamp filtering
                    ts_mask_lidar = frame_range_images["key.frame_timestamp_micros"] == timestamp_key
                    ts_mask_obj = frame_objects["key.frame_timestamp_micros"] == timestamp_key

                    range_image = frame_range_images[ts_mask_lidar].iloc[0]
                    objects = frame_objects[ts_mask_obj]

                    range_values = range_image["[LiDARComponent].range_image_return1.values"]
                    range_shape = range_image["[LiDARComponent].range_image_return1.shape"]

                    point_cloud = _convert_range_image_to_point_cloud(range_values, range_shape, beam_inclinations)

                    # Build objects array in sensor frame
                    if len(objects) > 0:
                        # Pre-allocate and fill objects array
                        n_objects = len(objects)
                        object_list = np.empty((n_objects, 10), dtype=np.float32)
                        object_list[:, 0] = objects["[LiDARBoxComponent].type"].values
                        object_list[:, 1] = objects["[LiDARBoxComponent].box.center.x"].values - translation[0]
                        object_list[:, 2] = objects["[LiDARBoxComponent].box.center.y"].values - translation[1]
                        object_list[:, 3] = objects["[LiDARBoxComponent].box.center.z"].values - translation[2]
                        object_list[:, 4] = objects["[LiDARBoxComponent].box.heading"].values
                        object_list[:, 5] = objects["[LiDARBoxComponent].box.size.x"].values
                        object_list[:, 6] = objects["[LiDARBoxComponent].box.size.y"].values
                        object_list[:, 7] = objects["[LiDARBoxComponent].box.size.z"].values
                        object_list[:, 8] = objects["[LiDARBoxComponent].num_top_lidar_points_in_box"].values
                        object_list[:, 9] = objects["[LiDARBoxComponent].difficulty_level.detection"].values
                    else:
                        object_list = np.zeros((0, 10), dtype=np.float32)

                    i += 1
                    yield (
                        i,
                        {
                            "point_cloud": point_cloud,
                            "objects": object_list,
                        },
                    )
                    # Explicitly delete large arrays to free memory immediately
                    del point_cloud, object_list, range_values

            # Clean up DataFrames after processing each file
            del lidar_pandas, lidar_objects_pandas, lidar_calibration_pandas
            gc.collect()

    def _generate_camera_objects_3d(self, split) -> Iterator[Tuple[int, Dict[str, Any]]]:
        """Generate samples containing camera images and 3D bounding boxes from Waymo Open Dataset parquet files.

        Args:
            split: Split name (e.g., 'training', 'validation', 'mini_training').

        Yields:
            Tuple of (example_id, example_dict) containing images and 3D objects.
        """
        i = -1

        if "training" in split:
            split_path = self.dataset_root_dir / "training"
        elif "validation" in split:
            split_path = self.dataset_root_dir / "validation"
        else:
            raise ValueError(f"Unknown split: {split}")

        camera_path = split_path / "camera_image"
        camera_files = sorted(camera_path.glob("*.parquet"))
        camera_calibration_path = split_path / "camera_calibration"
        camera_calibration_files = sorted(camera_calibration_path.glob("*.parquet"))
        lidar_box_path = split_path / "lidar_box"
        lidar_box_files = sorted(lidar_box_path.glob("*.parquet"))

        if "mini" in split:
            camera_files = camera_files[:1]
            camera_calibration_files = camera_calibration_files[:1]
            lidar_box_files = lidar_box_files[:1]
            process_every_nth_frame = 10
        else:
            process_every_nth_frame = 1

        for camera_file, camera_calibration_file, lidar_box_file in zip(
            camera_files, camera_calibration_files, lidar_box_files
        ):
            # Read parquet files with only needed columns
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

            # Filter only FRONT camera (camera_name == 1) upfront
            camera_pandas = camera_pandas[camera_pandas["key.camera_name"] == 1].copy()
            camera_calibration_pandas = camera_calibration_pandas[
                camera_calibration_pandas["key.camera_name"] == 1
            ].copy()

            for segment_context_key in camera_pandas["key.segment_context_name"].unique():
                # Use boolean masks for filtering
                segment_mask_img = camera_pandas["key.segment_context_name"] == segment_context_key
                segment_mask_obj = lidar_objects_pandas["key.segment_context_name"] == segment_context_key
                segment_mask_cal = camera_calibration_pandas["key.segment_context_name"] == segment_context_key

                frame_images = camera_pandas[segment_mask_img]
                frame_lidar_objects = lidar_objects_pandas[segment_mask_obj]
                frame_calibrations = camera_calibration_pandas[segment_mask_cal]

                assert len(frame_calibrations) == 1, "Expected exactly one calibration per frame"
                calibration = frame_calibrations.iloc[0]

                # Get camera extrinsic and compute inverse once
                extrinsic = np.array(calibration["[CameraCalibrationComponent].extrinsic.transform"]).reshape(4, 4)
                extrinsic_inv = np.linalg.inv(extrinsic)
                translation = extrinsic[:3, 3]

                # Build intrinsic matrix once
                intrinsic = np.array(
                    [
                        [
                            calibration["[CameraCalibrationComponent].intrinsic.f_u"],
                            0,
                            calibration["[CameraCalibrationComponent].intrinsic.c_u"],
                        ],
                        [
                            0,
                            calibration["[CameraCalibrationComponent].intrinsic.f_v"],
                            calibration["[CameraCalibrationComponent].intrinsic.c_v"],
                        ],
                        [0, 0, 1],
                    ],
                    dtype=np.float32,
                )

                # Get image dimensions
                width = int(calibration["[CameraCalibrationComponent].width"])
                height = int(calibration["[CameraCalibrationComponent].height"])

                for timestamp_key in frame_images["key.frame_timestamp_micros"].unique():
                    if (i + 1) % process_every_nth_frame != 0:
                        i += 1
                        continue

                    # Use boolean masks for timestamp filtering
                    ts_mask_img = frame_images["key.frame_timestamp_micros"] == timestamp_key
                    ts_mask_obj = frame_lidar_objects["key.frame_timestamp_micros"] == timestamp_key

                    image_row = frame_images[ts_mask_img].iloc[0]
                    frame_lidar_objects_ts = frame_lidar_objects[ts_mask_obj]

                    # Decode image directly
                    image = tf.io.decode_image(
                        image_row["[CameraImageComponent].image"], channels=3, expand_animations=False
                    ).numpy()

                    # Build 3D objects array by checking visibility in camera
                    if len(frame_lidar_objects_ts) > 0:
                        # Extract centers efficiently (pre-allocate)
                        n_objs = len(frame_lidar_objects_ts)
                        centers_vehicle = np.empty((n_objs, 3), dtype=np.float32)
                        centers_vehicle[:, 0] = frame_lidar_objects_ts["[LiDARBoxComponent].box.center.x"].values
                        centers_vehicle[:, 1] = frame_lidar_objects_ts["[LiDARBoxComponent].box.center.y"].values
                        centers_vehicle[:, 2] = frame_lidar_objects_ts["[LiDARBoxComponent].box.center.z"].values

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

                        del centers_homogeneous, centers_camera_origin

                        # Project to image plane
                        projected = (intrinsic @ centers_camera_frame.T).T
                        projected_2d = projected[:, :2] / projected[:, 2:3]

                        # Check visibility
                        visibility_mask = (
                            (centers_camera_frame[:, 2] > 0)
                            & (projected_2d[:, 0] >= 0)
                            & (projected_2d[:, 0] < width)
                            & (projected_2d[:, 1] >= 0)
                            & (projected_2d[:, 1] < height)
                        )

                        del projected, projected_2d, centers_camera_frame

                        visible_objects = frame_lidar_objects_ts[visibility_mask]

                        if len(visible_objects) > 0:
                            # Get centers for visible objects and apply translation
                            centers_visible = centers_vehicle[visibility_mask]
                            centers_camera_origin = centers_visible - translation

                            # Pre-allocate and fill objects array
                            n_visible = len(visible_objects)
                            objects = np.empty((n_visible, 10), dtype=np.float32)
                            objects[:, 0] = visible_objects["[LiDARBoxComponent].type"].values
                            objects[:, 1] = centers_camera_origin[:, 0]
                            objects[:, 2] = centers_camera_origin[:, 1]
                            objects[:, 3] = centers_camera_origin[:, 2]
                            objects[:, 4] = visible_objects["[LiDARBoxComponent].box.heading"].values
                            objects[:, 5] = visible_objects["[LiDARBoxComponent].box.size.x"].values
                            objects[:, 6] = visible_objects["[LiDARBoxComponent].box.size.y"].values
                            objects[:, 7] = visible_objects["[LiDARBoxComponent].box.size.z"].values
                            objects[:, 8] = visible_objects["[LiDARBoxComponent].num_top_lidar_points_in_box"].values
                            objects[:, 9] = visible_objects["[LiDARBoxComponent].difficulty_level.detection"].values

                            del centers_visible, centers_camera_origin
                        else:
                            objects = np.zeros((0, 10), dtype=np.float32)

                        del centers_vehicle, visibility_mask
                    else:
                        objects = np.zeros((0, 10), dtype=np.float32)

                    i += 1
                    yield (
                        i,
                        {
                            "image_front": image,
                            "objects": objects,
                        },
                    )
                    # Explicitly delete to free memory
                    del image, objects

            # Clean up DataFrames after processing each file
            del camera_pandas, lidar_objects_pandas, camera_calibration_pandas
            gc.collect()

    def _generate_camera_objects_2d(self, split) -> Iterator[Tuple[int, Dict[str, Any]]]:
        """Generate samples containing lidar point clouds and 2D bounding boxes from Waymo Open Dataset parquet files.

        Args:
            split: Split name (e.g., 'training', 'validation', 'mini_training').

        Yields:
            Tuple of (example_id, example_dict) containing images and 2D objects.
        """
        i = -1

        if "training" in split:
            split_path = self.dataset_root_dir / "training"
        elif "validation" in split:
            split_path = self.dataset_root_dir / "validation"
        else:
            raise ValueError(f"Unknown split: {split}")

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
            process_every_nth_frame = 10
        else:
            process_every_nth_frame = 1

        for camera_file, camera_box_file, camera_calibration_file in zip(
            camera_files, camera_box_files, camera_calibration_files
        ):
            # Read parquet files with only needed columns
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

            # Filter only FRONT camera (camera_name == 1) upfront
            camera_pandas = camera_pandas[camera_pandas["key.camera_name"] == 1].copy()
            camera_objects_pandas = camera_objects_pandas[camera_objects_pandas["key.camera_name"] == 1].copy()

            # Pre-compute half sizes for bounding boxes (vectorized)
            camera_objects_pandas["half_size_x"] = camera_objects_pandas["[CameraBoxComponent].box.size.x"].values / 2
            camera_objects_pandas["half_size_y"] = camera_objects_pandas["[CameraBoxComponent].box.size.y"].values / 2

            for segment_context_key in camera_pandas["key.segment_context_name"].unique():
                # Use boolean indexing which is faster than multiple filter operations
                segment_mask_img = camera_pandas["key.segment_context_name"] == segment_context_key
                segment_mask_obj = camera_objects_pandas["key.segment_context_name"] == segment_context_key

                frame_images = camera_pandas[segment_mask_img]
                frame_objects = camera_objects_pandas[segment_mask_obj]

                for timestamp_key in frame_images["key.frame_timestamp_micros"].unique():
                    if (i + 1) % process_every_nth_frame != 0:
                        i += 1
                        continue

                    # Direct iloc access is faster than multiple filtering
                    ts_mask_img = frame_images["key.frame_timestamp_micros"] == timestamp_key
                    image_row = frame_images[ts_mask_img].iloc[0]

                    # Get and decode image
                    image = tf.io.decode_image(
                        image_row["[CameraImageComponent].image"], channels=3, expand_animations=False
                    ).numpy()

                    # Filter objects for this timestamp
                    ts_mask_obj = frame_objects["key.frame_timestamp_micros"] == timestamp_key
                    frame_objects_ts = frame_objects[ts_mask_obj]

                    # Build 2D objects array (class_id, xmin, ymin, xmax, ymax, difficulty_level)
                    if len(frame_objects_ts) > 0:
                        # Extract values once and compute bounding boxes
                        center_x = frame_objects_ts["[CameraBoxComponent].box.center.x"].values
                        center_y = frame_objects_ts["[CameraBoxComponent].box.center.y"].values
                        half_size_x = frame_objects_ts["half_size_x"].values
                        half_size_y = frame_objects_ts["half_size_y"].values

                        # Pre-allocate array and fill it (faster than column_stack)
                        n_objects = len(frame_objects_ts)
                        objects = np.empty((n_objects, 6), dtype=np.int32)
                        objects[:, 0] = frame_objects_ts["[CameraBoxComponent].type"].values
                        objects[:, 1] = center_x - half_size_x
                        objects[:, 2] = center_y - half_size_y
                        objects[:, 3] = center_x + half_size_x
                        objects[:, 4] = center_y + half_size_y
                        objects[:, 5] = frame_objects_ts["[CameraBoxComponent].difficulty_level.detection"].values

                        del center_x, center_y, half_size_x, half_size_y
                    else:
                        objects = np.zeros((0, 6), dtype=np.int32)

                    i += 1
                    yield (
                        i,
                        {
                            "image_front": image,
                            "objects": objects,
                        },
                    )

                    # Explicitly delete to free memory
                    del image, objects

            # Clean up DataFrames after processing each file
            del camera_pandas, camera_objects_pandas
            gc.collect()


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


def get_class(class_in: Any, reverse: bool = False) -> Any:
    """Map between class names and class IDs for Waymo Open Dataset.

    Args:
        class_in: Class name (str) in forward mode, or class ID (int) in
            reverse mode.
        reverse: If True, convert class ID to class name. If False, convert
            class name to class ID.

    Returns:
        Class ID (int) in forward mode, or class name (str) in reverse mode.
        Returns None if class is not found.
    """
    class_name_to_class_id = {
        "UNKNOWN": 0,
        "VEHICLE": 1,
        "PEDESTRIAN": 2,
        "SIGN": 3,
        "CYCLIST": 4,
    }

    if not reverse:
        # Convert class name to class ID
        return class_name_to_class_id.get(class_in, None)
    else:
        # Convert class ID to class name
        class_id_to_class_name = {v: k for k, v in class_name_to_class_id.items()}
        return class_id_to_class_name.get(class_in, None)
