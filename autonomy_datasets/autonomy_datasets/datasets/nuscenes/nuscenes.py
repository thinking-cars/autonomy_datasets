# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

from typing import Any, Tuple

import numpy as np
from nuscenes import NuScenes
from nuscenes.eval.common.utils import quaternion_yaw
from nuscenes.utils.geometry_utils import BoxVisibility
from nuscenes.utils.splits import create_splits_scenes


class NuscenesAdapter:
    """Converts nuScenes dataset files to ROS 2 messages.
    """

    def __init__(self, dataset_root_dir: str, min_lidar_points_in_bbox: int = 1, 
                 camera_box_visibility: BoxVisibility = BoxVisibility.ANY, camera_box_min_points: int = 1) -> None:
        
        self.version = "0.1.0"
        self.release_notes = {
            "0.1.0": "Initial integration into Autonomy.Datasets",
        }
        # Root directory of the extracted nuScenes dataset
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

        self.nusc = None

    def generate_samples(self, config: str, split: str):
        
        if split == "mini":
            self.nusc = NuScenes(version="v1.0-mini", dataroot=str(self.dataset_root_dir), verbose=True)
            self.every_nth_sample = 10
        else:
            self.nusc = NuScenes(version="v1.0-trainval", dataroot=str(self.dataset_root_dir), verbose=True)
            self.every_nth_sample = 1

        return self._generate_examples(config, split)

    def _generate_examples(self, config: str, split: str):
        """Generate examples for the given split.

        Args:
            config: The configuration for the examples.
            split: The dataset split ('train' or 'val').

        Yields:
            Tuple of (example_id, example_dict) for each sample.
        """

        scene_splits = create_splits_scenes()
        count_examples = 0
        for scene in self.nusc.scene:
            if scene["name"] in scene_splits[split]:
                sample_token = scene["first_sample_token"]
                while sample_token != "":
                    if count_examples % self.every_nth_sample != 0:
                        sample_token = self.nusc.get("sample", sample_token)["next"]
                        count_examples += 1
                        continue

                    sample = self.nusc.get("sample", sample_token)
                    return_dict = {}

                    # CONFIG 1: lidar + objects
                    if "lidar_objects" in config:
                        sample_data_lidar_top_token = sample["data"]["LIDAR_TOP"]
                        pcl_path, annotations, _ = self.nusc.get_sample_data(sample_data_lidar_top_token)

                        # Lidar point cloud
                        # Transform from nuScenes frame (x=right, y=front, z=up)
                        # to desired frame (x=forward, y=left, z=up)
                        scan = np.fromfile(pcl_path, dtype=np.float32)
                        scan = scan.reshape((-1, 5))
                        # Swap and negate: new_x = old_y, new_y = -old_x
                        x_old = scan[:, 0].copy()
                        y_old = scan[:, 1].copy()
                        scan[:, 0] = y_old  # x = forward (was y)
                        scan[:, 1] = -x_old  # y = left (was -x)
                        return_dict["point_cloud"] = scan

                        # Object list
                        object_list = []
                        for ann in annotations:
                            class_id = get_class(ann.name)
                            sample_annotation = self.nusc.get("sample_annotation", ann.token)
                            num_pts = sample_annotation["num_lidar_pts"]
                            if num_pts >= MIN_LIDAR_POINTS_IN_BBOX:
                                x_nusc, y_nusc, ann_z = ann.center
                                # Transform center coordinates
                                ann_x = y_nusc  # forward
                                ann_y = -x_nusc  # left
                                width, length, height = ann.wlh
                                yaw = quaternion_yaw(ann.orientation)
                                # Adjust yaw for coordinate frame change
                                yaw = yaw + np.pi + np.pi / 2
                                while yaw > np.pi:
                                    yaw -= 2 * np.pi
                                sample_object = (class_id, ann_x, ann_y, ann_z, yaw, length, width, height, num_pts)
                                object_list.append(sample_object)
                        if not object_list:
                            object_list = np.empty((0, 9), dtype=np.float32)
                        return_dict["objects"] = object_list

                    # CONFIG 2: camera + objects (2D/3D)
                    elif "camera_objects" in config:
                        sample_data_cam_front_token = sample["data"]["CAM_FRONT"]
                        image_path, annotations, camera_intrinsic = self.nusc.get_sample_data(
                            sample_data_cam_front_token, box_vis_level=CAMERA_BOX_VISIBILITY
                        )

                        # Camera image
                        return_dict["image_front"] = image_path

                        object_list = []
                        for ann in annotations:
                            class_id = get_class(ann.name)
                            # Ignore annotations with too less lidar or radar points
                            # as they may not be visible in the camera image
                            sample_annotation = self.nusc.get("sample_annotation", ann.token)
                            num_lidar_pts = sample_annotation["num_lidar_pts"]
                            num_radar_pts = sample_annotation["num_radar_pts"]
                            num_pts = num_lidar_pts + num_radar_pts
                            if num_pts < CAMERA_BOX_MIN_POINTS:
                                continue

                            ann_x, ann_y, ann_z = ann.center
                            ann_q = ann.orientation
                            ann_w, ann_l, ann_h = ann.wlh

                            # Check if object is in front of camera (z > 0 in camera frame)
                            if ann_z <= 0:
                                continue

                            if "2d" in config:
                                xmin, ymin, xmax, ymax = transform_3d_to_2d_bbox(
                                    ann_x, ann_y, ann_z, ann_w, ann_l, ann_h, ann_q, camera_intrinsic
                                )

                                # Crop bounding box to image size
                                xmin = max(0, xmin)
                                ymin = max(0, ymin)
                                xmax = min(1600, xmax)
                                ymax = min(900, ymax)
                                sample_object = (class_id, xmin, ymin, xmax, ymax, num_pts)

                            elif "3d" in config:
                                # Note: get_sample_data() returns annotations in the sensor frame
                                # nuScenes sensor frame: x=right, y=down, z=forward
                                # Transform to desired frame: x=front, y=left, z=up
                                x_cam = ann_z
                                y_cam = -ann_x
                                z_cam = -ann_y
                                yaw_cam = ann_q.yaw_pitch_roll[0] - np.pi / 2

                                sample_object = (class_id, x_cam, y_cam, z_cam, yaw_cam, ann_l, ann_w, ann_h, num_pts)
                            else:
                                raise ValueError(f"Invalid builder config: {config}")

                            object_list.append(sample_object)

                        if not object_list:
                            if "2d" in config:
                                object_list = np.empty((0, 6), dtype=np.int32)
                            else:
                                object_list = np.empty((0, 9), dtype=np.float32)
                        return_dict["objects"] = object_list

                    else:
                        raise ValueError("Invalid builder config")

                    sample_token = sample["next"]
                    count_examples += 1
                    yield count_examples, return_dict


def get_class(class_in: Any, reverse: bool = False) -> Any:
    """Map between class names and class IDs for nuScenes dataset.

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
        "animal": 0,
        "human.pedestrian.adult": 1,
        "human.pedestrian.child": 2,
        "human.pedestrian.construction_worker": 3,
        "human.pedestrian.personal_mobility": 4,
        "human.pedestrian.police_officer": 5,
        "human.pedestrian.stroller": 6,
        "human.pedestrian.wheelchair": 7,
        "movable_object.barrier": 8,
        "movable_object.debris": 9,
        "movable_object.pushable_pullable": 10,
        "movable_object.trafficcone": 11,
        "static_object.bicycle_rack": 12,
        "vehicle.bicycle": 13,
        "vehicle.bus.bendy": 14,
        "vehicle.bus.rigid": 15,
        "vehicle.car": 16,
        "vehicle.construction": 17,
        "vehicle.emergency.ambulance": 18,
        "vehicle.emergency.police": 19,
        "vehicle.motorcycle": 20,
        "vehicle.trailer": 21,
        "vehicle.truck": 22,
    }

    if reverse:
        # Find class name by class ID
        for name, cid in class_name_to_class_id.items():
            if cid == class_in:
                return name
        return None
    else:
        return class_name_to_class_id.get(class_in)


def transform_3d_to_2d_bbox(
    x: float,
    y: float,
    z: float,
    width: float,
    length: float,
    height: float,
    orientation: Any,
    camera_intrinsic: np.ndarray,
) -> Tuple[int, int, int, int]:
    """Transform 3D bounding box to 2D bounding box in image coordinates.

    Projects the 8 corners of a 3D bounding box to the image plane and
    computes the 2D bounding box that encompasses all projected corners.

    Args:
        x: X-coordinate of the bounding box center.
        y: Y-coordinate of the bounding box center.
        z: Z-coordinate of the bounding box center.
        width: Width of the bounding box.
        length: Length of the bounding box.
        height: Height of the bounding box.
        orientation: Quaternion representing the bounding box orientation.
        camera_intrinsic: Camera intrinsic matrix (3x3 or 4x4).

    Returns:
        Tuple of (xmin, ymin, xmax, ymax) representing the 2D bounding box
        in pixel coordinates.

    Note:
        Based on nuscenes.data_classes.Box.corners().
        3D bounding box convention: x points forward, y to the left, z up.
    """
    # Compute 3D bounding box corners
    x_corners = length / 2 * np.array([1, 1, 1, 1, -1, -1, -1, -1])
    y_corners = width / 2 * np.array([1, -1, -1, 1, 1, -1, -1, 1])
    z_corners = height / 2 * np.array([1, 1, -1, -1, 1, 1, -1, -1])
    corners = np.vstack((x_corners, y_corners, z_corners))

    # Rotate and translate corners
    corners = np.dot(orientation.rotation_matrix, corners)
    corners[0, :] += x
    corners[1, :] += y
    corners[2, :] += z

    # Project to camera view (based on nuscenes.utils.geometry_utils.view_points)
    view = camera_intrinsic
    points = corners
    normalize = True
    # Validate input dimensions
    assert view.shape[0] <= 4, "View matrix rows must be <= 4"
    assert view.shape[1] <= 4, "View matrix columns must be <= 4"
    assert points.shape[0] == 3, "Points must have 3 dimensions"

    # Convert to homogeneous coordinates
    viewpad = np.eye(4)
    viewpad[: view.shape[0], : view.shape[1]] = view
    nbr_points = points.shape[1]
    points = np.concatenate((points, np.ones((1, nbr_points))))
    points = np.dot(viewpad, points)
    points = points[:3, :]

    # Normalize by depth
    if normalize:
        depth = points[2:3, :].repeat(3, 0).reshape(3, nbr_points)
        points = points / depth

    # Extract 2D corners
    corners = points[:2, :]

    # Compute 2D bounding box
    xmin = int(np.min(corners[0]))
    xmax = int(np.max(corners[0]))
    ymin = int(np.min(corners[1]))
    ymax = int(np.max(corners[1]))

    return xmin, ymin, xmax, ymax
