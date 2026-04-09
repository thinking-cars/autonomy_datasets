# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

from typing import Any, Dict, Iterator, List, Tuple

from autonomy_datasets.datasets.utils import timestamp_micros_to_clock
from autonomy_datasets.datasets.dataset import DatasetAdapter
import numpy as np
from scipy.spatial.transform import Rotation

from nuscenes import NuScenes
from nuscenes.eval.common.utils import quaternion_yaw
from nuscenes.utils.geometry_utils import BoxVisibility
from nuscenes.utils.splits import create_splits_scenes

from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Quaternion, TransformStamped, Transform, Vector3
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header
from perception_msgs.msg import EgoData, EGO, ObjectList, Object, ObjectClassification, HEXAMOTION
import perception_msgs_utils as pmu
from tf2_msgs.msg import TFMessage


# Mapping from dataset class names to ROS ObjectClassification types
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
    "vehicle.bicycle": [ObjectClassification.BICYCLE],
    "vehicle.bus.bendy": [ObjectClassification.BUS],
    "vehicle.bus.rigid": [ObjectClassification.BUS],
    "vehicle.car": [ObjectClassification.CAR],
    "vehicle.construction": [ObjectClassification.UTILITY],
    "vehicle.emergency.ambulance": [ObjectClassification.UTILITY],
    "vehicle.emergency.police": [ObjectClassification.UTILITY],
    "vehicle.motorcycle": [ObjectClassification.MOTORCYCLE],
    "vehicle.trailer": [ObjectClassification.UTILITY],
    "vehicle.truck": [ObjectClassification.UTILITY],
}

_SENSOR_FEATURE_TO_TOPIC = {
    "CAM_FRONT": "camera_01",
    "LIDAR_TOP": "lidar_01",
}

_SENSOR_FEATURE_TO_FRAME_ID = {
    "CAM_FRONT": "cam_front",
    "LIDAR_TOP": "lidar_top",
}


class NuscenesAdapter(DatasetAdapter):
    """Converts nuScenes dataset files to ROS 2 messages."""

    def __init__(self, data_publishers: Dict[str, Any], split: str,
                 dataset_root_dir: str,
                 object_model: str = "HEXAMOTION",
                 use_camera: bool = False,
                 use_lidar: bool = False,
                 min_lidar_points_in_bbox: int = 1,
                 camera_box_visibility: BoxVisibility = BoxVisibility.ANY, camera_box_min_points: int = 1) -> None:

        super().__init__(
            data_publishers=data_publishers,
            version="0.1.0",
            release_notes={
                "0.1.0": "Initial integration into Autonomy.Datasets",
            },
        )
        self.split = split
        self.object_model = object_model
        self.use_camera = use_camera
        self.use_lidar = use_lidar

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

        if "mini" in self.split:
            self.nusc = NuScenes(version="v1.0-mini", dataroot=str(self.dataset_root_dir), verbose=True)
        else:
            self.nusc = NuScenes(version="v1.0-trainval", dataroot=str(self.dataset_root_dir), verbose=True)

        # add publishers for outgoing messages, actual publisher will be created in AutonomyDatasets node
        # self.data_publishers["ego_data"] = None
        if self.use_lidar:
            self.data_publishers["object_list/lidar_01"] = None
        if self.use_camera:
            self.data_publishers["object_list/camera_01"] = None
        for topic in _SENSOR_FEATURE_TO_TOPIC.values():
            if self.use_camera:
                if topic.startswith("camera_"):
                    self.data_publishers[f"{topic}/image_raw"] = None
                    self.data_publishers[f"{topic}/camera_info"] = None
            if self.use_lidar:
                if topic.startswith("lidar_"):
                    self.data_publishers[f"{topic}/point_cloud"] = None

    def generate_samples(self) -> Iterator[Tuple[int, Dict[str, Any]]]:
        scene_splits = create_splits_scenes()
        count_examples = 0
        for scene in self.nusc.scene:
            if scene["name"] in scene_splits[self.split]:
                instance_id_map: Dict[str, int] = {}
                sample_token = scene["first_sample_token"]
                while sample_token != "":
                    nusc_sample = self.nusc.get("sample", sample_token)
                    sample: Dict[str, Any] = {}
                    clock_msg = timestamp_micros_to_clock(int(nusc_sample["timestamp"]))

                    if self.use_lidar:
                        sample_data_lidar_top_token = nusc_sample["data"]["LIDAR_TOP"]
                        pcl_path, annotations, _ = self.nusc.get_sample_data(sample_data_lidar_top_token)

                        # Lidar point cloud in nuScenes frame (x=right, y=front, z=up)
                        scan = np.fromfile(pcl_path, dtype=np.float32).reshape((-1, 5))
                        lidar_msg = _get_lidar_point_cloud(scan, clock_msg.clock)
                        sample["lidar_01/point_cloud"] = lidar_msg

                        # Object list
                        object_list = []
                        for ann in annotations:
                            sample_annotation = self.nusc.get("sample_annotation", ann.token)
                            num_pts = sample_annotation["num_lidar_pts"]
                            if num_pts >= self.min_lidar_points_in_bbox:
                                instance_token = sample_annotation["instance_token"]
                                if instance_token not in instance_id_map:
                                    instance_id_map[instance_token] = len(instance_id_map)
                                object_list.append((ann, num_pts, instance_id_map[instance_token]))
                        object_list_msg = _labels_to_object_list(object_list, "lidar_top", clock_msg.clock)
                        sample["object_list/lidar_01"] = object_list_msg

                    if self.use_camera:
                        sample_data_cam_front_token = nusc_sample["data"]["CAM_FRONT"]
                        image_path, annotations, camera_intrinsic = self.nusc.get_sample_data(
                            sample_data_cam_front_token, box_vis_level=self.camera_box_visibility
                        )
                        # Camera image
                        image_msg = image_path  # TODO: convert to ROS message
                        sample["camera_01/image_raw"] = image_msg
                        sample["camera_01/camera_info"] = None  # TODO

                        object_list = []
                        for ann in annotations:
                            object_classification = _CLASS_MAPPING[ann.name]
                            # Ignore annotations with too less lidar or radar points
                            # as they may not be visible in the camera image
                            sample_annotation = self.nusc.get("sample_annotation", ann.token)
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

                            if self.object_model == "CAMERA2D":
                                xmin, ymin, xmax, ymax = transform_3d_to_2d_bbox(
                                    ann_x, ann_y, ann_z, ann_w, ann_l, ann_h, ann_q, camera_intrinsic
                                )

                                # Crop bounding box to image size
                                xmin = max(0, xmin)
                                ymin = max(0, ymin)
                                xmax = min(1600, xmax)
                                ymax = min(900, ymax)
                                sample_object = (object_classification, xmin, ymin, xmax, ymax, num_pts)

                            elif self.object_model == "HEXAMOTION":
                                # Note: get_sample_data() returns annotations in the sensor frame
                                # nuScenes sensor frame: x=right, y=down, z=forward
                                # Transform to desired frame: x=front, y=left, z=up
                                x_cam = ann_z
                                y_cam = -ann_x
                                z_cam = -ann_y
                                yaw_cam = ann_q.yaw_pitch_roll[0] - np.pi / 2

                                sample_object = (object_classification, x_cam, y_cam, z_cam, yaw_cam, ann_l, ann_w, ann_h, num_pts)
                            else:
                                raise ValueError(f"Invalid object model: {self.object_model}")

                            object_list.append(sample_object)

                        object_list_msg = object_list  # TODO: convert to ROS message
                        sample["object_list/camera_01"] = object_list_msg

                    # Build static TF messages from sensor calibration
                    tf_msgs = _build_tf_msgs(self.nusc, nusc_sample)

                    sample["scene_id"] = scene["token"]
                    sample["/clock"] = clock_msg
                    sample["/tf_static"] = TFMessage(transforms=tf_msgs)
                    sample["/tf"] = TFMessage(transforms=[])

                    sample_token = nusc_sample["next"]
                    count_examples += 1
                    yield count_examples, sample


def _build_tf_msgs(nusc: NuScenes, nusc_sample: Dict[str, Any]) -> List[TransformStamped]:
    """Build static TF messages from nuScenes sensor calibration.

    Retrieves the calibrated sensor extrinsics (translation + rotation) for each
    sensor channel in the sample and creates TransformStamped messages from
    base_link to the respective sensor frame.

    Args:
        nusc: NuScenes database instance.
        nusc_sample: A nuScenes sample record dict.

    Returns:
        List of TransformStamped messages.
    """
    tf_msgs = []
    for sensor_channel, child_frame_id in _SENSOR_FEATURE_TO_FRAME_ID.items():
        if sensor_channel not in nusc_sample["data"]:
            continue
        sample_data = nusc.get("sample_data", nusc_sample["data"][sensor_channel])
        calibrated_sensor = nusc.get(
            "calibrated_sensor", sample_data["calibrated_sensor_token"]
        )
        translation = calibrated_sensor["translation"]
        # nuScenes quaternion is [w, x, y, z]
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


def _labels_to_object_list(labels: List[Any], frame_id: str, stamp_msg: Time) -> ObjectList:
    """Convert labels to a ROS ObjectList message."""
    object_list_msg = ObjectList()
    object_list_msg.header.frame_id = frame_id
    object_list_msg.header.stamp = stamp_msg

    for label, num_pts, instance_id in labels:
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

        object_list_msg.objects.append(obj_msg)

    return object_list_msg


def _get_lidar_point_cloud(lidar_data, stamp_msg: Time) -> PointCloud2:
    # Build fields: x, y, z (intensity from attributes if available)
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="timestamp", offset=16, datatype=PointField.FLOAT32, count=1),
    ]

    header = Header(frame_id="lidar_top", stamp=stamp_msg)
    return create_cloud(header, fields, lidar_data)


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
