# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

from typing import Any, Dict, Iterator, List, Optional, Tuple

import cv2
import numpy as np
import perception_msgs_utils as pmu
from autonomy_datasets.datasets.dataset import DatasetAdapter
from autonomy_datasets.datasets.nuscenes.lanelet2_converter import get_location_origin, nuscenes_map_to_lanelet2_osm
from autonomy_datasets.datasets.utils import timestamp_micros_to_clock
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from nuscenes import NuScenes
from nuscenes.can_bus.can_bus_api import NuScenesCanBus
from nuscenes.utils.data_classes import RadarPointCloud
from nuscenes.utils.geometry_utils import BoxVisibility
from nuscenes.utils.splits import create_splits_scenes
from perception_msgs.msg import EGO, EgoData, HEXAMOTION, Object, ObjectClassification, ObjectList, ObjectReferencePoint
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header
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
    "CAM_FRONT_RIGHT": "camera_02",
    "CAM_BACK_RIGHT": "camera_03",
    "CAM_BACK": "camera_04",
    "CAM_BACK_LEFT": "camera_05",
    "CAM_FRONT_LEFT": "camera_06",
    "LIDAR_TOP": "lidar_01",
    "RADAR_FRONT": "radar_01",
    "RADAR_FRONT_RIGHT": "radar_02",
    "RADAR_BACK_RIGHT": "radar_03",
    "RADAR_BACK_LEFT": "radar_04",
    "RADAR_FRONT_LEFT": "radar_05",
}

_SENSOR_FEATURE_TO_FRAME_ID = {
    "CAM_FRONT": "cam_front",
    "CAM_FRONT_RIGHT": "cam_front_right",
    "CAM_BACK_RIGHT": "cam_back_right",
    "CAM_BACK": "cam_back",
    "CAM_BACK_LEFT": "cam_back_left",
    "CAM_FRONT_LEFT": "cam_front_left",
    "LIDAR_TOP": "lidar_top",
    "RADAR_FRONT": "radar_front",
    "RADAR_FRONT_RIGHT": "radar_front_right",
    "RADAR_BACK_RIGHT": "radar_back_right",
    "RADAR_BACK_LEFT": "radar_back_left",
    "RADAR_FRONT_LEFT": "radar_front_left",
}

_MISSING_META_INFO_WARNING_PRINTED = False

# Maximum time difference between a sample and the CAN bus message applied to it. The nuScenes
# CAN bus expansion logs "pose" at 50 Hz, "steeranglefeedback" at 100 Hz and "vehicle_monitor"
# at 2 Hz; messages further away than a few logging periods are not representative anymore.
_MAX_POSE_AGE_MICROS = 100_000
_MAX_STEERING_AGE_MICROS = 100_000
_MAX_VEHICLE_MONITOR_AGE_MICROS = 1_000_000

# Ratio between the logged steering wheel angle and the Ackermann steering angle of the road
# wheels. Determined from the CAN bus data itself by comparing "steeranglefeedback" against the
# steering angle of a kinematic bicycle model, atan(yaw_rate * wheelbase / velocity), using the
# 2.588 m wheelbase of the Renault Zoe: median 15.56 over 75 scenes (10th..90th percentile
# 14.45..16.57), which matches the steering ratio documented for that vehicle.
_STEERING_RATIO = 15.56

# Longitudinal velocity below which the ego vehicle is reported to be at standstill [m/s]
_STANDSTILL_VELOCITY = 0.1

# "brake_switch" value logged while the brake pedal is released; the pressed pedal is reported
# as 2 or 4 (the only other values occurring in the dataset)
_BRAKE_SWITCH_RELEASED = 1


class _SceneCanBus:
    """Time-aligned access to the CAN bus messages recorded for a single nuScenes scene.

    The CAN bus expansion provides the ego vehicle signals that the nuScenes ego poses lack:
    velocity, acceleration, yaw rate, steering angle and the state of the vehicle's indicators.
    Each sample is filled from the message closest in time, which is dropped when it is not
    recorded close enough to the sample.
    """

    def __init__(self, can_bus: NuScenesCanBus, scene_name: str) -> None:
        """Load the CAN bus messages of a scene, or nothing if the scene has no CAN bus data.

        Args:
            can_bus: nuScenes CAN bus expansion API.
            scene_name: Name of the scene to load the messages for, for example scene-0061.
        """
        self.scene_name = scene_name
        self._pose_times, self._pose = _load_can_bus_messages(can_bus, scene_name, "pose")
        self._monitor_times, self._monitor = _load_can_bus_messages(can_bus, scene_name, "vehicle_monitor")
        self._steering_times, steering = _load_can_bus_messages(can_bus, scene_name, "steeranglefeedback")

        self._steering_angles = np.array([message["value"] for message in steering], dtype=np.float64)
        if len(self._steering_times) > 1:
            # Differentiate the logged steering angle so that angle and rate stem from the same signal
            self._steering_rates = np.gradient(self._steering_angles, self._steering_times / 1e6)
        else:
            self._steering_rates = np.zeros_like(self._steering_angles)

        missing = [
            name
            for name, timestamps in (
                ("pose", self._pose_times),
                ("vehicle_monitor", self._monitor_times),
                ("steeranglefeedback", self._steering_times),
            )
            if len(timestamps) == 0
        ]
        if missing:
            print(
                f"Warning: scene {scene_name} has no CAN bus {', '.join(missing)} messages; "
                "the EgoData entries derived from them stay unset"
            )

    def fill_ego_data(self, ego_data_msg: EgoData, timestamp_micros: int) -> None:
        """Fill the CAN-bus-derived entries of an EgoData message for a single sample.

        Args:
            ego_data_msg: Message to fill; entries without CAN bus data are left untouched.
            timestamp_micros: Timestamp of the sample the message belongs to.
        """
        state = ego_data_msg.state

        index = _nearest_message_index(self._pose_times, timestamp_micros, _MAX_POSE_AGE_MICROS)
        if index is not None:
            pose = self._pose[index]
            # Velocity, acceleration and rotation rate are logged in the ego vehicle frame.
            # nuScenes only populates the longitudinal component of the velocity.
            state.continuous_state[EGO.VEL_LON] = float(pose["vel"][0])
            state.continuous_state[EGO.VEL_LAT] = float(pose["vel"][1])
            state.continuous_state[EGO.ACC_LON] = float(pose["accel"][0])
            state.continuous_state[EGO.ACC_LAT] = float(pose["accel"][1])
            state.continuous_state[EGO.YAW_RATE] = float(pose["rotation_rate"][2])
            state.discrete_state[EGO.STANDSTILL] = int(abs(pose["vel"][0]) < _STANDSTILL_VELOCITY)

        index = _nearest_message_index(self._steering_times, timestamp_micros, _MAX_STEERING_AGE_MICROS)
        if index is not None:
            state.continuous_state[EGO.STEERING_ANGLE_ACK] = float(self._steering_angles[index] / _STEERING_RATIO)
            state.continuous_state[EGO.STEERING_ANGLE_RATE_ACK] = float(self._steering_rates[index] / _STEERING_RATIO)

        index = _nearest_message_index(self._monitor_times, timestamp_micros, _MAX_VEHICLE_MONITOR_AGE_MICROS)
        if index is not None:
            monitor = self._monitor[index]
            state.discrete_state[EGO.TURN_INDICATOR] = _turn_indicator(monitor)
            state.discrete_state[EGO.BRAKE_LIGHT] = _brake_light(monitor)
            # The reverse light stays unknown: nuScenes logs a gear position without documenting
            # which of its values encodes the reverse gear


def _load_can_bus_messages(
    can_bus: NuScenesCanBus, scene_name: str, message_name: str
) -> Tuple[np.ndarray, List[Dict[str, Any]]]:
    """Return the timestamps and messages of one CAN bus message type, empty if it is unavailable."""
    try:
        # Some scenes have no CAN bus data at all, some lack individual message types
        messages = list(can_bus.get_messages(scene_name, message_name, print_warnings=False))
    except Exception:
        return np.empty(0), []
    timestamps = np.array([message["utime"] for message in messages], dtype=np.float64)
    return timestamps, messages


def _nearest_message_index(timestamps: np.ndarray, timestamp_micros: int, max_age_micros: int) -> Optional[int]:
    """Return the index of the message closest in time, or None if none is recorded close enough."""
    if len(timestamps) == 0:
        return None
    index = int(np.argmin(np.abs(timestamps - timestamp_micros)))
    if abs(timestamps[index] - timestamp_micros) > max_age_micros:
        return None
    return index


def _turn_indicator(vehicle_monitor: Dict[str, Any]) -> int:
    """Convert the logged indicator signals to an EGO turn indicator state."""
    left = bool(vehicle_monitor["left_signal"])
    right = bool(vehicle_monitor["right_signal"])
    if left and right:
        return EGO.TURN_INDICATOR_HAZARD
    if left:
        return EGO.TURN_INDICATOR_LEFT
    if right:
        return EGO.TURN_INDICATOR_RIGHT
    return EGO.TURN_INDICATOR_OFF


def _brake_light(vehicle_monitor: Dict[str, Any]) -> int:
    """Convert the logged brake switch to an EGO brake light state."""
    if vehicle_monitor["brake_switch"] == _BRAKE_SWITCH_RELEASED:
        return EGO.LIGHT_OFF
    return EGO.LIGHT_ON


class NuscenesAdapter(DatasetAdapter):
    """Converts nuScenes dataset files to ROS 2 messages."""

    VERSION = "1.1.0"
    RELEASE_NOTES = {
        "0.1.0": "Initial integration into Autonomy.Datasets",
        "1.0.0": "Create version subfolders, add velocity, acceleration, steering angle and lights info to EgoData, "
        "publish radar point clouds",
        "1.1.0": "Create Lanelet2 maps",
    }

    def __init__(
        self,
        data_publishers: Dict[str, Any],
        split: str,
        dataset_root_dir: str,
        publish_ego_data: bool = True,
        publish_camera_images: bool = False,
        publish_lidar_pointclouds: bool = False,
        publish_radar_pointclouds: bool = False,
        publish_lidar_object_lists: bool = True,
        publish_camera_01_object_lists: bool = True,
        min_lidar_points_in_bbox: int = 1,
        camera_box_visibility: BoxVisibility = BoxVisibility.ANY,
        camera_box_min_points: int = 1,
        start_scene_index: int = 0,
        generate_lanelet2_map: bool = True,
        lanelet2_lane_width: float = 3.0,
    ) -> None:
        """Initialize the nuScenes dataset adapter.

        Args:
            data_publishers: Mapping of topic names to publisher instances.
            split: Dataset split name (for example, mini_train, mini_val, train, val).
            dataset_root_dir: Root directory of the extracted nuScenes dataset.
            publish_camera_images: Whether to publish camera image data.
            publish_lidar_pointclouds: Whether to publish lidar point cloud data.
            publish_radar_pointclouds: Whether to publish radar point cloud data.
            publish_ego_data: Whether to publish ego data.
            publish_lidar_object_lists: Whether to publish lidar object lists.
            publish_camera_01_object_lists: Whether to publish camera_01 (front) object lists.
            min_lidar_points_in_bbox: Minimum lidar points required for lidar object labels.
            camera_box_visibility: Required camera box visibility filter for annotations.
            camera_box_min_points: Minimum lidar+radar points required for camera object labels.
            start_scene_index: Number of scenes to skip before generating samples.
            generate_lanelet2_map: Whether to convert each scene's nuScenes map.
            lanelet2_lane_width: Assumed lane width in meters.
        """

        super().__init__(data_publishers=data_publishers)
        self.split = split

        self.publish_ego_data = publish_ego_data
        self.publish_camera_images = publish_camera_images
        self.publish_lidar_pointclouds = publish_lidar_pointclouds
        self.publish_radar_pointclouds = publish_radar_pointclouds
        self.publish_lidar_object_lists = publish_lidar_object_lists
        self.publish_camera_01_object_lists = publish_camera_01_object_lists
        self.start_scene_index = start_scene_index

        self.generate_lanelet2_map = generate_lanelet2_map
        self.lanelet2_lane_width = lanelet2_lane_width

        self._map_contents_cache: Dict[str, str] = {}

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

        # The CAN bus expansion holds the ego vehicle signals that the ego poses lack; it is
        # downloaded separately, so publish EgoData without them when it is not available
        self.can_bus: Optional[NuScenesCanBus] = None
        try:
            self.can_bus = NuScenesCanBus(dataroot=str(self.dataset_root_dir))
        except Exception as error:
            print(
                f"Warning: nuScenes CAN bus expansion not available ({error}); EgoData is published "
                "without velocity, acceleration, steering angle and indicator states"
            )

        # add publishers for outgoing messages, actual publisher will be created in AutonomyDatasets node
        if self.publish_ego_data:
            self.data_publishers["ego_data"] = None
        if self.publish_lidar_object_lists:
            self.data_publishers["object_list/lidar_01"] = None
        if self.publish_camera_01_object_lists:
            self.data_publishers["object_list/camera_01"] = None
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

    def _get_map_contents_for_scene(self, scene: Dict[str, Any]) -> str:
        """Return the Lanelet2 OSM map string for a scene's map location.

        Args:
            scene: A nuScenes scene record dict.

        Returns:
            The Lanelet2 map as an OSM XML string, or an empty string.
        """
        if not self.generate_lanelet2_map:
            return ""

        location = self.nusc.get("log", scene["log_token"])["location"]
        if location not in self._map_contents_cache:
            from nuscenes.map_expansion.map_api import NuScenesMap

            nusc_map = NuScenesMap(dataroot=str(self.dataset_root_dir), map_name=location)
            self._map_contents_cache[location] = nuscenes_map_to_lanelet2_osm(
                nusc_map,
                location=location,
                lane_width=self.lanelet2_lane_width,
            )
            print(f"Converted nuScenes map '{location}' to Lanelet2")

        return self._map_contents_cache[location]

    def _get_map_origin_for_scene(self, scene: Dict[str, Any]) -> Tuple[float, float]:
        """Return the (lat, lon) geographic origin of a scene's map location.

        This matches the origin used to project the Lanelet2 map, so the map
        server can anchor the map correctly.

        Args:
            scene: A nuScenes scene record dict.

        Returns:
            The ``(origin_lat, origin_lon)`` origin in WGS84 degrees.
        """
        location = self.nusc.get("log", scene["log_token"])["location"]
        return get_location_origin(location)

    def generate_samples(self) -> Iterator[Tuple[int, Dict[str, Any]]]:
        """Yield sequential sample indices and ROS-ready sample payloads for the configured nuScenes split."""
        scene_splits = create_splits_scenes()
        count_examples = 0
        skipped_scene_count = 0
        for scene in self.nusc.scene:
            if scene["name"] in scene_splits[self.split]:
                if skipped_scene_count < self.start_scene_index:
                    skipped_scene_count += 1
                    print(f"Skipping already stored scene {skipped_scene_count}: {scene['token']}")
                    continue

                map_contents = self._get_map_contents_for_scene(scene)
                map_origin_lat, map_origin_lon = self._get_map_origin_for_scene(scene)

                scene_id = scene["token"]
                scene_can_bus = _SceneCanBus(self.can_bus, scene["name"]) if self.can_bus is not None else None
                instance_id_map: Dict[str, int] = {}
                sample_token = scene["first_sample_token"]
                while sample_token != "":
                    nusc_sample = self.nusc.get("sample", sample_token)
                    sample: Dict[str, Any] = {}
                    clock_msg = timestamp_micros_to_clock(int(nusc_sample["timestamp"]))

                    # Get ego pose via any sample_data record's ego_pose_token
                    sample_data_for_ego = self.nusc.get("sample_data", next(iter(nusc_sample["data"].values())))
                    ego_pose = self.nusc.get("ego_pose", sample_data_for_ego["ego_pose_token"])
                    ego_data_msg, tf_msg = _egomotion_to_ego_data(ego_pose, clock_msg.clock)
                    if scene_can_bus is not None:
                        scene_can_bus.fill_ego_data(ego_data_msg, int(nusc_sample["timestamp"]))

                    if self.publish_ego_data:
                        sample["ego_data"] = ego_data_msg

                    if self.publish_lidar_pointclouds or self.publish_lidar_object_lists:
                        sample_data_lidar_top_token = nusc_sample["data"]["LIDAR_TOP"]
                        pcl_path, annotations, _ = self.nusc.get_sample_data(sample_data_lidar_top_token)

                        if self.publish_lidar_pointclouds:
                            # Lidar point cloud in nuScenes frame (x=right, y=front, z=up)
                            scan = np.fromfile(pcl_path, dtype=np.float32).reshape((-1, 5))
                            lidar_msg = _get_lidar_point_cloud(scan, clock_msg.clock)
                            sample["lidar_01/point_cloud"] = lidar_msg

                        if self.publish_lidar_object_lists:
                            # Object list with meta information for evaluation
                            object_list = []
                            for ann in annotations:
                                sample_annotation = self.nusc.get("sample_annotation", ann.token)
                                num_lidar_pts = sample_annotation["num_lidar_pts"]
                                num_radar_pts = sample_annotation["num_radar_pts"]
                                if num_lidar_pts >= self.min_lidar_points_in_bbox:
                                    instance_token = sample_annotation["instance_token"]
                                    if instance_token not in instance_id_map:
                                        instance_id_map[instance_token] = len(instance_id_map)
                                    attributes = []
                                    for attribute_token in sample_annotation["attribute_tokens"]:
                                        attributes.append(self.nusc.get("attribute", attribute_token)["name"])
                                    object_list.append(
                                        (ann, num_lidar_pts, num_radar_pts, attributes, instance_id_map[instance_token])
                                    )
                            object_list_msg = _labels_to_object_list(object_list, "lidar_top", clock_msg.clock, scene_id)
                            sample["object_list/lidar_01"] = object_list_msg

                    if self.publish_camera_images:
                        for sensor_feature, topic in _SENSOR_FEATURE_TO_TOPIC.items():
                            if not topic.startswith("camera_") or sensor_feature not in nusc_sample["data"]:
                                continue

                            sample_data_token = nusc_sample["data"][sensor_feature]
                            sample_data = self.nusc.get("sample_data", sample_data_token)
                            image_path, _, camera_intrinsic = self.nusc.get_sample_data(sample_data_token)
                            camera_intrinsic = np.asarray(camera_intrinsic, dtype=np.float64)
                            camera_frame_id = _SENSOR_FEATURE_TO_FRAME_ID[sensor_feature]

                            sample[f"{topic}/image_raw"] = _image_path_to_ros_msg(image_path, clock_msg.clock, camera_frame_id)
                            sample[f"{topic}/camera_info"] = _camera_intrinsic_to_camera_info_msg(
                                camera_intrinsic,
                                sample_data["width"],
                                sample_data["height"],
                                clock_msg.clock,
                                camera_frame_id,
                            )

                    if self.publish_radar_pointclouds:
                        for sensor_feature, topic in _SENSOR_FEATURE_TO_TOPIC.items():
                            if not topic.startswith("radar_") or sensor_feature not in nusc_sample["data"]:
                                continue

                            radar_path, _, _ = self.nusc.get_sample_data(nusc_sample["data"][sensor_feature])
                            sample[f"{topic}/point_cloud"] = _get_radar_point_cloud(
                                radar_path,
                                clock_msg.clock,
                                _SENSOR_FEATURE_TO_FRAME_ID[sensor_feature],
                            )

                    if self.publish_camera_01_object_lists:
                        sample_data_cam_front_token = nusc_sample["data"]["CAM_FRONT"]
                        _, annotations, _ = self.nusc.get_sample_data(
                            sample_data_cam_front_token, box_vis_level=self.camera_box_visibility
                        )
                        camera_frame_id = _SENSOR_FEATURE_TO_FRAME_ID["CAM_FRONT"]

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

                            rot_cam = Rotation.from_quat([ann_q.q[1], ann_q.q[2], ann_q.q[3], ann_q.q[0]])
                            roll_cam, pitch_cam, yaw_cam = rot_cam.as_euler("xyz")

                            sample_object = (
                                instance_id_map[instance_token],
                                ann.name,
                                object_classification,
                                ann_x,
                                ann_y,
                                ann_z,
                                roll_cam,
                                pitch_cam,
                                yaw_cam,
                                ann_l,
                                ann_w,
                                ann_h,
                                num_pts,
                            )

                            object_list.append(sample_object)

                        sample["object_list/camera_01"] = _camera_labels_to_object_list(
                            object_list,
                            camera_frame_id,
                            clock_msg.clock,
                            scene_id,
                        )

                    # Build static TF messages from sensor calibration
                    tf_msgs = _build_tf_msgs(self.nusc, nusc_sample)

                    sample["scene_id"] = scene["token"]
                    sample["map_contents"] = map_contents
                    sample["map_origin_lat"] = map_origin_lat
                    sample["map_origin_lon"] = map_origin_lon
                    sample["scene_id"] = scene_id
                    sample["/clock"] = clock_msg
                    sample["/tf"] = tf_msg
                    sample["/tf_static"] = TFMessage(transforms=tf_msgs)

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
        calibrated_sensor = nusc.get("calibrated_sensor", sample_data["calibrated_sensor_token"])
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


def _labels_to_object_list(labels: List[Any], frame_id: str, stamp_msg: Time, scene_id: str) -> ObjectList:
    """Convert labels to a ROS ObjectList message."""
    object_list_msg = ObjectList()
    object_list_msg.header.frame_id = frame_id
    object_list_msg.header.stamp = stamp_msg
    objects: List[Object] = []

    for label, num_lidar_pts, num_radar_pts, attributes, instance_id in labels:
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

        # Meta information for evaluation
        if hasattr(obj_msg, "meta_info"):
            obj_msg.meta_info.append(f"scene_id:{scene_id}")
            obj_msg.meta_info.append(f"original_class:{label.name}")
            obj_msg.meta_info.append(f"num_lidar_pts:{num_lidar_pts}")
            obj_msg.meta_info.append(f"num_radar_pts:{num_radar_pts}")
            for attr in attributes:
                obj_msg.meta_info.append(f"attribute:{attr}")
        else:
            _warn_missing_meta_info_once()

        objects.append(obj_msg)

    object_list_msg.objects = objects
    return object_list_msg


def _camera_labels_to_object_list(labels: List[Any], frame_id: str, stamp_msg: Time, scene_id: str) -> ObjectList:
    """Convert camera annotations to a ROS ObjectList message."""
    object_list_msg = ObjectList()
    object_list_msg.header.frame_id = frame_id
    object_list_msg.header.stamp = stamp_msg
    objects: List[Object] = []

    for label in labels:
        obj_msg = Object()
        obj_msg.existence_probability = 1.0

        (
            instance_id,
            original_class,
            class_types,
            x_cam,
            y_cam,
            z_cam,
            roll_cam,
            pitch_cam,
            yaw_cam,
            length,
            width,
            height,
            num_pts,
        ) = label
        obj_msg.id = instance_id
        pmu.initialize_state(obj_msg.state, HEXAMOTION.MODEL_ID)
        obj_msg.state.continuous_state[HEXAMOTION.X] = float(x_cam)
        obj_msg.state.continuous_state[HEXAMOTION.Y] = float(y_cam)
        obj_msg.state.continuous_state[HEXAMOTION.Z] = float(z_cam)
        obj_msg.state.continuous_state[HEXAMOTION.ROLL] = float(roll_cam)
        obj_msg.state.continuous_state[HEXAMOTION.PITCH] = float(pitch_cam)
        obj_msg.state.continuous_state[HEXAMOTION.YAW] = float(yaw_cam)
        obj_msg.state.continuous_state[HEXAMOTION.LENGTH] = float(length)
        obj_msg.state.continuous_state[HEXAMOTION.WIDTH] = float(width)
        obj_msg.state.continuous_state[HEXAMOTION.HEIGHT] = float(height)
        obj_msg.state.discrete_state[HEXAMOTION.TURN_INDICATOR] = HEXAMOTION.TURN_INDICATOR_UNKNOWN
        obj_msg.state.discrete_state[HEXAMOTION.BRAKE_LIGHT] = HEXAMOTION.LIGHT_UNKNOWN
        obj_msg.state.discrete_state[HEXAMOTION.REVERSE_LIGHT] = HEXAMOTION.LIGHT_UNKNOWN

        obj_msg.state.classifications = [ObjectClassification(type=class_type, probability=1.0) for class_type in class_types]
        if hasattr(obj_msg, "meta_info"):
            obj_msg.meta_info.append(f"scene_id:{scene_id}")
            obj_msg.meta_info.append(f"original_class:{original_class}")
            obj_msg.meta_info.append(f"num_points:{num_pts}")
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


def _egomotion_to_ego_data(ego_pose: Dict[str, Any], stamp_msg: Time) -> Tuple[EgoData, TFMessage]:
    """Convert a nuScenes ego_pose record to a ROS EgoData message and TF.

    Args:
        ego_pose: nuScenes ego_pose record with 'translation' [x, y, z]
            and 'rotation' [w, x, y, z] quaternion.
        stamp_msg: ROS Time message.

    Returns:
        Tuple of (EgoData message, TFMessage with map->base_link transform).
    """
    tx, ty, tz = ego_pose["translation"]
    qw, qx, qy, qz = ego_pose["rotation"]

    ego_data_msg = EgoData()
    ego_data_msg.header.frame_id = "map"
    ego_data_msg.header.stamp = stamp_msg
    pmu.initialize_state(ego_data_msg.state, EGO.MODEL_ID)

    # Reference Point - nuScenes ego_pose is at the center of the rear axle on the ground
    # Renault Zoe: length=4.084m, rear_overhang=0.600m, height=1.562m
    # x: length/2 - rear_overhang = 1.442m forward to geometric center
    # z: height/2 = 0.781m up to geometric center
    ego_data_msg.state.reference_point = ObjectReferencePoint(
        value=ObjectReferencePoint.REAR_AXLE_GROUND,
        translation_to_geometric_center=Vector3(x=1.442, y=0.0, z=0.781),
    )

    # Position
    ego_data_msg.state.continuous_state[EGO.X] = float(tx)
    ego_data_msg.state.continuous_state[EGO.Y] = float(ty)
    ego_data_msg.state.continuous_state[EGO.Z] = float(tz)

    # Orientation: extract roll, pitch, yaw from quaternion
    rot = Rotation.from_quat([qx, qy, qz, qw])
    roll, pitch, yaw = rot.as_euler("xyz")
    ego_data_msg.state.continuous_state[EGO.ROLL] = float(roll)
    ego_data_msg.state.continuous_state[EGO.PITCH] = float(pitch)
    ego_data_msg.state.continuous_state[EGO.YAW] = float(yaw)

    # Dimensions - nuScenes ego vehicle is a Renault Zoe (not in dataset, known from docs)
    ego_data_msg.length = 4.084
    ego_data_msg.width = 1.730
    ego_data_msg.height = 1.562

    # Create TFMessage for ego pose in map frame
    tf_msg = TFMessage(
        transforms=[
            TransformStamped(
                header=Header(frame_id="map", stamp=stamp_msg),
                child_frame_id="base_link",
                transform=Transform(
                    translation=Vector3(
                        x=float(tx),
                        y=float(ty),
                        z=float(tz),
                    ),
                    rotation=Quaternion(
                        x=float(qx),
                        y=float(qy),
                        z=float(qz),
                        w=float(qw),
                    ),
                ),
            )
        ]
    )

    return ego_data_msg, tf_msg


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


def _get_radar_point_cloud(radar_path: str, stamp_msg: Time, frame_id: str) -> PointCloud2:
    """Convert a nuScenes radar detection file to a ROS PointCloud2 message.

    Args:
        radar_path: Path of the radar point cloud file of a single scan.
        stamp_msg: ROS Time message.
        frame_id: Frame the detections are given in.

    Returns:
        Point cloud with the fields (x, y, z, radial_velocity, rcs) in the radar sensor frame.
    """
    # The devkit applies its default filters, keeping only valid and unambiguous detections
    points = RadarPointCloud.from_file(radar_path).points
    x, y, z = points[0], points[1], points[2]
    rcs = points[5]

    # nuScenes reports the radial Doppler measurement decomposed into x and y, so project it
    # back onto the line of sight to obtain the measured radial velocity
    distance = np.hypot(x, y)
    radial_velocity = np.divide(
        points[6] * x + points[7] * y,
        distance,
        out=np.zeros_like(distance),
        where=distance > 0,
    )

    point_cloud = np.column_stack([x, y, z, radial_velocity, rcs]).astype(np.float32)

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="radial_velocity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="rcs", offset=16, datatype=PointField.FLOAT32, count=1),
    ]

    header = Header(frame_id=frame_id, stamp=stamp_msg)
    return create_cloud(header, fields, point_cloud)


def _image_path_to_ros_msg(image_path: str, stamp_msg: Time, frame_id: str) -> Image:
    """Load an image file and convert it to a ROS Image message."""
    img_array = cv2.imread(image_path, cv2.IMREAD_COLOR)
    if img_array is None:
        raise ValueError(f"Failed to read image: {image_path}")
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


def _camera_intrinsic_to_camera_info_msg(
    camera_intrinsic: np.ndarray,
    width: int,
    height: int,
    stamp_msg: Time,
    frame_id: str,
) -> CameraInfo:
    """Convert a nuScenes intrinsic matrix to a ROS CameraInfo message."""
    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = frame_id
    camera_info_msg.header.stamp = stamp_msg
    camera_info_msg.width = int(width)
    camera_info_msg.height = int(height)
    camera_info_msg.k = [
        float(camera_intrinsic[0, 0]),
        float(camera_intrinsic[0, 1]),
        float(camera_intrinsic[0, 2]),
        float(camera_intrinsic[1, 0]),
        float(camera_intrinsic[1, 1]),
        float(camera_intrinsic[1, 2]),
        float(camera_intrinsic[2, 0]),
        float(camera_intrinsic[2, 1]),
        float(camera_intrinsic[2, 2]),
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
        float(camera_intrinsic[0, 0]),
        float(camera_intrinsic[0, 1]),
        float(camera_intrinsic[0, 2]),
        0.0,
        float(camera_intrinsic[1, 0]),
        float(camera_intrinsic[1, 1]),
        float(camera_intrinsic[1, 2]),
        0.0,
        float(camera_intrinsic[2, 0]),
        float(camera_intrinsic[2, 1]),
        float(camera_intrinsic[2, 2]),
        0.0,
    ]

    return camera_info_msg
