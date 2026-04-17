# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

import os
import sys
import select
import termios
import threading
import time
import tty
from typing import Any, Optional, Union

from ament_index_python import get_package_share_directory
from perception_msgs.msg import EgoData, ObjectList
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_msgs.msg import TFMessage
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import (
    DurabilityPolicy,
    Duration,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
import rclpy.exceptions
from rclpy.serialization import serialize_message
from rcl_interfaces.msg import (
    FloatingPointRange,
    IntegerRange,
    ParameterDescriptor,
    SetParametersResult,
)
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from .datasets.nuscenes.nuscenes import NuscenesAdapter
from .datasets.waymo_open_dataset.waymo_open_dataset import WaymoOpenDatasetAdapter
from .datasets.nvidia_physicalai_av_dataset.nvidia_physicalai_av_dataset import (
    NvidiaPhysicalAiAvDatasetAdapter,
)
from .datasets.rosbag.rosbag import (
    create_rosbag_writer,
    find_existing_rosbags,
    RosbagReplayAdapter,
)


class AutonomyDatasets(Node):
    def __init__(self):
        """Constructor"""
        super().__init__("autonomy_datasets")

        # Common parameters
        self.auto_reconfigurable_params: list[str] = []
        datasets_path = self.declare_and_load_parameter(
            name="datasets_path",
            param_type=rclpy.Parameter.Type.STRING,
            description="path to datasets directory",
            default="/datasets",
        )
        self.dataset = self.declare_and_load_parameter(
            name="dataset",
            param_type=rclpy.Parameter.Type.STRING,
            description="name of the dataset to use",
            default="waymo_open_dataset",
        )
        self.dataset_path = os.path.join(datasets_path, self.dataset)
        self.dataset_split = self.declare_and_load_parameter(
            name="dataset_split",
            param_type=rclpy.Parameter.Type.STRING,
            description="split of the dataset to use",
            default="validation_mini",
        )
        self.start_paused = self.declare_and_load_parameter(
            name="start_paused",
            param_type=rclpy.Parameter.Type.BOOL,
            description="whether to start playback in paused mode",
            default=False,
            add_to_auto_reconfigurable_params=False,
            read_only=True,
        )
        self.target_frame_rate = self.declare_and_load_parameter(
            name="target_frame_rate",
            param_type=rclpy.Parameter.Type.DOUBLE,
            description="playback speed multiplier based on recorded timestamps (1.0 = real-time, 2.0 = double speed, 0.0 = unlimited)",
            default=0.0,
            from_value=0.0,
            to_value=1000.0,
        )
        self.publish_samples = self.declare_and_load_parameter(
            name="publish_samples",
            param_type=rclpy.Parameter.Type.BOOL,
            description="whether to publish samples to ROS topics",
            default=True,
        )
        self.write_rosbag = self.declare_and_load_parameter(
            name="write_rosbag",
            param_type=rclpy.Parameter.Type.BOOL,
            description="whether to write samples to rosbag",
            default=True,
        )
        self.wait_for_ack = self.declare_and_load_parameter(
            name="wait_for_ack",
            param_type=rclpy.Parameter.Type.BOOL,
            description="whether to wait for subscriber acknowledgement after publishing",
            default=True,
        )
        self.use_camera = self.declare_and_load_parameter(
            name="use_camera",
            param_type=rclpy.Parameter.Type.BOOL,
            description="whether to publish camera images",
            default=True,
        )
        self.use_lidar = self.declare_and_load_parameter(
            name="use_lidar",
            param_type=rclpy.Parameter.Type.BOOL,
            description="whether to publish lidar point clouds",
            default=True,
        )
        self.use_radar = self.declare_and_load_parameter(
            name="use_radar",
            param_type=rclpy.Parameter.Type.BOOL,
            description="whether to publish radar data",
            default=True,
        )
        self.object_model = self.declare_and_load_parameter(
            name="object_model",
            param_type=rclpy.Parameter.Type.STRING,
            description="model for object representation",
            default="HEXAMOTION",
        )

        # Waymo Open Dataset parameters
        if self.dataset == "waymo_open_dataset":
            self.waymo_lidar_object_list_filter_cam_front = self.declare_and_load_parameter(
                name="waymo_lidar_object_list_filter_cam_front",
                param_type=rclpy.Parameter.Type.BOOL,
                description="use only objects covered by front camera",
                default=False,
            )
            self.waymo_min_lidar_points_in_bbox = self.declare_and_load_parameter(
                name="waymo_min_lidar_points_in_bbox",
                param_type=rclpy.Parameter.Type.INTEGER,
                description="minimum number of lidar points required in a bounding box",
                default=1,
            )
        elif self.dataset == "nvidia_physicalai_av_dataset":
            self.nvidia_filter_countries = self.declare_and_load_parameter(
                name="nvidia_filter_countries",
                param_type=rclpy.Parameter.Type.STRING,
                description="comma-separated list of countries to include (e.g. 'germany,japan'); if empty, includes all countries",
                default=None,
            )
            if self.nvidia_filter_countries:
                self.nvidia_filter_countries = self.nvidia_filter_countries.split(",")
        else:
            pass

        self.setup()

    def declare_and_load_parameter(
        self,
        name: str,
        param_type: rclpy.Parameter.Type,
        description: str,
        default: Optional[Any] = None,
        add_to_auto_reconfigurable_params: bool = True,
        is_required: bool = False,
        read_only: bool = False,
        from_value: Optional[Union[int, float]] = None,
        to_value: Optional[Union[int, float]] = None,
        step_value: Optional[Union[int, float]] = None,
        additional_constraints: str = "",
    ) -> Any:
        """Declares and loads a ROS parameter

        Args:
            name (str): name
            param_type (rclpy.Parameter.Type): parameter type
            description (str): description
            default (Optional[Any], optional): default value
            add_to_auto_reconfigurable_params (bool, optional): enable reconfiguration of parameter
            is_required (bool, optional): whether failure to load parameter will stop node
            read_only (bool, optional): set parameter to read-only
            from_value (Optional[Union[int, float]], optional): parameter range minimum
            to_value (Optional[Union[int, float]], optional): parameter range maximum
            step_value (Optional[Union[int, float]], optional): parameter range step
            additional_constraints (str, optional): additional constraints description

        Returns:
            Any: parameter value
        """

        # declare parameter
        param_desc = ParameterDescriptor()
        param_desc.description = description
        param_desc.additional_constraints = additional_constraints
        param_desc.read_only = read_only
        if from_value is not None and to_value is not None:
            if param_type == rclpy.Parameter.Type.INTEGER:
                range = IntegerRange(from_value=from_value, to_value=to_value)
                if step_value is not None:
                    range.step = step_value
                param_desc.integer_range = [range]
            elif param_type == rclpy.Parameter.Type.DOUBLE:
                range = FloatingPointRange(from_value=from_value, to_value=to_value)
                if step_value is not None:
                    range.step = step_value
                param_desc.floating_point_range = [range]
            else:
                self.get_logger().warn(f"Parameter type of parameter '{name}' does not support specifying a range")
        self.declare_parameter(name, param_type, param_desc)

        # load parameter
        try:
            param = self.get_parameter(name).value
            self.get_logger().info(f"Loaded parameter '{name}': {param}")
        except rclpy.exceptions.ParameterUninitializedException:
            if is_required:
                self.get_logger().fatal(f"Missing required parameter '{name}', exiting")
                raise SystemExit(1)
            else:
                self.get_logger().warn(f"Missing parameter '{name}', using default value: {default}")
                param = default
                self.set_parameters([rclpy.Parameter(name=name, value=param)])

        # add parameter to auto-reconfigurable parameters
        if add_to_auto_reconfigurable_params:
            self.auto_reconfigurable_params.append(name)

        return param

    def parameters_callback(self, parameters: list[rclpy.Parameter]) -> SetParametersResult:
        """Handles reconfiguration when a parameter value is changed

        Args:
            parameters (list[rclpy.Parameter]): parameters

        Returns:
            SetParametersResult: parameter change result
        """

        for param in parameters:
            if param.name in self.auto_reconfigurable_params:
                setattr(self, param.name, param.value)
                self.get_logger().info(f"Reconfigured parameter '{param.name}' to: {param.value}")

        result = SetParametersResult()
        result.successful = True

        return result

    def setup(self):
        """Set up subscribers, publishers, etc. to configure the node."""
        # callback for dynamic parameter configuration
        self.add_on_set_parameters_callback(self.parameters_callback)

        # dictionary of topic name to publisher function, initialized with tf_static broadcaster
        # and populated with dataset-specific publishers in publish_data()
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.data_publishers: dict[str, Optional[Publisher]] = {
            "/clock": None,
            "/tf_static": None,
            "/tf": None,
        }

        # rosbag writer will be initalized in publish_data()
        self.rosbag_writer = None
        self.rosbag_topics = {}

        # start publishing samples form dataset
        self.publish_data()

    def initialize_rosbag(self, name: str):
        if self.rosbag_writer is not None:
            self.rosbag_writer.close()
            del self.rosbag_writer
        bag_root_dir = os.path.join(self.dataset_path, "bags")
        os.makedirs(bag_root_dir, exist_ok=True)
        bag_uri = os.path.join(
            bag_root_dir,
            f"{self.dataset}_{self.dataset_split}_{name}",
        )
        self.rosbag_writer = create_rosbag_writer(
            bag_uri,
            self.rosbag_topics,
            os.path.join(
                get_package_share_directory("autonomy_datasets"),
                "config",
                "mcap_storage_config.yaml",
            ),
        )

    def publish_data(self):
        """Publish data from the dataset."""
        # Check for existing rosbags
        existing_bags = find_existing_rosbags(self.dataset_path, self.dataset, self.dataset_split)
        if existing_bags:
            self.get_logger().info(f"Found {len(existing_bags)} existing rosbag(s), replaying instead of generating new samples")
            self.write_rosbag = False
            dataset_handler = RosbagReplayAdapter(rosbag_paths=existing_bags, data_publishers=self.data_publishers)
            sample_generator = dataset_handler.generate_samples()

        elif self.dataset == "waymo_open_dataset":
            assert self.waymo_lidar_object_list_filter_cam_front is not None and self.waymo_min_lidar_points_in_bbox is not None
            dataset_handler = WaymoOpenDatasetAdapter(
                data_publishers=self.data_publishers,
                dataset_path=self.dataset_path,
                split=self.dataset_split,
                object_model=self.object_model,
                use_camera=self.use_camera,
                use_lidar=self.use_lidar,
                lidar_min_points_in_bbox=self.waymo_min_lidar_points_in_bbox,
                lidar_object_list_filter_cam_front=self.waymo_lidar_object_list_filter_cam_front,
            )
            sample_generator = dataset_handler.generate_samples()
        elif self.dataset == "nuscenes":
            dataset_handler = NuscenesAdapter(
                data_publishers=self.data_publishers,
                split=self.dataset_split,
                object_model=self.object_model,
                use_camera=self.use_camera,
                use_lidar=self.use_lidar,
                dataset_root_dir=self.dataset_path,
                # TODO: add nuscenes parameters
            )
            sample_generator = dataset_handler.generate_samples()
        elif self.dataset == "nvidia_physicalai_av_dataset":
            dataset_handler = NvidiaPhysicalAiAvDatasetAdapter(
                data_publishers=self.data_publishers,
                split=self.dataset_split,
                object_model=self.object_model,
                use_camera=self.use_camera,
                use_lidar=self.use_lidar,
                use_radar=self.use_radar,
                filter_countries=self.nvidia_filter_countries,
            )
            sample_generator = dataset_handler.generate_samples()
        else:
            self.get_logger().fatal(f"Unsupported dataset: {self.dataset}")
            raise SystemExit(1)

        # create ros publishers for all topic keys in self.data_publishers
        for topic, publisher in self.data_publishers.items():
            if publisher is None:
                if topic == "/clock":
                    msg_type = Clock
                    msg_type_str = "rosgraph_msgs/msg/Clock"
                elif topic == "ego_data":
                    msg_type = EgoData
                    msg_type_str = "perception_msgs/msg/EgoData"
                elif topic == "/tf_static":
                    msg_type = TFMessage
                    msg_type_str = "tf2_msgs/msg/TFMessage"
                elif topic == "/tf":
                    msg_type = TFMessage
                    msg_type_str = "tf2_msgs/msg/TFMessage"
                elif "object_list" in topic:
                    msg_type = ObjectList
                    msg_type_str = "perception_msgs/msg/ObjectList"
                elif "image_raw" in topic:
                    msg_type = Image
                    msg_type_str = "sensor_msgs/msg/Image"
                elif "camera_info" in topic:
                    msg_type = CameraInfo
                    msg_type_str = "sensor_msgs/msg/CameraInfo"
                elif "lidar" in topic or "radar" in topic:
                    msg_type = PointCloud2
                    msg_type_str = "sensor_msgs/msg/PointCloud2"
                else:
                    raise ValueError(
                        f"Topic '{topic}' does not match expected patterns for object lists, camera data, or point clouds; defaulting to PointCloud2 message type"
                    )

                # create topic in rosbag
                self.rosbag_topics[topic] = msg_type_str
                # create publisher for all topics except /tf_static published by tf_static_broadcaster
                if topic == "/tf_static":
                    self.data_publishers[topic] = self.tf_static_broadcaster.pub_tf
                elif topic == "/tf":
                    self.data_publishers[topic] = self.tf_broadcaster.pub_tf
                else:
                    self.data_publishers[topic] = self.create_publisher(
                        msg_type,
                        topic,
                        qos_profile=QoSProfile(
                            reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.VOLATILE,
                            history=HistoryPolicy.KEEP_LAST,
                            depth=1,
                        ),
                    )
                    self.get_logger().info(f"Publishing '{topic}' to '{self.data_publishers[topic].topic_name}'")

        if self.wait_for_ack and self.publish_samples:
            self.get_logger().info("Waiting for subscribers to connect to publishers...")
            while True:
                all_connected = True
                for topic, publisher in self.data_publishers.items():
                    assert publisher is not None
                    if publisher.get_subscription_count() == 0:
                        all_connected = False
                        self.get_logger().debug(f"Waiting for subscribers to connect to '{topic}' (0 subscribers connected)")
                    else:
                        self.get_logger().debug(f"Publisher '{topic}' has no subscriber(s) connected")
                if all_connected:
                    break
                time.sleep(1.0)

        self._start_key_listener()
        self.get_logger().info("Playback controls: SPACE = pause/resume, RIGHT ARROW = step (while paused)")

        last_scene_id = -1
        scene_count = 0
        prev_clock_ns = None

        try:
            for sample_idx, sample in sample_generator:
                self._wait_if_paused()
                frame_start = time.monotonic()

                current_clock_ns = sample["/clock"].clock.sec * 1_000_000_000 + sample["/clock"].clock.nanosec

                self.get_logger().debug(f"Publishing sample {sample_idx}")

                if sample["scene_id"] != last_scene_id:
                    scene_count += 1
                    self.get_logger().info(f"Processing scene {scene_count}: {sample['scene_id']})")
                    if self.write_rosbag:
                        self.initialize_rosbag(f"{sample['scene_id']}")
                    last_scene_id = sample["scene_id"]

                # publish sample data
                for topic, publisher in self.data_publishers.items():
                    assert publisher is not None
                    msg = sample[topic]
                    if self.publish_samples:
                        publisher.publish(msg)
                    if self.write_rosbag:
                        assert self.rosbag_writer is not None
                        self.rosbag_writer.write(
                            topic,
                            serialize_message(msg),
                            sample["/clock"].clock.sec * 1_000_000_000 + sample["/clock"].clock.nanosec,
                        )

                if self.wait_for_ack and self.publish_samples:
                    self.get_logger().debug("Waiting for all subscribers to acknowledge receipt of message...")
                    all_acknowledged = False
                    while not all_acknowledged:
                        all_acknowledged = True
                        for topic, publisher in self.data_publishers.items():
                            assert publisher is not None
                            if publisher.get_subscription_count() > 0:
                                all_acknowledged = all_acknowledged and publisher.wait_for_all_acked(Duration(seconds=1.0))
                    self.get_logger().debug("All subscribers acknowledged receipt of message")

                if self.target_frame_rate > 0 and prev_clock_ns is not None:
                    frame_duration = (current_clock_ns - prev_clock_ns) / 1e9 / self.target_frame_rate
                    elapsed = time.monotonic() - frame_start
                    remaining = frame_duration - elapsed
                    if remaining > 0:
                        time.sleep(remaining)
                prev_clock_ns = current_clock_ns
        finally:
            self._stop_key_listener()
            if self.rosbag_writer is not None:
                self.rosbag_writer.close()
            del self.rosbag_writer

        self.get_logger().info("Finished publishing all samples")

    def _start_key_listener(self):
        """Start a background thread that listens for keyboard input.

        Space toggles pause, right arrow steps one iteration while paused.
        """
        self._paused = self.start_paused
        self._step_event = threading.Event()
        self._stop_listener = threading.Event()
        self._key_thread = None

        if not sys.stdin.isatty():
            self.get_logger().info(
                "No TTY detected — keyboard controls disabled (run directly, not via 'ros2 launch', to enable)"
            )
            return

        def listener():
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                while not self._stop_listener.is_set():
                    if select.select([fd], [], [], 0.1)[0]:
                        data = os.read(fd, 32)
                        if not data:
                            continue
                        # Process all bytes/sequences in the chunk
                        idx = 0
                        while idx < len(data):
                            b = data[idx]
                            if b == ord(" "):
                                self._paused = not self._paused
                                state = "PAUSED" if self._paused else "RUNNING"
                                self.get_logger().info(f"Playback {state} (press space to toggle, right arrow to step)")
                                idx += 1
                            elif b == 0x1B and idx + 2 < len(data) and data[idx + 1] == ord("[") and data[idx + 2] == ord("C"):
                                self._step_event.set()
                                idx += 3
                            else:
                                idx += 1
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        self._key_thread = threading.Thread(target=listener, daemon=True)
        self._key_thread.start()

    def _stop_key_listener(self):
        """Stop the keyboard listener thread."""
        self._stop_listener.set()
        if self._key_thread is not None:
            self._key_thread.join(timeout=1.0)

    def _wait_if_paused(self):
        """Block while paused. Unblocks on unpause (space) or single step (right arrow)."""
        while self._paused:
            if self._step_event.wait(timeout=0.1):
                self._step_event.clear()
                return


def main():

    rclpy.init()
    node = AutonomyDatasets()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
