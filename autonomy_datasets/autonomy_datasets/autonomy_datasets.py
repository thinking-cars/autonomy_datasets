import os
import sys
import select
import termios
import threading
import time
import tty
from typing import Any, Optional, Union

from perception_msgs.msg import ObjectList
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    Duration,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
import rclpy.exceptions
from rcl_interfaces.msg import (
    FloatingPointRange,
    IntegerRange,
    ParameterDescriptor,
    SetParametersResult,
)
from tf2_ros import StaticTransformBroadcaster

from .datasets.nuscenes.nuscenes import NuscenesAdapter
from .datasets.waymo_open_dataset.waymo_open_dataset import WaymoOpenDatasetAdapter
from .datasets.nvidia_physicalai_av_dataset.nvidia_physicalai_av_dataset import NvidiaPhysicalAiAvDatasetAdapter


class AutonomyDatasets(Node):
    def __init__(self):
        """Constructor"""
        super().__init__("autonomy_datasets")

        self.data_publishers = {}
        self.publisher_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Common parameters
        self.auto_reconfigurable_params: list[str] = []
        self.datasets_path = self.declare_and_load_parameter(
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
            description="target frame rate for publishing samples in Hz (0.0 = unlimited)",
            default=0.0,
            from_value=0.0,
            to_value=1000.0,
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
                self.get_logger().warn(
                    f"Parameter type of parameter '{name}' does not support specifying a range"
                )
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
                self.get_logger().warn(
                    f"Missing parameter '{name}', using default value: {default}"
                )
                param = default
                self.set_parameters([rclpy.Parameter(name=name, value=param)])

        # add parameter to auto-reconfigurable parameters
        if add_to_auto_reconfigurable_params:
            self.auto_reconfigurable_params.append(name)

        return param

    def parameters_callback(
        self, parameters: list[rclpy.Parameter]
    ) -> SetParametersResult:
        """Handles reconfiguration when a parameter value is changed

        Args:
            parameters (list[rclpy.Parameter]): parameters

        Returns:
            SetParametersResult: parameter change result
        """

        for param in parameters:
            if param.name in self.auto_reconfigurable_params:
                setattr(self, param.name, param.value)
                self.get_logger().info(
                    f"Reconfigured parameter '{param.name}' to: {param.value}"
                )

        result = SetParametersResult()
        result.successful = True

        return result

    def setup(self):
        """Sets up subscribers, publishers, etc. to configure the node"""

        # callback for dynamic parameter configuration
        self.add_on_set_parameters_callback(self.parameters_callback)

        # publishers for static transformations to sensor frames and clock
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.publisher_clock = self.create_publisher(
            Clock, "/clock", qos_profile=QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
        )

        self.publish_data()

    def publish_data(self):
        """Publishes data from the dataset"""

        self.get_logger().info("Waiting for subscribers to connect to publishers...")
        while True:
            all_connected = True
            for topic, publisher in self.data_publishers.items():
                if publisher.get_subscription_count() == 0:
                    all_connected = False
                    self.get_logger().debug(
                        f"Waiting for subscribers to connect to '{topic}' (0 subscribers connected)"
                    )
                else:
                    self.get_logger().debug(
                        f"Publisher '{topic}' has {publisher.get_subscription_count()} subscriber(s) connected"
                    )
            if all_connected:
                break
            time.sleep(1.0)

        if self.dataset == "waymo_open_dataset":
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
                dataset_root_dir=self.dataset_path,
            )
            sample_generator = dataset_handler.generate_samples(
                split=self.dataset_split, config="lidar_objects"
            )
        elif self.dataset == "nvidia_physicalai_av_dataset":
            dataset_handler = NvidiaPhysicalAiAvDatasetAdapter(
                data_publishers=self.data_publishers,
                dataset_path=self.dataset_path,
                split=self.dataset_split,
                object_model=self.object_model,
                use_camera=self.use_camera,
                use_lidar=self.use_lidar,
                use_radar=self.use_radar
            )
            sample_generator = dataset_handler.generate_samples()
        else:
            self.get_logger().fatal(f"Unsupported dataset: {self.dataset}")
            raise SystemExit(1)
        
        # create ros publishers
        for topic, publisher in self.data_publishers.items():
            if publisher is None:
                if "object_list" in topic:
                    self.data_publishers[topic] = self.create_publisher(
                        ObjectList,
                        f"/autonomy_datasets/{topic}",
                        qos_profile=self.publisher_qos_profile,
                    )
                elif "image_raw" in topic:
                    self.data_publishers[topic] = self.create_publisher(
                        Image,
                        f"/autonomy_datasets/{topic}",
                        qos_profile=self.publisher_qos_profile,
                    )
                elif "camera_info" in topic:
                    self.data_publishers[topic] = self.create_publisher(
                        CameraInfo,
                        f"/autonomy_datasets/{topic}",
                        qos_profile=self.publisher_qos_profile,
                    )
                elif "lidar" in topic or "radar" in topic:
                    self.data_publishers[topic] = self.create_publisher(
                        PointCloud2,
                        f"/autonomy_datasets/{topic}",
                        qos_profile=self.publisher_qos_profile,
                    )
                else:
                    raise ValueError(f"Topic '{topic}' does not match expected patterns for object lists, camera data, or point clouds; defaulting to PointCloud2 message type")
                self.get_logger().info(
                    f"Publishing '{topic}' to '{self.data_publishers[topic].topic_name}'"
                )

        self._start_key_listener()
        self.get_logger().info(
            "Playback controls: SPACE = pause/resume, RIGHT ARROW = step (while paused)"
        )

        try:
            for sample_idx, sample in sample_generator:
                self._wait_if_paused()
                frame_start = time.monotonic()

                self.get_logger().debug(f"Publishing sample {sample_idx}")
                if "stamp" in sample:
                    clock_msg = Clock()
                    clock_msg.clock = sample["stamp"]
                    self.publisher_clock.publish(clock_msg)
                if "tf" in sample:
                    self.tf_static_broadcaster.sendTransform(sample["tf"])

                dataset_handler.publish_sample(sample)

                self.get_logger().debug(
                    "Waiting for all subscribers to acknowledge receipt of message..."
                )
                all_acknowledged = False
                while not all_acknowledged:
                    all_acknowledged = True
                    for topic, publisher in self.data_publishers.items():
                        if publisher.get_subscription_count() > 0:
                            all_acknowledged = (
                                all_acknowledged
                                and publisher.wait_for_all_acked(Duration(seconds=1.0))
                            )
                self.get_logger().debug(
                    "All subscribers acknowledged receipt of message"
                )

                if self.target_frame_rate > 0:
                    frame_duration = 1.0 / self.target_frame_rate
                    elapsed = time.monotonic() - frame_start
                    remaining = frame_duration - elapsed
                    if remaining > 0:
                        time.sleep(remaining)
        finally:
            self._stop_key_listener()

        self.get_logger().info("Finished publishing all samples")


    def _start_key_listener(self):
        """Starts a background thread that listens for keyboard input.

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
                                self.get_logger().info(
                                    f"Playback {state} (press space to toggle, right arrow to step)"
                                )
                                idx += 1
                            elif (
                                b == 0x1B
                                and idx + 2 < len(data)
                                and data[idx + 1] == ord("[")
                                and data[idx + 2] == ord("C")
                            ):
                                self._step_event.set()
                                idx += 3
                            else:
                                idx += 1
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        self._key_thread = threading.Thread(target=listener, daemon=True)
        self._key_thread.start()

    def _stop_key_listener(self):
        """Stops the keyboard listener thread."""
        self._stop_listener.set()
        if self._key_thread is not None:
            self._key_thread.join(timeout=1.0)

    def _wait_if_paused(self):
        """Blocks while paused. Unblocks on unpause (space) or single step (right arrow)."""
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
