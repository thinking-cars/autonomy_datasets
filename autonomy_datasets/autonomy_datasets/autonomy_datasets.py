import os
import sys
import select
import termios
import threading
import time
import tty
from typing import Any, Optional, Union

from sensor_msgs.msg import Image, PointCloud2
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, Duration, HistoryPolicy, QoSProfile, ReliabilityPolicy
import rclpy.exceptions
from rcl_interfaces.msg import (FloatingPointRange, IntegerRange, ParameterDescriptor, SetParametersResult)

from .datasets.nuscenes.nuscenes import NuscenesAdapter
from .datasets.waymo_open_dataset.waymo_open_dataset import WaymoOpenDatasetAdapter


class AutonomyDatasets(Node):

    def __init__(self):
        """Constructor"""
        super().__init__("autonomy_datasets")

        self.publisher_image = None
        self.publisher_point_cloud = None
        self.publisher_object_list = None

        self.auto_reconfigurable_params: list[str] = []
        self.datasets_path = self.declare_and_load_parameter(name="datasets_path",
                                                    param_type=rclpy.Parameter.Type.STRING,
                                                    description="path to datasets directory",
                                                    default="/datasets")
        self.dataset = self.declare_and_load_parameter(name="dataset",
                                                       param_type=rclpy.Parameter.Type.STRING,
                                                       description="name of the dataset to use",
                                                       default="waymo_open_dataset")
        self.dataset_config = self.declare_and_load_parameter(name="dataset_config",
                                                              param_type=rclpy.Parameter.Type.STRING,
                                                              description="configuration of the dataset to use",
                                                              default="lidar_objects")
        self.dataset_split = self.declare_and_load_parameter(name="dataset_split",
                                                             param_type=rclpy.Parameter.Type.STRING,
                                                             description="split of the dataset to use",
                                                             default="validation")
        self.publish_images = self.declare_and_load_parameter(name="publish_images",
                                                              param_type=rclpy.Parameter.Type.BOOL,
                                                              description="whether to publish images",
                                                              default=True)
        self.publish_point_clouds = self.declare_and_load_parameter(name="publish_point_clouds",
                                                                   param_type=rclpy.Parameter.Type.BOOL,
                                                                   description="whether to publish point clouds",
                                                                   default=True)
        self.publish_object_lists = self.declare_and_load_parameter(name="publish_object_lists",
                                                                    param_type=rclpy.Parameter.Type.BOOL,
                                                                    description="whether to publish object lists",
                                                                    default=False)

        self.setup()

    def declare_and_load_parameter(self,
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
        additional_constraints: str = "") -> Any:
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

    def parameters_callback(self,
                           parameters: list[rclpy.Parameter]) -> SetParametersResult:
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
        """Sets up subscribers, publishers, etc. to configure the node"""

        # callback for dynamic parameter configuration
        self.add_on_set_parameters_callback(self.parameters_callback)

        # publisher for publishing outgoing messages
        publisher_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        if self.publish_images:
            self.publisher_image = self.create_publisher(Image,
                                                        "~/image",
                                                        qos_profile=publisher_qos_profile)
            self.get_logger().info(f"Publishing images to '{self.publisher_image.topic_name}'")
        else:
            self.publisher_image = None
        if self.publish_point_clouds:
            self.publisher_point_cloud = self.create_publisher(PointCloud2,
                                                               "~/point_cloud",
                                                               qos_profile=publisher_qos_profile)
            self.get_logger().info(f"Publishing point clouds to '{self.publisher_point_cloud.topic_name}'")
        else:
            self.publisher_point_cloud = None
        # if self.publish_object_lists:
        #     self.publisher_object_list = self.create_publisher(ObjectList,
        #                                                     "~/object_list",
        #                                                     qos_profile=publisher_qos_profile)
        #     self.get_logger().info(f"Publishing object lists to '{self.publisher_object_list.topic_name}'")
        # else:
        #     self.publisher_object_list = None

        if self.dataset == "waymo_open_dataset":
            dataset_handler = WaymoOpenDatasetAdapter(os.path.join(self.datasets_path, 'waymo_open_dataset'))
        elif self.dataset == "nuscenes":
            dataset_handler = NuscenesAdapter(os.path.join(self.datasets_path, 'nuscenes'))
        else:
            self.get_logger().fatal(f"Unsupported dataset: {self.dataset}")
            raise SystemExit(1)

        self.publish_data(dataset_handler)

    def _start_key_listener(self):
        """Starts a background thread that listens for keyboard input.

        Space toggles pause, right arrow steps one iteration while paused.
        """
        self._paused = False
        self._step_event = threading.Event()
        self._stop_listener = threading.Event()

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
                            if b == ord(' '):
                                self._paused = not self._paused
                                state = "PAUSED" if self._paused else "RUNNING"
                                self.get_logger().info(f"Playback {state} (press space to toggle, right arrow to step)")
                                idx += 1
                            elif b == 0x1b and idx + 2 < len(data) and data[idx + 1] == ord('[') and data[idx + 2] == ord('C'):
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
        self._key_thread.join(timeout=1.0)

    def _wait_if_paused(self):
        """Blocks while paused. Unblocks on unpause (space) or single step (right arrow)."""
        while self._paused:
            if self._step_event.wait(timeout=0.1):
                self._step_event.clear()
                return

    def publish_data(self, dataset_handler):
        """Publishes data from the dataset"""

        self.get_logger().info("Waiting for subscribers to connect to publishers...")
        while True:
            all_connected = True
            if self.publisher_image and self.publisher_image.get_subscription_count() == 0:
                all_connected = False
            if self.publisher_point_cloud and self.publisher_point_cloud.get_subscription_count() == 0:
                all_connected = False
            # if self.publisher_object_list and self.publisher_object_list.get_subscription_count() == 0:
            #     all_connected = False

            if all_connected:
                break

            time.sleep(1.0)

        self._start_key_listener()
        self.get_logger().info("Playback controls: SPACE = pause/resume, RIGHT ARROW = step (while paused)")

        try:
            for sample_idx, sample in dataset_handler.generate_samples(self.dataset_config, self.dataset_split):
                self._wait_if_paused()

                self.get_logger().debug(f"Publishing sample {sample_idx}")
                if self.publisher_image:
                    self.get_logger().debug("Publishing image")
                    self.publisher_image.publish(Image())
                if self.publisher_point_cloud:
                    self.get_logger().debug("Publishing point cloud")
                    self.publisher_point_cloud.publish(sample["point_cloud"])
                # if self.publisher_object_list:
                #     self.get_logger().info("Publishing object list")
                #     self.publisher_object_list.publish(ObjectList())
                
                self.get_logger().debug("Waiting for all subscribers to acknowledge receipt of message...")
                all_acknowledged = False
                while not all_acknowledged:
                    all_acknowledged = True
                    if self.publisher_image and self.publisher_image.get_subscription_count() > 0:
                        all_acknowledged = all_acknowledged and self.publisher_image.wait_for_all_acked(Duration(seconds=1.0))
                    if self.publisher_point_cloud and self.publisher_point_cloud.get_subscription_count() > 0:
                        all_acknowledged = all_acknowledged and self.publisher_point_cloud.wait_for_all_acked(Duration(seconds=1.0))
                    # if self.publisher_object_list and self.publisher_object_list.get_subscription_count() > 0:
                    #     all_acknowledged = all_acknowledged and self.publisher_object_list.wait_for_all_acked(Duration(seconds=1.0))
                self.get_logger().debug("All subscribers acknowledged receipt of message")
        finally:
            self._stop_key_listener()
        
        self.get_logger().info("Finished publishing all samples")

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


if __name__ == '__main__':
    main()
