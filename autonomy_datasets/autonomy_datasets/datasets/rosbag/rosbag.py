# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

import os
from typing import Any, Dict, Iterator, Tuple

import rosbag2_py
from perception_msgs.msg import EgoData, ObjectList
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_msgs.msg import TFMessage
from rclpy.serialization import deserialize_message

MSG_TYPE_MAP = {
    "rosgraph_msgs/msg/Clock": Clock,
    "tf2_msgs/msg/TFMessage": TFMessage,
    "perception_msgs/msg/EgoData": EgoData,
    "perception_msgs/msg/ObjectList": ObjectList,
    "sensor_msgs/msg/Image": Image,
    "sensor_msgs/msg/CameraInfo": CameraInfo,
    "sensor_msgs/msg/PointCloud2": PointCloud2,
}


class RosbagReplayAdapter:
    """Dataset adapter for replaying samples from existing rosbags instead of generating new ones from raw data.

    This is used to avoid regenerating rosbag samples on every run during development, which can be time-consuming for large datasets like Waymo Open Dataset.
    """

    def __init__(self, rosbag_paths: list[str], data_publishers: dict[str, Any]):
        self.rosbag_paths = rosbag_paths
        self.current_bag_index = 0
        self.reader = None
        self.topic_type_map = {}

        assert len(rosbag_paths) > 0, "RosbagReplayAdapter requires at least one existing bag to replay from"

        # initialize data_publishers for all topics based on first rosbag
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=self.rosbag_paths[0], storage_id="mcap"),
            rosbag2_py.ConverterOptions(
                input_serialization_format="",
                output_serialization_format="",
            ),
        )
        for topic_meta in reader.get_all_topics_and_types():
            data_publishers[topic_meta.name] = None
            self.topic_type_map[topic_meta.name] = MSG_TYPE_MAP.get(topic_meta.type, None)
        reader.close()
        del reader

    def generate_samples(self) -> Iterator[Tuple[int, Dict[str, Any]]]:
        """Generate samples as ROS messages from Rosbags.

        Yields:
            Tuple of (example_id, example_dict) containing ROS messages for each sample.
        """
        i = 0
        for bag_idx, bag_path in enumerate(self.rosbag_paths):
            scene_id = os.path.basename(bag_path).split("_")[-1]
            print(f"Replaying scene {bag_idx + 1}/{len(self.rosbag_paths)}: {os.path.basename(scene_id)}")

            reader = rosbag2_py.SequentialReader()
            reader.open(
                rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap"),
                rosbag2_py.ConverterOptions(input_serialization_format="", output_serialization_format=""),
            )

            last_timestamp = None
            sample = {
                "scene_id": scene_id,
            }
            while reader.has_next():
                topic, data, timestamp = reader.read_next()

                if topic not in self.topic_type_map:
                    raise ValueError(f"Topic '{topic}' in rosbag does not match expected topics")

                if last_timestamp is not None and timestamp != last_timestamp:
                    # yield complete sample before starting next one
                    complete_sample = sample
                    sample = {
                        "scene_id": scene_id,
                    }
                    i += 1
                    yield i, complete_sample

                # store sample data for current timestamp
                sample[topic] = deserialize_message(data, self.topic_type_map[topic])
                last_timestamp = timestamp

            reader.close()
            del reader

        print("Finished replaying all rosbags")


def find_existing_rosbags(dataset_path: str, dataset: str, dataset_split: str) -> list[str]:
    """Returns sorted paths of all existing rosbag directories for the given dataset and split."""
    bag_root_dir = os.path.join(dataset_path, "bags")
    if not os.path.isdir(bag_root_dir):
        return []
    prefix = f"{dataset}_{dataset_split}_"
    return sorted(
        [
            os.path.join(bag_root_dir, d)
            for d in os.listdir(bag_root_dir)
            if d.startswith(prefix) and os.path.isdir(os.path.join(bag_root_dir, d))
        ]
    )


def create_rosbag_writer(
    bag_uri: str,
    rosbag_topics: dict[str, str],
    storage_config_uri: str,
) -> rosbag2_py.SequentialWriter:
    """Creates, opens, and configures a SequentialWriter for the given bag URI and topics."""
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(
            uri=bag_uri,
            storage_id="mcap",
            storage_config_uri=storage_config_uri,
        ),
        rosbag2_py.ConverterOptions(
            input_serialization_format="",
            output_serialization_format="",
        ),
    )
    for topic_id, (topic, msg_type) in enumerate(rosbag_topics.items()):
        offered_qos = []
        if "/tf_static" in topic:
            offered_qos = [rosbag2_py._storage.QoS(100).reliable().transient_local()]
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=topic_id,
                name=topic,
                type=msg_type,
                serialization_format="cdr",
                offered_qos_profiles=offered_qos,
            )
        )
    return writer
