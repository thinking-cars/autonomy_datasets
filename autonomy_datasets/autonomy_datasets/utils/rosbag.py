import os

import rosbag2_py
from perception_msgs.msg import ObjectList
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_msgs.msg import TFMessage

MSG_TYPE_MAP = {
    "rosgraph_msgs/msg/Clock": Clock,
    "tf2_msgs/msg/TFMessage": TFMessage,
    "perception_msgs/msg/ObjectList": ObjectList,
    "sensor_msgs/msg/Image": Image,
    "sensor_msgs/msg/CameraInfo": CameraInfo,
    "sensor_msgs/msg/PointCloud2": PointCloud2,
}


def find_existing_rosbags(dataset_path: str, dataset: str, dataset_split: str) -> list[str]:
    """Returns sorted paths of all existing rosbag directories for the given dataset and split."""
    bag_root_dir = os.path.join(dataset_path, "bags")
    if not os.path.isdir(bag_root_dir):
        return []
    prefix = f"{dataset}_{dataset_split}_"
    return sorted([
        os.path.join(bag_root_dir, d)
        for d in os.listdir(bag_root_dir)
        if d.startswith(prefix) and os.path.isdir(os.path.join(bag_root_dir, d))
    ])


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


def get_bag_topic_types(bag_path: str, msg_type_map: dict | None = None) -> dict:
    """Reads topic metadata from a rosbag and returns a topic-name to msg-class mapping.

    Topics whose type is not present in msg_type_map are silently omitted.
    Defaults to MSG_TYPE_MAP when msg_type_map is not provided.
    """
    if msg_type_map is None:
        msg_type_map = MSG_TYPE_MAP
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="",
            output_serialization_format="",
        ),
    )
    topic_type_map = {}
    for topic_meta in reader.get_all_topics_and_types():
        msg_class = msg_type_map.get(topic_meta.type)
        if msg_class is not None:
            topic_type_map[topic_meta.name] = msg_class
    del reader
    return topic_type_map
