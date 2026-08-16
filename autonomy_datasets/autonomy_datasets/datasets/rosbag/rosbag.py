# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

import hashlib
import os
import shutil
from typing import Any, Dict, Iterator, Optional, Tuple

import rosbag2_py
import rosbag2_py._storage as rosbag2_storage
import yaml
from perception_msgs.msg import EgoData, ObjectList
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_msgs.msg import TFMessage

MSG_TYPE_MAP = {
    "rosgraph_msgs/msg/Clock": Clock,
    "tf2_msgs/msg/TFMessage": TFMessage,
    "perception_msgs/msg/EgoData": EgoData,
    "perception_msgs/msg/ObjectList": ObjectList,
    "sensor_msgs/msg/Image": Image,
    "sensor_msgs/msg/CameraInfo": CameraInfo,
    "sensor_msgs/msg/PointCloud2": PointCloud2,
}

# A scene's map is not a topic, so it is stored as files next to the rosbag data: the map itself
# as a standalone OSM file and its origin as a small YAML file inside the rosbag directory. That
# way it is deleted, copied and moved together with the rosbag it belongs to.
MAP_METADATA_FILENAME = "map.yaml"
MAP_CONTENTS_FILENAME = "map.osm"

# Scenes recorded at the same location share the same map, which is tens of megabytes in size.
# Maps are therefore written to a store shared by all rosbags of a dataset version, named by
# their content hash, and hard-linked into each rosbag directory, so that each rosbag holds a
# regular map file but identical maps occupy disk space only once.
MAP_STORE_DIRNAME = "maps"


class RosbagReplayAdapter:
    """Dataset adapter for replaying samples from existing rosbags instead of generating new ones from raw data."""

    def __init__(self, rosbag_paths: list[str], data_publishers: dict[str, Any], restore_map: bool = True):
        """Initialize the adapter with existing rosbag paths and pre-register topic publishers.

        Args:
            rosbag_paths: Paths to rosbag directories to replay.
            data_publishers: Topic-to-publisher mapping with requested topics.
            restore_map: Whether to restore the map stored next to each rosbag.

        Raises:
            AssertionError: If no rosbag paths are provided.
        """
        self.rosbag_paths = rosbag_paths
        self.data_publishers = data_publishers
        self.restore_map = restore_map
        self.current_bag_index = 0
        self.topic_type_map = {}

        assert len(rosbag_paths) > 0, "RosbagReplayAdapter requires at least one existing bag to replay from"

        # Check if rosbag contains all requested topics (keys in self.data_publishers)
        for bag_path in self.rosbag_paths:
            reader = rosbag2_py.SequentialReader()
            reader.open(
                rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap"),
                rosbag2_py.ConverterOptions(input_serialization_format="", output_serialization_format=""),
            )
            bag_topics = {t.name for t in reader.get_all_topics_and_types()}
            missing_topics = set(self.data_publishers.keys()) - bag_topics
            if missing_topics:
                raise ValueError(
                    f"Rosbag '{bag_path}' is missing requested topics: {missing_topics}. " f"Available topics: {bag_topics}"
                )
            reader.close()
            del reader

    def generate_samples(self) -> Iterator[Tuple[int, Dict[str, Any]]]:
        """Generate samples as ROS messages from Rosbags.

        Yields:
            Tuple of (example_id, example_dict) containing ROS messages for each sample.
        """
        i = 0
        for bag_idx, bag_path in enumerate(self.rosbag_paths):
            scene_id = os.path.basename(bag_path)
            print(f"Replaying scene {bag_idx + 1}/{len(self.rosbag_paths)}: {scene_id}")

            reader = rosbag2_py.SequentialReader()
            reader.open(
                rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap"),
                rosbag2_py.ConverterOptions(input_serialization_format="", output_serialization_format=""),
            )

            if reader.get_metadata().duration == Duration(seconds=0):
                print(f"Warning: Rosbag '{bag_path}' has zero duration, skipping")
                continue

            # The map is stored next to the rosbag while it is generated and is added to every
            # sample of the scene here, so that replay restores the map parameters without
            # re-generating the map from the original dataset. It is left out entirely if map
            # publishing is disabled, so that no map is read from disk in the first place.
            map_fields = read_rosbag_map(bag_path) if self.restore_map else {}
            if map_fields:
                print(f"Restored map stored with scene '{scene_id}' (map_contents size={len(map_fields['map_contents'])})")
            # Fields that carry scene metadata instead of message data of a single sample
            metadata_fields = {"scene_id", *map_fields}

            last_timestamp = None
            sample = {
                "scene_id": scene_id,
                **map_fields,
            }
            while reader.has_next():
                topic, data, timestamp = reader.read_next()

                if topic not in self.data_publishers.keys():
                    continue  # skip topics that are not requested

                if last_timestamp is not None and timestamp != last_timestamp:
                    # yield complete sample before starting next one
                    complete_sample = sample
                    sample = {
                        "scene_id": scene_id,
                        **map_fields,
                    }
                    if complete_sample.keys() <= {"/clock", *metadata_fields}:
                        print(f"Warning: Sample {i} in scene '{scene_id}' incomplete, skipping")
                    else:
                        i += 1
                        yield i, complete_sample

                # store topic type for deserialization if not already known
                if topic not in self.topic_type_map:
                    topic_meta = next((t for t in reader.get_all_topics_and_types() if t.name == topic), None)
                    if topic_meta is not None:
                        self.topic_type_map[topic] = MSG_TYPE_MAP.get(topic_meta.type, None)
                    else:
                        print(f"Warning: Topic '{topic}' not found in rosbag '{bag_path}', skipping")
                        continue

                # store sample data for current timestamp
                sample[topic] = deserialize_message(data, self.topic_type_map[topic])
                last_timestamp = timestamp

            reader.close()
            del reader

        print("Finished replaying all rosbags")


def write_rosbag_map(bag_uri: str, map_contents: str, map_origin_lat: float, map_origin_lon: float) -> str:
    """Stores a scene's map next to the rosbag it belongs to.

    Args:
        bag_uri: Path of the rosbag directory the map belongs to.
        map_contents: Lanelet2 map of the scene as an OSM XML string.
        map_origin_lat: WGS84 latitude of the map origin.
        map_origin_lon: WGS84 longitude of the map origin.

    Returns:
        Path of the written map file.
    """
    os.makedirs(bag_uri, exist_ok=True)
    map_contents_path = os.path.join(bag_uri, MAP_CONTENTS_FILENAME)

    # Write the map to the shared store first, so that rosbags of scenes recorded at the same
    # location can link to the same file instead of storing the identical map multiple times
    stored_map_path = os.path.join(
        os.path.dirname(bag_uri),
        MAP_STORE_DIRNAME,
        f"{hashlib.sha1(map_contents.encode()).hexdigest()}.osm",
    )
    if not os.path.isfile(stored_map_path):
        os.makedirs(os.path.dirname(stored_map_path), exist_ok=True)
        with open(stored_map_path, "w") as map_file:
            map_file.write(map_contents)
    if os.path.exists(map_contents_path):
        os.remove(map_contents_path)
    try:
        os.link(stored_map_path, map_contents_path)
    except OSError:
        # Hard links are not supported by every file system; a copy is equivalent, just larger
        shutil.copyfile(stored_map_path, map_contents_path)

    with open(os.path.join(bag_uri, MAP_METADATA_FILENAME), "w") as metadata_file:
        yaml.safe_dump(
            {
                "map_contents_file": MAP_CONTENTS_FILENAME,
                "map_origin_lat": float(map_origin_lat),
                "map_origin_lon": float(map_origin_lon),
            },
            metadata_file,
        )
    return map_contents_path


def read_rosbag_map(bag_path: str) -> Dict[str, Any]:
    """Returns the map stored next to a rosbag as sample fields.

    Args:
        bag_path: Path of the rosbag directory to read the map of.

    Returns:
        The ``map_contents``, ``map_origin_lat`` and ``map_origin_lon`` sample fields, or an
        empty dict if the rosbag was recorded without a map.
    """
    metadata_path = os.path.join(bag_path, MAP_METADATA_FILENAME)
    if not os.path.isfile(metadata_path):
        return {}

    try:
        with open(metadata_path) as metadata_file:
            metadata = yaml.safe_load(metadata_file) or {}
        map_contents_path = os.path.join(bag_path, metadata.get("map_contents_file", MAP_CONTENTS_FILENAME))
        with open(map_contents_path) as map_file:
            map_contents = map_file.read()
        return {
            "map_contents": map_contents,
            "map_origin_lat": float(metadata.get("map_origin_lat", 0.0)),
            "map_origin_lon": float(metadata.get("map_origin_lon", 0.0)),
        }
    except (OSError, TypeError, ValueError, yaml.YAMLError) as error:
        print(f"Warning: Failed to read map stored with rosbag '{bag_path}' ({error}); replaying without map")
        return {}


def get_rosbag_root_dir(dataset_path: str, version: Optional[str] = None) -> str:
    """Returns the rosbag directory of a dataset version.

    Args:
        dataset_path: Path to the dataset directory.
        version: Version of the dataset adapter the rosbags belong to. Without a version, the
            unversioned legacy directory written before adapter version 1.0.0 is returned.
    """
    bag_root_dir = os.path.join(dataset_path, "bags")
    if version is None:
        return bag_root_dir
    return os.path.join(bag_root_dir, version)


def find_existing_rosbags(dataset_path: str, dataset: str, dataset_split: str, version: Optional[str] = None) -> list[str]:
    """Returns sorted paths of all existing rosbag directories for the given dataset, split, and version."""
    bag_root_dir = get_rosbag_root_dir(dataset_path, version)
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


def get_latest_stored_scene_index(existing_bags: list[str], dataset: str, dataset_split: str) -> int:
    """Return the highest stored 1-based scene index encoded in rosbag directory names."""
    if not existing_bags:
        return 0

    prefix = f"{dataset}_{dataset_split}_"
    stored_scene_indices = []
    for bag_path in existing_bags:
        bag_name = os.path.basename(bag_path)
        if not bag_name.startswith(prefix):
            continue

        scene_index, _, _ = bag_name[len(prefix) :].partition("_")
        if scene_index.isdigit():
            stored_scene_indices.append(int(scene_index))

    if stored_scene_indices:
        return max(stored_scene_indices)

    return len(existing_bags)


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
            offered_qos = [rosbag2_storage.QoS(100).reliable().transient_local()]
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
