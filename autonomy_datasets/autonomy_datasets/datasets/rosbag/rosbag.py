# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

import fcntl
import os
import shutil
import tempfile
from typing import Any, Callable, Dict, Iterator, Optional, Tuple

import rosbag2_py
import rosbag2_py._storage as rosbag2_storage
import yaml
from autonomy_datasets.datasets.rosbag.s3 import (
    download_rosbag_infos,
    is_s3_uri,
    join_s3_uri,
    RosbagInfo,
    S3McapReader,
    S3Storage,
)
from autonomy_datasets.datasets.utils import DatasetLogger
from autonomy_datasets_msgs.msg import ObjectListMetaInfo
from perception_msgs.msg import EgoData, ObjectList
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_msgs.msg import TFMessage

LOGGER = DatasetLogger("rosbag")

MSG_TYPE_MAP = {
    "rosgraph_msgs/msg/Clock": Clock,
    "tf2_msgs/msg/TFMessage": TFMessage,
    "autonomy_datasets_msgs/msg/ObjectListMetaInfo": ObjectListMetaInfo,
    "perception_msgs/msg/EgoData": EgoData,
    "perception_msgs/msg/ObjectList": ObjectList,
    "sensor_msgs/msg/Image": Image,
    "sensor_msgs/msg/CameraInfo": CameraInfo,
    "sensor_msgs/msg/PointCloud2": PointCloud2,
}

# A scene's map is not a topic, so it is stored as files next to the rosbag data: the map itself
# as a standalone OSM file and its origin as a small YAML file inside the rosbag directory. That
# way it is deleted, copied and moved together with the rosbag it belongs to. Each rosbag holds
# its own copy of the map, even if scenes recorded at the same location share the same map, as
# links between files are not supported by every storage, e.g. not by S3.
MAP_METADATA_FILENAME = "map.yaml"
MAP_CONTENTS_FILENAME = "map.osm"

# Rosbags are written to this directory below the rosbag directory of a dataset version and only
# moved next to the other rosbags once their scene is complete, so that a rosbag of a scene that
# is interrupted, e.g. by the node being stopped, is never mistaken for a complete one.
PARTIAL_ROSBAGS_DIRNAME = "partial"

# Magic bytes an MCAP file starts and ends with; a file not ending with them has not been closed
MCAP_MAGIC = b"\x89MCAP0\r\n"

# Key of the rosbag's custom metadata holding the ID of the scene the rosbag was generated from, so
# that replaying it publishes the same scene ID as its generation did
SCENE_ID_METADATA_KEY = "scene_id"


class RosbagReplayAdapter:
    """Dataset adapter for replaying samples from existing rosbags instead of generating new ones from raw data.

    Rosbags are read from the local file system or, if their paths are S3 URIs, streamed from S3.
    In both cases, only the messages of the requested topics are read.
    """

    def __init__(
        self,
        rosbag_paths: list[str],
        data_publishers: dict[str, Any],
        restore_map: bool = True,
        request_stored_topics: bool = False,
        topic_filter: Optional[Callable[[str], bool]] = None,
        storage: Optional[S3Storage] = None,
    ):
        """Initialize the adapter with existing rosbag paths and pre-register topic publishers.

        Args:
            rosbag_paths: Paths to rosbag directories to replay, or S3 URIs of rosbag directories.
            data_publishers: Topic-to-publisher mapping with requested topics.
            restore_map: Whether to restore the map stored next to each rosbag.
            request_stored_topics: Whether to add all topics stored in the rosbags to the requested
                topics, which is required if the topics cannot be requested by the adapter of the
                dataset, as its raw data is not available.
            topic_filter: Returns whether a stored topic is added to the requested topics if
                ``request_stored_topics`` is set, e.g. a
                :class:`~autonomy_datasets.datasets.topic_filter.TopicFilter`; all stored topics
                are added if not given.
            storage: S3 storage holding the rosbags whose paths are S3 URIs.

        Raises:
            AssertionError: If no rosbag paths are provided.
        """
        self.data_publishers = data_publishers
        self.restore_map = restore_map
        self.storage = storage
        self.current_bag_index = 0

        assert len(rosbag_paths) > 0, "RosbagReplayAdapter requires at least one existing bag to replay from"
        assert storage is not None or not any(is_s3_uri(path) for path in rosbag_paths), "S3 rosbags require a storage"

        # corrupted rosbags are skipped, so that they do not break the replay of the remaining ones
        self.rosbag_paths = []
        # scene ID of each rosbag that is replayed
        self.scene_ids: dict[str, str] = {}
        # metadata of each rosbag that is replayed
        self.rosbag_infos: dict[str, RosbagInfo] = {}
        # stored topics that are not replayed, as the filter rejects them
        filtered_topics: set[str] = set()
        for bag_path, info, error in self._read_rosbag_infos(rosbag_paths):
            if info is None:
                LOGGER.warn(
                    f"Skipping corrupted rosbag '{bag_path}': {error}; "
                    "regenerate the rosbags with 'overwrite_rosbag:=true' to replay it"
                )
                continue
            if SCENE_ID_METADATA_KEY not in info.custom_data:
                LOGGER.warn(
                    f"Skipping rosbag '{bag_path}', as it was generated without storing its scene ID; "
                    "regenerate the rosbags with 'overwrite_rosbag:=true' to replay it"
                )
                continue
            bag_topics = set(info.topic_types)
            if request_stored_topics:
                for topic in bag_topics:
                    if topic_filter is None or topic_filter(topic):
                        self.data_publishers.setdefault(topic, None)
                    elif topic not in self.data_publishers:
                        filtered_topics.add(topic)
            else:
                # Check if rosbag contains all requested topics (keys in self.data_publishers)
                missing_topics = set(self.data_publishers.keys()) - bag_topics
                if missing_topics:
                    raise ValueError(
                        f"Rosbag '{bag_path}' is missing requested topics: {missing_topics}. " f"Available topics: {bag_topics}"
                    )
            self.rosbag_paths.append(bag_path)
            self.scene_ids[bag_path] = info.custom_data[SCENE_ID_METADATA_KEY]
            self.rosbag_infos[bag_path] = info
        if filtered_topics:
            LOGGER.info(f"Not replaying stored topics excluded by the topic filter: {', '.join(sorted(filtered_topics))}")

    def _read_rosbag_infos(self, rosbag_paths: list[str]) -> list[Tuple[str, Optional[RosbagInfo], Optional[ValueError]]]:
        """Returns the metadata of each rosbag, or the error that the rosbag is incomplete or cannot be read."""
        local_paths = [path for path in rosbag_paths if not is_s3_uri(path)]
        s3_uris = [path for path in rosbag_paths if is_s3_uri(path)]
        results: dict[str, Tuple[str, Optional[RosbagInfo], Optional[ValueError]]] = {}
        for bag_path in local_paths:
            try:
                results[bag_path] = (bag_path, rosbag_info_from_metadata(read_complete_rosbag_metadata(bag_path)), None)
            except ValueError as error:
                results[bag_path] = (bag_path, None, error)
        if s3_uris:
            assert self.storage is not None
            for result in download_rosbag_infos(self.storage, s3_uris, MCAP_MAGIC):
                results[result[0]] = result
        return [results[path] for path in rosbag_paths]

    def _read_messages(self, bag_path: str, topics: set[str]) -> Iterator[Tuple[str, bytes, int]]:
        """Yields (topic, serialized message, timestamp) of the requested topics of a rosbag."""
        if is_s3_uri(bag_path):
            assert self.storage is not None
            bytes_read = size = 0
            for relative_file_path in self.rosbag_infos[bag_path].relative_file_paths:
                reader = S3McapReader(self.storage, join_s3_uri(bag_path, relative_file_path), topics)
                yield from reader.iter_messages()
                bytes_read += reader.bytes_read
                size += reader.size
            LOGGER.debug(f"Streamed {bytes_read / 1e6:.1f} MB of {size / 1e6:.1f} MB of rosbag '{bag_path}'")
            return

        reader = rosbag2_py.SequentialReader()
        try:
            reader.open(
                rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap"),
                rosbag2_py.ConverterOptions(input_serialization_format="", output_serialization_format=""),
            )
            # lets the storage plugin skip the chunks of the rosbag that hold no requested topic
            reader.set_filter(rosbag2_py.StorageFilter(topics=sorted(topics)))
            while reader.has_next():
                yield reader.read_next()
            reader.close()
        finally:
            del reader

    def generate_samples(self) -> Iterator[Tuple[int, Dict[str, Any]]]:
        """Generate samples as ROS messages from Rosbags.

        Yields:
            Tuple of (example_id, example_dict) containing ROS messages for each sample.
        """
        i = 0
        for bag_idx, bag_path in enumerate(self.rosbag_paths):
            scene_id = self.scene_ids[bag_path]
            info = self.rosbag_infos[bag_path]
            LOGGER.info(f"Replaying scene {bag_idx + 1}/{len(self.rosbag_paths)}: {scene_id}")

            # The map is stored next to the rosbag while it is generated and is added to every
            # sample of the scene here, so that replay restores the map parameters without
            # re-generating the map from the original dataset. It is left out entirely if map
            # publishing is disabled, so that no map is read from disk in the first place.
            map_fields = read_rosbag_map(bag_path, self.storage) if self.restore_map else {}
            if map_fields:
                LOGGER.info(f"Restored map stored with scene '{scene_id}' (map_contents size={len(map_fields['map_contents'])})")
            elif self.restore_map:
                # 'publish_lanelet2_map' is enabled, but the rosbag holds no map to publish, e.g.
                # because it was recorded before map support or without the dataset's map data
                LOGGER.warn(
                    f"No map stored with scene '{scene_id}', replaying without map; " "re-record the rosbags to include the map"
                )
            # Fields that carry scene metadata instead of message data of a single sample
            metadata_fields = {"scene_id", *map_fields}

            # only the messages of the requested topics are read
            topics = {topic for topic in self.data_publishers.keys() if topic in info.topic_types}
            message_count = sum(info.topic_message_counts.get(topic, 0) for topic in topics)
            try:
                read_message_count = 0
                last_timestamp = None
                sample = {
                    "scene_id": scene_id,
                    **map_fields,
                }
                for topic, data, timestamp in self._read_messages(bag_path, topics):
                    read_message_count += 1

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
                            LOGGER.warn(f"Sample {i} in scene '{scene_id}' incomplete, skipping")
                        else:
                            i += 1
                            yield i, complete_sample

                    msg_type = MSG_TYPE_MAP.get(info.topic_types[topic])
                    if msg_type is None:
                        LOGGER.warn(f"Topic '{topic}' of rosbag '{bag_path}' has unsupported type '{info.topic_types[topic]}'")
                        continue

                    # store sample data for current timestamp
                    sample[topic] = deserialize_message(data, msg_type)
                    last_timestamp = timestamp
                # the MCAP storage plugin ends reading at corrupted data without raising an error
                if read_message_count < message_count:
                    LOGGER.warn(
                        f"Rosbag '{bag_path}' is corrupted, reading it ended after {read_message_count} of its "
                        f"{message_count} requested messages; skipped its remaining samples"
                    )
                elif last_timestamp is not None:
                    # the last sample is not followed by a sample that would yield it, so it is
                    # yielded once all messages of the rosbag have been read
                    if sample.keys() <= {"/clock", *metadata_fields}:
                        LOGGER.warn(f"Sample {i} in scene '{scene_id}' incomplete, skipping")
                    else:
                        i += 1
                        yield i, sample
            except Exception as error:
                # a rosbag that is corrupted within its data is only noticed while reading it
                LOGGER.warn(f"Failed to read rosbag '{bag_path}' ({error}); skipping its remaining samples")

        LOGGER.info("Finished replaying all rosbags")


def rosbag_info_from_metadata(metadata: rosbag2_py.BagMetadata) -> RosbagInfo:
    """Returns the metadata of a local rosbag required for replaying it."""
    return RosbagInfo(
        message_count=metadata.message_count,
        topic_types={topic.topic_metadata.name: topic.topic_metadata.type for topic in metadata.topics_with_message_count},
        topic_message_counts={topic.topic_metadata.name: topic.message_count for topic in metadata.topics_with_message_count},
        custom_data=dict(metadata.custom_data),
        relative_file_paths=list(metadata.relative_file_paths),
    )


def _write_map_atomically(map_path: str, map_contents: str):
    """Writes a map file so that it appears with its full contents or not at all.

    Maps are read back exactly as they are found, without verifying them. A write interrupted
    halfway, by a node being killed for instance, would therefore leave behind a truncated map
    that every later run keeps restoring. Writing the contents to a
    temporary file and renaming it into place only once they are complete lets the map appear in
    one step instead.

    Args:
        map_path: Path of the map file to write.
        map_contents: Lanelet2 map to write as an OSM XML string.
    """
    # placed next to the destination, so that renaming it does not cross file system boundaries
    partial_map_path = f"{map_path}.{os.getpid()}.partial"
    try:
        with open(partial_map_path, "w") as map_file:
            map_file.write(map_contents)
            # the rename must not be able to publish contents that are still buffered
            map_file.flush()
            os.fsync(map_file.fileno())
        os.replace(partial_map_path, map_path)
    finally:
        # a successful rename leaves nothing behind; an interrupted write leaves the partial file
        if os.path.exists(partial_map_path):
            os.remove(partial_map_path)


def write_rosbag_map(bag_uri: str, map_contents: str, map_origin_lat: float, map_origin_lon: float) -> str:
    """Stores a scene's map in the directory of the rosbag it belongs to.

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
    # replaces a map file hard-linked by an older version of the node instead of writing through
    # the link, which would change the maps of the other rosbags sharing it
    _write_map_atomically(map_contents_path, map_contents)

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


def read_rosbag_map(bag_path: str, storage: Optional[S3Storage] = None) -> Dict[str, Any]:
    """Returns the map stored next to a rosbag as sample fields.

    Args:
        bag_path: Path of the rosbag directory to read the map of, or its S3 URI.
        storage: S3 storage holding the rosbag if its path is an S3 URI.

    Returns:
        The ``map_contents``, ``map_origin_lat`` and ``map_origin_lon`` sample fields, or an
        empty dict if the rosbag was recorded without a map.
    """
    if is_s3_uri(bag_path):
        assert storage is not None, "S3 rosbags require a storage"

        def read_file(file_name: str) -> str:
            return storage.read(join_s3_uri(bag_path, file_name)).decode()

    else:

        def read_file(file_name: str) -> str:
            with open(os.path.join(bag_path, file_name)) as file:
                return file.read()

    try:
        metadata = yaml.safe_load(read_file(MAP_METADATA_FILENAME)) or {}
    except FileNotFoundError:
        return {}
    except (OSError, UnicodeDecodeError, yaml.YAMLError) as error:
        LOGGER.warn(f"Failed to read map stored with rosbag '{bag_path}' ({error}); replaying without map")
        return {}

    try:
        map_contents = read_file(metadata.get("map_contents_file", MAP_CONTENTS_FILENAME))
        return {
            "map_contents": map_contents,
            "map_origin_lat": float(metadata.get("map_origin_lat", 0.0)),
            "map_origin_lon": float(metadata.get("map_origin_lon", 0.0)),
        }
    except (OSError, AttributeError, TypeError, ValueError) as error:
        LOGGER.warn(f"Failed to read map stored with rosbag '{bag_path}' ({error}); replaying without map")
        return {}


def read_complete_rosbag_metadata(bag_path: str) -> rosbag2_py.BagMetadata:
    """Returns the metadata of a rosbag after verifying that the rosbag is complete.

    A rosbag is incomplete if writing it was interrupted: its metadata is only written once the
    rosbag is closed, and an MCAP file that has not been closed lacks its trailing magic bytes, but
    can otherwise still be opened and silently ends early when read.

    Args:
        bag_path: Path of the rosbag directory.

    Returns:
        Metadata of the rosbag.

    Raises:
        ValueError: If the rosbag is incomplete or cannot be read.
    """
    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(
            rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap"),
            rosbag2_py.ConverterOptions(input_serialization_format="", output_serialization_format=""),
        )
        metadata = reader.get_metadata()
    except RuntimeError as error:
        raise ValueError(f"cannot be opened ({error})") from error
    finally:
        del reader

    for relative_file_path in metadata.relative_file_paths:
        try:
            with open(os.path.join(bag_path, relative_file_path), "rb") as bag_file:
                bag_file.seek(-len(MCAP_MAGIC), os.SEEK_END)
                file_complete = bag_file.read() == MCAP_MAGIC
        except OSError as error:
            raise ValueError(f"file '{relative_file_path}' cannot be read ({error})") from error
        if not file_complete:
            raise ValueError(f"file '{relative_file_path}' is truncated")
    if metadata.message_count == 0:
        raise ValueError("holds no message")
    return metadata


def get_rosbag_root_dir(dataset_path: str, version: Optional[str] = None) -> str:
    """Returns the rosbag directory of a dataset version.

    Args:
        dataset_path: Path to the dataset directory, or its S3 URI.
        version: Version of the dataset adapter the rosbags belong to. Without a version, the
            unversioned legacy directory written before adapter version 1.0.0 is returned.
    """
    join = join_s3_uri if is_s3_uri(dataset_path) else os.path.join
    bag_root_dir = join(dataset_path, "bags")
    if version is None:
        return bag_root_dir
    return join(bag_root_dir, version)


def list_rosbag_versions(dataset_path: str, storage: Optional[S3Storage] = None) -> list[str]:
    """Returns the sorted names of the subdirectories of the rosbag directory of a dataset.

    These are the versions rosbags have been generated for, besides the directories of legacy
    rosbags generated before adapter version 1.0.0.

    Args:
        dataset_path: Path to the dataset directory, or its S3 URI.
        storage: S3 storage holding the rosbags if the dataset path is an S3 URI.
    """
    bags_dir = get_rosbag_root_dir(dataset_path)
    if is_s3_uri(bags_dir):
        assert storage is not None, "S3 rosbags require a storage"
        return [uri.rsplit("/", 1)[-1] for uri in storage.list_subdirectories(bags_dir)]
    if not os.path.isdir(bags_dir):
        return []
    return sorted(d for d in os.listdir(bags_dir) if os.path.isdir(os.path.join(bags_dir, d)))


def find_existing_rosbags(
    dataset_path: str,
    dataset: str,
    dataset_split: str,
    version: Optional[str] = None,
    storage: Optional[S3Storage] = None,
) -> list[str]:
    """Returns sorted paths of all existing rosbag directories for the given dataset, split, and version.

    Args:
        dataset_path: Path to the dataset directory, or its S3 URI.
        dataset: Name of the dataset.
        dataset_split: Split of the dataset.
        version: Version of the dataset adapter the rosbags belong to.
        storage: S3 storage holding the rosbags if the dataset path is an S3 URI, in which case
            the S3 URIs of the rosbag directories are returned.
    """
    bag_root_dir = get_rosbag_root_dir(dataset_path, version)
    prefix = f"{dataset}_{dataset_split}_"
    if is_s3_uri(bag_root_dir):
        assert storage is not None, "S3 rosbags require a storage"
        return [uri for uri in storage.list_subdirectories(bag_root_dir) if uri.rsplit("/", 1)[-1].startswith(prefix)]
    if not os.path.isdir(bag_root_dir):
        return []
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


def get_stored_scene_ids(existing_bags: list[str], dataset: str, dataset_split: str) -> set[str]:
    """Returns the IDs of the scenes stored in local rosbags.

    A scene ID is read from the metadata of its rosbag. Rosbags generated before the scene ID was
    stored in their metadata hold it in their name, after the scene index.

    Args:
        existing_bags: Paths of the local rosbag directories.
        dataset: Name of the dataset.
        dataset_split: Split of the dataset.
    """
    prefix = f"{dataset}_{dataset_split}_"
    metadata_io = rosbag2_py.MetadataIo()
    scene_ids = set()
    for bag_path in existing_bags:
        try:
            scene_id = metadata_io.read_metadata(bag_path).custom_data.get(SCENE_ID_METADATA_KEY)
        except RuntimeError:
            scene_id = None
        if scene_id is None:
            bag_name = os.path.basename(bag_path)
            scene_index, _, scene_id = bag_name[len(prefix) :].partition("_") if bag_name.startswith(prefix) else ("", "", "")
            if not scene_index.isdigit() or not scene_id:
                LOGGER.warn(f"Cannot determine the scene stored in rosbag '{bag_path}'; its scene is generated again")
                continue
        scene_ids.add(scene_id)
    return scene_ids


class PartialRosbagsDir:
    """Directory of the rosbags a process is writing, below the partial rosbags directory shared by all processes.

    Every process writing rosbags of a dataset version claims its own subdirectory of the shared
    partial rosbags directory, so that processes writing other splits or replaying the rosbags do
    not discard the rosbags it is writing. A process holds a lock on its subdirectory while it is
    claimed; the subdirectories nobody holds a lock on were left behind by processes that were
    killed, and are removed by the next process writing rosbags.
    """

    LOCK_FILENAME = ".lock"
    # prefix of a subdirectory being claimed, which is renamed once its lock is held
    _CLAIMING_PREFIX = ".claiming-"

    def __init__(self, root: str) -> None:
        """Initialize the directory below the shared partial rosbags directory, without claiming it yet.

        Args:
            root: Path of the partial rosbags directory shared by all processes.
        """
        self.root = root
        self.path: Optional[str] = None
        self._lock_fd: Optional[int] = None

    def claim(self) -> str:
        """Returns the path of the subdirectory of this process, claiming it if not done yet."""
        if self.path is not None:
            return self.path
        os.makedirs(self.root, exist_ok=True)
        # the subdirectory is locked before it gets the name under which other processes may remove it
        claiming_path = tempfile.mkdtemp(prefix=self._CLAIMING_PREFIX, dir=self.root)
        self._lock_fd = os.open(os.path.join(claiming_path, self.LOCK_FILENAME), os.O_RDWR | os.O_CREAT, 0o644)
        try:
            fcntl.flock(self._lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as error:
            # without locks, e.g. on file systems not supporting them, other processes keep the subdirectory
            LOGGER.warn(f"Cannot lock '{claiming_path}' ({error}); it is not removed if this process is killed")
        self.path = os.path.join(self.root, os.path.basename(claiming_path)[len(self._CLAIMING_PREFIX) :])
        os.rename(claiming_path, self.path)
        return self.path

    def release(self) -> None:
        """Removes the subdirectory of this process together with the rosbags left in it."""
        if self.path is not None:
            shutil.rmtree(self.path, ignore_errors=True)
            self.path = None
        if self._lock_fd is not None:
            os.close(self._lock_fd)
            self._lock_fd = None

    def remove_abandoned(self) -> None:
        """Removes the subdirectories left behind by processes that were killed, which nobody holds a lock on.

        Rosbags directly in the shared directory were left behind by versions of the node writing all
        rosbags there, and are removed as well.
        """
        if not os.path.isdir(self.root):
            return
        for entry in os.scandir(self.root):
            if entry.name.startswith(self._CLAIMING_PREFIX) or entry.path == self.path:
                continue
            if not entry.is_dir(follow_symlinks=False):
                continue
            try:
                lock_fd = os.open(os.path.join(entry.path, self.LOCK_FILENAME), os.O_RDWR)
            except FileNotFoundError:
                shutil.rmtree(entry.path, ignore_errors=True)
                continue
            except OSError:
                continue
            try:
                fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError:
                # the subdirectory is claimed by a running process, or its lock cannot be checked
                continue
            else:
                LOGGER.info(f"Removing rosbags left incomplete by a process that was stopped: '{entry.path}'")
                shutil.rmtree(entry.path, ignore_errors=True)
            finally:
                os.close(lock_fd)


def create_rosbag_writer(
    bag_uri: str,
    rosbag_topics: dict[str, str],
    storage_config_uri: str,
    scene_id: str,
) -> rosbag2_py.SequentialWriter:
    """Creates, opens, and configures a SequentialWriter for the given bag URI and topics.

    Args:
        bag_uri: Path of the rosbag directory to write.
        rosbag_topics: Mapping of topic names to their message types.
        storage_config_uri: Path of the MCAP storage configuration.
        scene_id: ID of the scene the rosbag is generated from, stored in the rosbag's metadata.
    """
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(
            uri=bag_uri,
            storage_id="mcap",
            storage_config_uri=storage_config_uri,
            custom_data={SCENE_ID_METADATA_KEY: scene_id},
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
