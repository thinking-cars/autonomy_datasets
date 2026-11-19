# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for streaming rosbags from S3 and replaying only their requested topics.

The S3 server is replaced by a fake client serving a local directory, which records the bytes
it serves, so that the tests confirm which parts of a rosbag are downloaded.
"""

import io
import os
import shutil
import tempfile
import unittest

from autonomy_datasets.datasets.rosbag.rosbag import (
    create_rosbag_writer,
    find_existing_rosbags,
    get_rosbag_root_dir,
    list_rosbag_versions,
    RosbagReplayAdapter,
    write_rosbag_map,
)
from autonomy_datasets.datasets.rosbag.s3 import join_s3_uri, S3McapReader, S3Storage, split_s3_uri
from mcap.reader import make_reader
from rclpy.serialization import serialize_message
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_msgs.msg import TFMessage

NUM_SAMPLES = 5
DATASET = "nuscenes"
SPLIT = "mini_val"
VERSION = "1.0.0"
TOPICS = {
    "/clock": "rosgraph_msgs/msg/Clock",
    "/tf": "tf2_msgs/msg/TFMessage",
    "camera_01/image_raw": "sensor_msgs/msg/Image",
    "camera_01/camera_info": "sensor_msgs/msg/CameraInfo",
    "lidar_01/point_cloud": "sensor_msgs/msg/PointCloud2",
}
SENSOR_TOPICS = {"camera_01/image_raw", "camera_01/camera_info", "lidar_01/point_cloud"}
NON_SENSOR_TOPICS = set(TOPICS) - SENSOR_TOPICS
# sample fields that carry scene metadata instead of message data
METADATA_FIELDS = {"scene_id", "map_contents", "map_origin_lat", "map_origin_lon"}

STORAGE_CONFIG = os.path.join(os.path.dirname(__file__), "..", "config", "mcap_storage_config.yaml")


class ClientError(Exception):
    """Error raised by the fake client, shaped like the one of botocore."""

    def __init__(self, code: str):
        """Create an error with the given botocore error code."""
        super().__init__(code)
        self.response = {"Error": {"Code": code}}


class FakeS3Client:
    """S3 client serving the objects of a bucket from a local directory.

    Listings are returned in pages of two entries, so that pagination is exercised.
    """

    PAGE_SIZE = 2

    def __init__(self, root_dir: str):
        """Serve the buckets stored as subdirectories of root_dir."""
        self.root_dir = root_dir
        #: Number of bytes served by each request for object data
        self.served_bytes: list[int] = []

    def _path(self, bucket: str, key: str) -> str:
        return os.path.join(self.root_dir, bucket, key)

    def list_objects_v2(self, Bucket, Prefix="", Delimiter=None, ContinuationToken=None):
        """Return a page of the keys and common prefixes below Prefix, like the S3 API does."""
        bucket_dir = os.path.join(self.root_dir, Bucket)
        keys = sorted(
            os.path.relpath(os.path.join(directory, name), bucket_dir)
            for directory, _, names in os.walk(bucket_dir)
            for name in names
        )
        entries = []
        for key in keys:
            if not key.startswith(Prefix):
                continue
            if Delimiter and Delimiter in key[len(Prefix) :]:
                entry = ("prefix", Prefix + key[len(Prefix) :].split(Delimiter)[0] + Delimiter)
            else:
                entry = ("key", key)
            if entry not in entries:
                entries.append(entry)
        start = int(ContinuationToken or 0)
        page = entries[start : start + self.PAGE_SIZE]
        response = {
            "Contents": [{"Key": value} for kind, value in page if kind == "key"],
            "CommonPrefixes": [{"Prefix": value} for kind, value in page if kind == "prefix"],
            "IsTruncated": start + self.PAGE_SIZE < len(entries),
        }
        if response["IsTruncated"]:
            response["NextContinuationToken"] = str(start + self.PAGE_SIZE)
        return response

    def get_object(self, Bucket, Key, Range=None):
        """Return the data of an object, or of the byte range of it given by Range."""
        path = self._path(Bucket, Key)
        if not os.path.isfile(path):
            raise ClientError("NoSuchKey")
        with open(path, "rb") as file:
            data = file.read()
        size = len(data)
        if Range is None:
            self.served_bytes.append(size)
            return {"Body": io.BytesIO(data)}
        first, _, last = Range[len("bytes=") :].partition("-")
        if not first:
            start, end = max(0, size - int(last)), size
        else:
            start, end = int(first), size if not last else min(size, int(last) + 1)
        self.served_bytes.append(end - start)
        return {"Body": io.BytesIO(data[start:end]), "ContentRange": f"bytes {start}-{end - 1}/{size}"}


def sample_messages(index: int) -> dict:
    """Return the messages of a sample, with camera images and point clouds of realistic size."""
    clock = Clock()
    clock.clock.sec = index
    image = Image(height=400, width=640, encoding="rgb8", step=640 * 3)
    image.data = os.urandom(image.height * image.step)
    point_cloud = PointCloud2(height=1, width=20000, point_step=16, row_step=20000 * 16)
    point_cloud.data = os.urandom(point_cloud.row_step)
    return {
        "/clock": clock,
        "camera_01/image_raw": image,
        "/tf": TFMessage(),
        "camera_01/camera_info": CameraInfo(),
        "lidar_01/point_cloud": point_cloud,
    }


class RosbagS3TestBase(unittest.TestCase):
    """Writes rosbags to a local dataset directory and uploads them to a fake S3 bucket."""

    def setUp(self):
        """Create a local dataset directory and the S3 storage of a fake bucket to upload its rosbags to."""
        self.tmp_dir = tempfile.mkdtemp(prefix="rosbag_s3_")
        self.addCleanup(shutil.rmtree, self.tmp_dir)
        self.dataset_path = os.path.join(self.tmp_dir, "local", DATASET)
        self.bag_root_dir = get_rosbag_root_dir(self.dataset_path, VERSION)
        self.client = FakeS3Client(os.path.join(self.tmp_dir, "s3"))
        self.storage = S3Storage(client=self.client)
        self.s3_dataset_uri = f"s3://bucket/datasets/{DATASET}"

    def write_rosbag(self, scene_index: int, storage_config: str = STORAGE_CONFIG, with_map: bool = False) -> str:
        """Write a rosbag of NUM_SAMPLES samples and return its path."""
        scene_id = f"scene-{scene_index:04d}"
        bag_path = os.path.join(self.bag_root_dir, f"{DATASET}_{SPLIT}_{scene_index:05d}_{scene_id}")
        writer = create_rosbag_writer(bag_path, TOPICS, storage_config, scene_id=scene_id)
        for index in range(NUM_SAMPLES):
            for topic, message in sample_messages(index).items():
                writer.write(topic, serialize_message(message), index * 1_000_000_000)
        writer.close()
        del writer
        if with_map:
            write_rosbag_map(bag_path, "<osm/>", 50.78, 6.06)
        return bag_path

    def upload(self):
        """Upload the local dataset directory to the fake bucket."""
        bucket, key = split_s3_uri(self.s3_dataset_uri)
        shutil.copytree(self.dataset_path, os.path.join(self.client.root_dir, bucket, key), dirs_exist_ok=True)

    def replay(self, bag_paths: list[str], topics: set[str], **kwargs) -> list[dict]:
        """Replay the given topics of rosbags and return the replayed samples."""
        adapter = RosbagReplayAdapter(bag_paths, dict.fromkeys(topics), storage=self.storage, **kwargs)
        return [sample for _, sample in adapter.generate_samples()]


class TestRosbagS3Streaming(RosbagS3TestBase):
    """Rosbags stored in S3 are replayed like local ones, downloading only the requested topics."""

    def test_writes_one_message_per_chunk(self):
        """Rosbags are written with a chunk per message, each indexing the topic of its message."""
        bag_path = self.write_rosbag(1)
        with open(os.path.join(bag_path, f"{os.path.basename(bag_path)}_0.mcap"), "rb") as mcap_file:
            summary = make_reader(mcap_file).get_summary()
        self.assertEqual(summary.statistics.chunk_count, summary.statistics.message_count)
        self.assertTrue(all(len(chunk.message_index_offsets) == 1 for chunk in summary.chunk_indexes))

    def test_finds_rosbags_in_s3(self):
        """Rosbags are found in S3 with the layout of the local dataset directory."""
        bag_paths = [self.write_rosbag(index) for index in (1, 2, 3)]
        self.upload()
        s3_bags = find_existing_rosbags(self.s3_dataset_uri, DATASET, SPLIT, VERSION, storage=self.storage)
        self.assertEqual([uri.rsplit("/", 1)[-1] for uri in s3_bags], [os.path.basename(path) for path in bag_paths])
        self.assertEqual(find_existing_rosbags(self.s3_dataset_uri, DATASET, "train", VERSION, storage=self.storage), [])
        self.assertEqual(list_rosbag_versions(self.s3_dataset_uri, self.storage), [VERSION])

    def test_replays_like_local_rosbags(self):
        """Samples streamed from S3 equal the samples replayed from the local rosbag, including the map."""
        bag_path = self.write_rosbag(1, with_map=True)
        self.upload()
        s3_bags = find_existing_rosbags(self.s3_dataset_uri, DATASET, SPLIT, VERSION, storage=self.storage)
        for topics in (set(TOPICS), NON_SENSOR_TOPICS):
            local_samples = self.replay([bag_path], topics)
            s3_samples = self.replay(s3_bags, topics)
            self.assertEqual(len(s3_samples), NUM_SAMPLES)
            self.assertEqual(s3_samples, local_samples)
            self.assertTrue(all(set(sample) - METADATA_FIELDS == topics for sample in s3_samples))
            self.assertEqual(s3_samples[0]["map_contents"], "<osm/>")

    def test_downloads_requested_topics_only(self):
        """Without requesting sensor topics, their messages are not downloaded."""
        bag_path = self.write_rosbag(1)
        self.upload()
        s3_bags = find_existing_rosbags(self.s3_dataset_uri, DATASET, SPLIT, VERSION, storage=self.storage)
        bag_size = sum(os.path.getsize(os.path.join(bag_path, name)) for name in os.listdir(bag_path))

        self.client.served_bytes.clear()
        self.replay(s3_bags, NON_SENSOR_TOPICS)
        self.assertLess(sum(self.client.served_bytes), 0.05 * bag_size)

        self.client.served_bytes.clear()
        self.replay(s3_bags, set(TOPICS))
        self.assertGreater(sum(self.client.served_bytes), 0.95 * bag_size)

    def test_reads_summary_larger_than_first_request(self):
        """A summary larger than the bytes first downloaded from the end of a file is read completely."""
        bag_path = self.write_rosbag(1)
        self.upload()
        mcap_name = f"{os.path.basename(bag_path)}_0.mcap"
        uri = join_s3_uri(find_existing_rosbags(self.s3_dataset_uri, DATASET, SPLIT, VERSION, storage=self.storage)[0], mcap_name)
        from autonomy_datasets.datasets.rosbag import s3

        original_size = s3.SUMMARY_READ_SIZE
        s3.SUMMARY_READ_SIZE = 64
        self.addCleanup(setattr, s3, "SUMMARY_READ_SIZE", original_size)
        messages = list(S3McapReader(self.storage, uri, {"/clock"}).iter_messages())
        self.assertEqual([timestamp for _, _, timestamp in messages], [i * 1_000_000_000 for i in range(NUM_SAMPLES)])

    def test_skips_incomplete_rosbags(self):
        """Rosbags in S3 without metadata or with truncated files are skipped."""
        self.write_rosbag(1)
        truncated_bag = self.write_rosbag(2)
        os.remove(os.path.join(self.write_rosbag(3), "metadata.yaml"))
        mcap_path = os.path.join(truncated_bag, f"{os.path.basename(truncated_bag)}_0.mcap")
        with open(mcap_path, "r+b") as mcap_file:
            mcap_file.truncate(os.path.getsize(mcap_path) // 2)
        self.upload()
        s3_bags = find_existing_rosbags(self.s3_dataset_uri, DATASET, SPLIT, VERSION, storage=self.storage)
        self.assertEqual(len(s3_bags), 3)
        adapter = RosbagReplayAdapter(s3_bags, dict.fromkeys(NON_SENSOR_TOPICS), storage=self.storage)
        self.assertEqual(adapter.rosbag_paths, s3_bags[:1])

    def test_streams_rosbags_with_large_chunks(self):
        """Rosbags written with large chunks are streamed as well."""
        bag_path = self.write_rosbag(1, storage_config="")
        self.upload()
        s3_bags = find_existing_rosbags(self.s3_dataset_uri, DATASET, SPLIT, VERSION, storage=self.storage)
        self.assertEqual(self.replay(s3_bags, NON_SENSOR_TOPICS), self.replay([bag_path], NON_SENSOR_TOPICS))


class TestRosbagMaps(RosbagS3TestBase):
    """Maps are stored as regular files in the directory of each rosbag, so that they can be uploaded to S3."""

    def test_stores_map_with_each_rosbag(self):
        """Scenes sharing a map each store their own copy of it, without links or a shared store."""
        bag_paths = [self.write_rosbag(index, with_map=True) for index in (1, 2)]
        for bag_path in bag_paths:
            map_path = os.path.join(bag_path, "map.osm")
            self.assertFalse(os.path.islink(map_path))
            self.assertEqual(os.stat(map_path).st_nlink, 1)
        self.assertEqual(sorted(os.listdir(self.bag_root_dir)), sorted(os.path.basename(path) for path in bag_paths))

    def test_replaces_hard_linked_map(self):
        """Rewriting a map hard-linked by an older version leaves the map of the other rosbag unchanged."""
        bag_paths = [self.write_rosbag(index, with_map=True) for index in (1, 2)]
        os.remove(os.path.join(bag_paths[1], "map.osm"))
        os.link(os.path.join(bag_paths[0], "map.osm"), os.path.join(bag_paths[1], "map.osm"))
        write_rosbag_map(bag_paths[1], "<osm changed/>", 50.78, 6.06)
        with open(os.path.join(bag_paths[0], "map.osm")) as map_file:
            self.assertEqual(map_file.read(), "<osm/>")
        with open(os.path.join(bag_paths[1], "map.osm")) as map_file:
            self.assertEqual(map_file.read(), "<osm changed/>")


class TestTopicFilter(RosbagS3TestBase):
    """Stored topics rejected by the topic filter are not streamed without a dataset adapter."""

    def test_filters_stored_topics(self):
        """Only stored topics accepted by the filter are requested, and only those are downloaded."""
        self.write_rosbag(1)
        self.upload()
        s3_bags = find_existing_rosbags(self.s3_dataset_uri, DATASET, SPLIT, VERSION, storage=self.storage)
        data_publishers: dict = {}
        adapter = RosbagReplayAdapter(
            s3_bags,
            data_publishers,
            request_stored_topics=True,
            topic_filter=lambda topic: topic not in SENSOR_TOPICS,
            storage=self.storage,
        )
        self.assertEqual(set(data_publishers), NON_SENSOR_TOPICS)
        samples = [sample for _, sample in adapter.generate_samples()]
        self.assertTrue(all(set(sample) - METADATA_FIELDS == NON_SENSOR_TOPICS for sample in samples))


if __name__ == "__main__":
    unittest.main()
