# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

"""Streaming of rosbags stored in an S3 bucket, e.g. on a MinIO server.

rosbag2 can only read rosbags from the local file system, so rosbags stored in S3 are read with
the ``mcap`` library instead, using HTTP range requests. The summary at the end of an MCAP file
indexes its chunks and which topics each chunk holds messages of. Only the chunks that hold
messages of the requested topics are therefore downloaded, so that e.g. disabled camera images
are never transferred.

Since a chunk can only be downloaded and decompressed as a whole, rosbags are written with one
message per chunk (see ``chunkSize`` in ``config/mcap_storage_config.yaml``); rosbags written with
larger chunks are streamed as well, but transfer the sensor data that shares chunks with the
requested topics.

The rosbags are stored in S3 with the same layout as below ``datasets_path``, i.e. in
``<url>/<dataset>/bags/<version>/<rosbag>/``. The credentials of the S3 server are read from the
standard environment variables ``AWS_ACCESS_KEY_ID`` and ``AWS_SECRET_ACCESS_KEY``.
"""

import heapq
import io
import itertools
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass, field
from typing import Any, Collection, Dict, Iterator, List, Optional, Tuple

import yaml

S3_URI_SCHEME = "s3://"

# Name of the rosbag2 metadata file of a rosbag directory
ROSBAG_METADATA_FILENAME = "metadata.yaml"

# Number of bytes downloaded from the end of an MCAP file to read its footer and summary, which
# are typically far smaller; a larger summary is completed with a second request
SUMMARY_READ_SIZE = 256 * 1024

# Selected chunks closer to each other than this are downloaded with a single request, including
# the bytes in between, which is cheaper than a further request for such small gaps. The gap is
# smaller than any camera image or point cloud, so that skipped sensor data is never downloaded.
MAX_CHUNK_GAP = 64 * 1024

# Upper bound of the bytes downloaded with a single request, unless a single chunk is larger
MAX_REQUEST_SIZE = 16 * 1024 * 1024

# Requests are downloaded ahead of the messages currently read, which hides the latency of the S3
# server from the replay: up to this many requests in parallel ...
PREFETCHED_REQUESTS = 16
# ... as long as they download fewer bytes than this in total
PREFETCHED_SIZE = 32 * 1024 * 1024

# Number of rosbags whose metadata is downloaded in parallel
METADATA_DOWNLOAD_WORKERS = 16

# Length of the record opcode and the record length preceding the content of each MCAP record
_RECORD_PREFIX_LENGTH = 1 + 8


def is_s3_uri(path: str) -> bool:
    """Returns whether a path is an S3 URI of the form ``s3://<bucket>/<key>``."""
    return path.startswith(S3_URI_SCHEME)


def split_s3_uri(uri: str) -> Tuple[str, str]:
    """Splits an S3 URI into its bucket and its key without trailing slashes.

    Raises:
        ValueError: If the URI is no S3 URI or lacks a bucket.
    """
    if not is_s3_uri(uri):
        raise ValueError(f"'{uri}' is no S3 URI of the form '{S3_URI_SCHEME}<bucket>/<key>'")
    bucket, _, key = uri[len(S3_URI_SCHEME) :].partition("/")
    if not bucket:
        raise ValueError(f"S3 URI '{uri}' lacks a bucket")
    return bucket, key.strip("/")


def join_s3_uri(uri: str, *parts: str) -> str:
    """Appends path components to an S3 URI."""
    return "/".join([uri.rstrip("/"), *(part.strip("/") for part in parts if part)])


class S3Storage:
    """Reads objects and object listings of an S3 server."""

    def __init__(self, endpoint_url: Optional[str] = None, client: Optional[Any] = None):
        """Connects to an S3 server.

        Args:
            endpoint_url: URL of the S3 server, e.g. ``https://minio.example.com:9000``. Without a
                URL, the endpoint is taken from the environment variable ``AWS_ENDPOINT_URL``, or
                AWS S3 is used.
            client: S3 client to use instead of a ``boto3`` client, e.g. in tests.
        """
        if client is None:
            # imported here, so that boto3 is only required if rosbags are streamed from S3
            import boto3
            from botocore.config import Config

            client = boto3.client(
                "s3",
                endpoint_url=endpoint_url or None,
                config=Config(
                    retries={"max_attempts": 5, "mode": "standard"},
                    max_pool_connections=max(METADATA_DOWNLOAD_WORKERS, PREFETCHED_REQUESTS) + 1,
                ),
            )
        self.client = client

    def list_subdirectories(self, uri: str) -> List[str]:
        """Returns the sorted URIs of the "directories" directly below an S3 URI.

        S3 has no directories, but keys whose path components below the URI contain a slash are
        listed as common prefixes, which correspond to the directories of a file system.
        """
        bucket, key = split_s3_uri(uri)
        prefix = f"{key}/" if key else ""
        subdirectories = []
        continuation_token = None
        while True:
            kwargs = {"Bucket": bucket, "Prefix": prefix, "Delimiter": "/"}
            if continuation_token is not None:
                kwargs["ContinuationToken"] = continuation_token
            response = self.client.list_objects_v2(**kwargs)
            for common_prefix in response.get("CommonPrefixes", []):
                subdirectories.append(f"{S3_URI_SCHEME}{bucket}/{common_prefix['Prefix'].rstrip('/')}")
            if not response.get("IsTruncated"):
                break
            continuation_token = response["NextContinuationToken"]
        return sorted(subdirectories)

    def read(self, uri: str, start: Optional[int] = None, end: Optional[int] = None) -> bytes:
        """Downloads an object or the byte range ``[start, end)`` of it.

        Raises:
            FileNotFoundError: If the object does not exist.
        """
        bucket, key = split_s3_uri(uri)
        kwargs = {"Bucket": bucket, "Key": key}
        if start is not None or end is not None:
            kwargs["Range"] = f"bytes={start or 0}-{'' if end is None else end - 1}"
        return self._get_object(uri, **kwargs)["Body"].read()

    def read_tail(self, uri: str, length: int) -> Tuple[bytes, int]:
        """Downloads the last ``length`` bytes of an object.

        Returns:
            The bytes read, which are fewer than ``length`` if the object is smaller, and the size
            of the object.

        Raises:
            FileNotFoundError: If the object does not exist.
        """
        bucket, key = split_s3_uri(uri)
        response = self._get_object(uri, Bucket=bucket, Key=key, Range=f"bytes=-{length}")
        data = response["Body"].read()
        # the total size follows the slash of the content range, e.g. "bytes 100-199/200"
        content_range = response.get("ContentRange")
        size = int(content_range.rsplit("/", 1)[1]) if content_range else len(data)
        return data, size

    def _get_object(self, uri: str, **kwargs) -> Dict[str, Any]:
        """Calls ``get_object`` and reports missing objects as ``FileNotFoundError``."""
        try:
            return self.client.get_object(**kwargs)
        except Exception as error:
            code = getattr(error, "response", {}).get("Error", {}).get("Code")
            if code in ("NoSuchKey", "404", "NotFound"):
                raise FileNotFoundError(f"'{uri}' does not exist") from error
            raise


@dataclass
class RosbagInfo:
    """Metadata of a rosbag required for replaying it."""

    #: Total number of messages of the rosbag
    message_count: int
    #: Message type of each topic of the rosbag
    topic_types: Dict[str, str]
    #: Number of messages of each topic of the rosbag
    topic_message_counts: Dict[str, int]
    #: Custom data stored with the rosbag, e.g. the ID of its scene
    custom_data: Dict[str, str] = field(default_factory=dict)
    #: Paths of the storage files of the rosbag relative to its directory
    relative_file_paths: List[str] = field(default_factory=list)


def read_s3_rosbag_info(storage: S3Storage, bag_uri: str, mcap_magic: bytes) -> RosbagInfo:
    """Returns the metadata of a rosbag stored in S3 after verifying that the rosbag is complete.

    Args:
        storage: S3 storage holding the rosbag.
        bag_uri: S3 URI of the rosbag directory.
        mcap_magic: Magic bytes a complete MCAP file ends with.

    Raises:
        ValueError: If the rosbag is incomplete or cannot be read.
    """
    try:
        metadata = yaml.safe_load(storage.read(join_s3_uri(bag_uri, ROSBAG_METADATA_FILENAME)))
        info = metadata["rosbag2_bagfile_information"]
        if info.get("storage_identifier", "mcap") != "mcap":
            raise ValueError(f"storage '{info['storage_identifier']}' is not supported, only 'mcap'")
        topics = info.get("topics_with_message_count") or []
        rosbag_info = RosbagInfo(
            message_count=int(info.get("message_count", 0)),
            topic_types={topic["topic_metadata"]["name"]: topic["topic_metadata"]["type"] for topic in topics},
            topic_message_counts={topic["topic_metadata"]["name"]: int(topic["message_count"]) for topic in topics},
            custom_data={str(key): str(value) for key, value in (info.get("custom_data") or {}).items()},
            relative_file_paths=list(info.get("relative_file_paths") or []),
        )
    except FileNotFoundError as error:
        raise ValueError(f"has no metadata ({error})") from error
    except (KeyError, TypeError, AttributeError, yaml.YAMLError) as error:
        raise ValueError(f"has invalid metadata ({error!r})") from error

    for relative_file_path in rosbag_info.relative_file_paths:
        try:
            tail, _ = storage.read_tail(join_s3_uri(bag_uri, relative_file_path), len(mcap_magic))
        except FileNotFoundError as error:
            raise ValueError(f"file '{relative_file_path}' does not exist") from error
        if tail != mcap_magic:
            raise ValueError(f"file '{relative_file_path}' is truncated")
    if rosbag_info.message_count == 0:
        raise ValueError("holds no message")
    return rosbag_info


@dataclass(eq=False)
class _Request:
    """Byte range of an MCAP file downloaded with a single request.

    Requests are compared by identity, so that each one can be used as a dictionary key.
    """

    start: int
    end: int
    #: Number of chunks within the range that have not been read yet
    pending_chunks: int = 0


class S3McapReader:
    """Reads the messages of selected topics from an MCAP file stored in S3.

    Only the chunks holding messages of the selected topics are downloaded. Chunks close to each
    other are downloaded with a single request, and requests are downloaded ahead of the
    messages being read.
    """

    def __init__(self, storage: S3Storage, uri: str, topics: Optional[Collection[str]] = None):
        """Prepares reading an MCAP file.

        Args:
            storage: S3 storage holding the MCAP file.
            uri: S3 URI of the MCAP file.
            topics: Topics whose messages are read; all topics are read if not given.
        """
        self.storage = storage
        self.uri = uri
        self.topics = None if topics is None else set(topics)
        #: Size of the MCAP file
        self.size = 0
        #: Number of bytes downloaded so far
        self.bytes_read = 0

    def iter_messages(self) -> Iterator[Tuple[str, bytes, int]]:
        """Yields the messages of the selected topics in the order of their log time.

        Yields:
            Tuples of (topic, serialized message, log time in nanoseconds).

        Raises:
            ValueError: If the MCAP file is invalid or has no summary indexing its chunks.
        """
        # imported here, so that the mcap library is only required if rosbags are streamed from S3
        from mcap.records import Channel, ChunkIndex

        channel_topics: Dict[int, str] = {}
        chunk_indexes: List[ChunkIndex] = []
        for record in self._read_summary():
            if isinstance(record, Channel):
                channel_topics[record.id] = record.topic
            elif isinstance(record, ChunkIndex):
                chunk_indexes.append(record)
        selected_channels = {
            channel_id for channel_id, topic in channel_topics.items() if self.topics is None or topic in self.topics
        }

        # A chunk without message indexes may hold messages of any channel
        chunks = [
            chunk_index
            for chunk_index in chunk_indexes
            if not chunk_index.message_index_offsets or selected_channels & chunk_index.message_index_offsets.keys()
        ]
        # chunks are read in the order of their messages, which is their order in a rosbag
        chunks.sort(key=lambda chunk_index: (chunk_index.message_start_time, chunk_index.chunk_start_offset))
        chunk_requests = self._plan_requests(chunks)

        # requests in the order they are first needed by the chunks, which are downloaded ahead
        request_order = list(dict.fromkeys(chunk_requests))
        request_positions = {request: position for position, request in enumerate(request_order)}
        with ThreadPoolExecutor(max_workers=PREFETCHED_REQUESTS) as executor:
            downloads: Dict[_Request, Future] = {}

            def download(request: _Request) -> bytes:
                """Returns the bytes of a request after starting the download of the next ones."""
                position = request_positions[request]
                prefetched_size = 0
                for upcoming in request_order[position : position + PREFETCHED_REQUESTS]:
                    prefetched_size += upcoming.end - upcoming.start
                    if upcoming is not request and prefetched_size > PREFETCHED_SIZE:
                        break
                    if upcoming not in downloads and upcoming.pending_chunks > 0:
                        downloads[upcoming] = executor.submit(self.storage.read, self.uri, upcoming.start, upcoming.end)
                return downloads[request].result()

            # messages of the chunks read so far, ordered by log time and the order they were read in
            messages: List[Tuple[int, int, str, bytes]] = []
            sequence = itertools.count()
            next_chunk = 0
            while next_chunk < len(chunks) or messages:
                # a chunk whose messages may precede the next message is read before yielding it
                if next_chunk < len(chunks) and (not messages or chunks[next_chunk].message_start_time <= messages[0][0]):
                    chunk_index, request = chunks[next_chunk], chunk_requests[next_chunk]
                    next_chunk += 1
                    data = download(request)
                    offset = chunk_index.chunk_start_offset - request.start
                    for channel_id, log_time, payload in self._read_chunk(data[offset : offset + chunk_index.chunk_length]):
                        if channel_id in selected_channels:
                            heapq.heappush(messages, (log_time, next(sequence), channel_topics[channel_id], payload))
                    request.pending_chunks -= 1
                    if request.pending_chunks == 0:
                        # the downloaded bytes are released once all chunks within them are read
                        self.bytes_read += request.end - request.start
                        del downloads[request]
                    continue
                log_time, _, topic, payload = heapq.heappop(messages)
                yield topic, payload, log_time

    def _read_summary(self) -> List[Any]:
        """Downloads and parses the summary section of the MCAP file.

        Raises:
            ValueError: If the MCAP file is invalid or has no summary.
        """
        from mcap.data_stream import ReadDataStream
        from mcap.opcode import Opcode
        from mcap.records import Footer
        from mcap.stream_reader import MAGIC_SIZE, StreamReader

        tail, self.size = self.storage.read_tail(self.uri, SUMMARY_READ_SIZE)
        self.bytes_read += len(tail)
        tail_start = self.size - len(tail)
        footer_length = _RECORD_PREFIX_LENGTH + 8 + 8 + 4
        footer_start = self.size - MAGIC_SIZE - footer_length
        if footer_start < tail_start or tail[footer_start - tail_start] != Opcode.FOOTER:
            raise ValueError(f"MCAP file '{self.uri}' has no footer")
        footer = Footer.read(ReadDataStream(io.BytesIO(tail[footer_start - tail_start + _RECORD_PREFIX_LENGTH :])))
        if footer.summary_start == 0:
            raise ValueError(f"MCAP file '{self.uri}' has no summary indexing its chunks")
        if footer.summary_start < tail_start:
            # the summary is larger than the downloaded tail
            tail = self.storage.read(self.uri, footer.summary_start, tail_start) + tail
            self.bytes_read += tail_start - footer.summary_start
            tail_start = footer.summary_start
        # parsed up to the end of the file, as the reader expects the footer and the magic bytes
        summary = tail[footer.summary_start - tail_start :]
        return list(StreamReader(io.BytesIO(summary), skip_magic=True).records)

    @staticmethod
    def _plan_requests(chunks: List[Any]) -> List[_Request]:
        """Groups chunks into the byte ranges downloaded with a single request.

        Returns:
            The request downloading each chunk, in the order of the given chunks.
        """
        requests: Dict[int, _Request] = {}  # by ID of the chunk index
        current: Optional[_Request] = None
        for chunk_index in sorted(chunks, key=lambda chunk_index: chunk_index.chunk_start_offset):
            start = chunk_index.chunk_start_offset
            end = start + chunk_index.chunk_length
            if current is None or start - current.end > MAX_CHUNK_GAP or end - current.start > MAX_REQUEST_SIZE:
                current = _Request(start=start, end=end)
            else:
                current.end = max(current.end, end)
            current.pending_chunks += 1
            requests[id(chunk_index)] = current
        return [requests[id(chunk_index)] for chunk_index in chunks]

    @staticmethod
    def _read_chunk(record: bytes) -> Iterator[Tuple[int, int, bytes]]:
        """Yields (channel ID, log time, payload) of each message of a chunk record."""
        from mcap.data_stream import ReadDataStream
        from mcap.opcode import Opcode
        from mcap.records import Chunk, Message
        from mcap.stream_reader import breakup_chunk

        if not record or record[0] != Opcode.CHUNK:
            raise ValueError("chunk index does not point to a chunk")
        chunk = Chunk.read(ReadDataStream(io.BytesIO(record[_RECORD_PREFIX_LENGTH:])))
        for chunk_record in breakup_chunk(chunk, validate_crc=True):
            if isinstance(chunk_record, Message):
                yield chunk_record.channel_id, chunk_record.log_time, chunk_record.data


def download_rosbag_infos(
    storage: S3Storage, bag_uris: List[str], mcap_magic: bytes
) -> List[Tuple[str, Optional[RosbagInfo], Optional[ValueError]]]:
    """Downloads the metadata of several rosbags stored in S3 in parallel.

    Returns:
        Tuples of (rosbag URI, metadata, error) in the order of the given URIs, holding either
        the metadata or the error that the rosbag is incomplete or cannot be read.
    """

    def read_info(bag_uri: str) -> Tuple[str, Optional[RosbagInfo], Optional[ValueError]]:
        try:
            return bag_uri, read_s3_rosbag_info(storage, bag_uri, mcap_magic), None
        except ValueError as error:
            return bag_uri, None, error

    with ThreadPoolExecutor(max_workers=METADATA_DOWNLOAD_WORKERS) as executor:
        return list(executor.map(read_info, bag_uris))
