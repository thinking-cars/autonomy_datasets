# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

"""Integration tests verifying that ``autonomy_datasets.launch.py`` publishes its topics.

For each dataset the node is launched and a helper node checks that the set of advertised
topics equals the set expected from the enabled ``publish_*`` parameters (the *requested*
topics), and that a message is actually received on every one of them.
"""

from dataset_test_base import DatasetNodeTestBase
from perception_msgs.msg import EgoData, ObjectList
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_msgs.msg import TFMessage


def _camera_topics(num_cameras):
    """Return {topic: type} for camera_01..camera_<num_cameras> image and info topics."""
    topics = {}
    for i in range(1, num_cameras + 1):
        topics[f"/camera_{i:02d}/image_raw"] = Image
        topics[f"/camera_{i:02d}/camera_info"] = CameraInfo
    return topics


# Topics each dataset is expected to publish when every publish_* parameter is enabled
_BASE_TOPICS = {
    "/clock": Clock,
    "/tf": TFMessage,
    "/tf_static": TFMessage,
    "/ego_data": EgoData,
}
EXPECTED_TOPICS_BY_DATASET = {
    "nvidia_physicalai_av_dataset": {
        **_BASE_TOPICS,
        "/object_list/lidar_01": ObjectList,
        "/lidar_01/point_cloud": PointCloud2,
        "/radar_01/point_cloud": PointCloud2,
        **_camera_topics(7),
    },
    "nuscenes": {
        **_BASE_TOPICS,
        "/object_list/lidar_01": ObjectList,
        "/object_list/camera_01": ObjectList,
        "/lidar_01/point_cloud": PointCloud2,
        **_camera_topics(6),
    },
    "waymo_open_dataset": {
        **_BASE_TOPICS,
        "/object_list/lidar_01": ObjectList,
        "/object_list/camera_01": ObjectList,
        "/object_list/camera_all": ObjectList,
        "/lidar_01/point_cloud": PointCloud2,
        **_camera_topics(5),
    },
}


class PublishedTopicsTestBase(DatasetNodeTestBase):
    """Verify that the launched node advertises and publishes the requested topics.

    Subclasses set :attr:`DATASET` and :attr:`EXPECTED_TOPICS`.
    """

    EXPECTED_TOPICS: dict = {}

    def test_requested_topics_are_published(self):
        """The advertised topics match the request and each one delivers a message."""
        self._launch()
        self._assert_topics_published(self.EXPECTED_TOPICS)


class TestNvidiaPhysicalAiAvDataset(PublishedTopicsTestBase):
    """Published-topics test for the nvidia_physicalai_av_dataset dataset."""

    __test__ = True
    DATASET = "nvidia_physicalai_av_dataset"
    EXPECTED_TOPICS = EXPECTED_TOPICS_BY_DATASET["nvidia_physicalai_av_dataset"]


class TestNuscenes(PublishedTopicsTestBase):
    """Published-topics test for the nuscenes dataset."""

    __test__ = True
    DATASET = "nuscenes"
    EXPECTED_TOPICS = EXPECTED_TOPICS_BY_DATASET["nuscenes"]


class TestWaymoOpenDataset(PublishedTopicsTestBase):
    """Published-topics test for the waymo_open_dataset dataset."""

    __test__ = True
    DATASET = "waymo_open_dataset"
    EXPECTED_TOPICS = EXPECTED_TOPICS_BY_DATASET["waymo_open_dataset"]
