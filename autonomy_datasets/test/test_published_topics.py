# Copyright Thinking Cars GmbH
# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

"""Integration tests verifying that ``autonomy_datasets.launch.py`` publishes its topics.

For each dataset the node is launched and a helper node checks that the set of advertised
topics equals the set expected from the enabled ``publish_*`` parameters (the *requested*
topics), and that a message is actually received on every one of them.
"""

import math
import os
import tempfile

from dataset_test_base import DatasetNodeTestBase
from perception_msgs.msg import EgoData, HEXAMOTION, ObjectList
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
    "driving": {
        **_BASE_TOPICS,
        "/object_list/lidar_01": ObjectList,
        "/lidar_01/point_cloud": PointCloud2,
        **_camera_topics(6),
    },
}


class PublishedTopicsTestBase(DatasetNodeTestBase):
    """Verify that the launched node advertises and publishes the requested topics.

    Subclasses set :attr:`DATASET` and :attr:`EXPECTED_TOPICS`.
    """

    EXPECTED_TOPICS: dict = {}
    PARAM_OVERRIDES: dict = {}

    def test_requested_topics_are_published(self):
        """The advertised topics match the request and each one delivers a message."""
        self._launch(param_overrides=self.PARAM_OVERRIDES)
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


class TestDrivIng(PublishedTopicsTestBase):
    """Published-topics test for the DrivIng dataset."""

    __test__ = True
    DATASET = "driving"
    EXPECTED_TOPICS = EXPECTED_TOPICS_BY_DATASET["driving"]

    def test_requested_topics_are_published(self):
        """Publish one sequence and validate the first synchronized ROS sample."""
        with tempfile.NamedTemporaryFile(mode="w", suffix=".yml", delete=False) as config:
            config.write(
                """\
/**/*:
  ros__parameters:
    dataset: driving
    dataset_split: night
    driving_auto_download: false
    driving_rosbag_duration_seconds: 20.0
    publish_ego_data: true
    publish_camera_images: true
    publish_lidar_pointclouds: true
    publish_lidar_object_lists: true
"""
            )
        self.addCleanup(os.remove, config.name)
        self._launch(config=config.name)
        messages = self._assert_topics_published(self.EXPECTED_TOPICS, capture_first=self.EXPECTED_TOPICS)

        self.assertGreater(messages["/clock"].clock.sec, 0)
        self.assertEqual(
            [(transform.header.frame_id, transform.child_frame_id) for transform in messages["/tf"].transforms],
            [("map", "base_link")],
        )
        static_frames = {(transform.header.frame_id, transform.child_frame_id) for transform in messages["/tf_static"].transforms}
        self.assertEqual(
            static_frames,
            {("base_link", "lidar_01"), *(("base_link", f"camera_{index:02d}") for index in range(1, 7))},
        )

        ego = messages["/ego_data"]
        self.assertGreater(ego.length, 0.0)
        self.assertGreater(ego.width, 0.0)
        self.assertGreater(ego.height, 0.0)
        self.assertTrue(all(math.isfinite(value) for value in ego.state.continuous_state))

        cloud = messages["/lidar_01/point_cloud"]
        self.assertGreater(cloud.width, 0)
        self.assertEqual(cloud.point_step, 24)
        self.assertEqual([field.name for field in cloud.fields], ["x", "y", "z", "intensity", "timestamp"])
        self.assertEqual(len(cloud.data), cloud.row_step * cloud.height)

        for index in range(1, 7):
            image = messages[f"/camera_{index:02d}/image_raw"]
            info = messages[f"/camera_{index:02d}/camera_info"]
            self.assertEqual(image.encoding, "rgb8")
            self.assertEqual((image.width, image.height), (info.width, info.height))
            self.assertEqual(len(image.data), image.step * image.height)
            self.assertEqual(len(info.k), 9)

        objects = messages["/object_list/lidar_01"]
        for obj in objects.objects:
            state = obj.state.continuous_state
            self.assertGreater(state[HEXAMOTION.LENGTH], 0.0)
            self.assertGreater(state[HEXAMOTION.WIDTH], 0.0)
            self.assertGreater(state[HEXAMOTION.HEIGHT], 0.0)
