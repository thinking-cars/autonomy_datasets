# Copyright Thinking Cars GmbH
# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

"""Integration tests verifying that ``autonomy_datasets.launch.py`` publishes its topics.

For each dataset the node is launched and a helper node checks that the set of advertised
topics equals the set expected from the enabled ``publish_*`` parameters (the *requested*
topics), that a message is actually received on every one of them, and that the object states
of the first ego data and object list messages carry the covariances perception_msgs specifies.
"""

import numpy as np
import perception_msgs_utils as pmu
from autonomy_datasets_msgs.msg import ObjectListMetaInfo
from dataset_test_base import DatasetNodeTestBase
from perception_msgs.msg import EGO, EgoData, HEXAMOTION, ObjectList
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


def _point_cloud_topics(sensor, num_sensors):
    """Return {topic: type} for <sensor>_01..<sensor>_<num_sensors> point cloud topics."""
    return {f"/{sensor}_{i:02d}/point_cloud": PointCloud2 for i in range(1, num_sensors + 1)}


def _object_list_topics(*object_list_topics):
    """Return {topic: type} for the given object list topics and their meta information topics."""
    topics = {}
    for topic in object_list_topics:
        topics[topic] = ObjectList
        topics[f"{topic}/meta_info"] = ObjectListMetaInfo
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
        **_object_list_topics("/object_list/lidar_01"),
        "/lidar_01/point_cloud": PointCloud2,
        "/radar_01/point_cloud": PointCloud2,
        **_camera_topics(7),
    },
    "nuscenes": {
        **_BASE_TOPICS,
        **_object_list_topics("/object_list/lidar_01", "/object_list/camera_01", "/object_list/detected"),
        "/lidar_01/point_cloud": PointCloud2,
        **{f"/radar_{i:02d}/point_cloud": PointCloud2 for i in range(1, 6)},
        **_camera_topics(6),
        "/object_list/detected": ObjectList,
    },
    "waymo_open_dataset": {
        **_BASE_TOPICS,
        **_object_list_topics("/object_list/lidar_01", "/object_list/camera_01", "/object_list/camera_all"),
        "/lidar_01/point_cloud": PointCloud2,
        **_camera_topics(5),
    },
    "driving": {
        **_BASE_TOPICS,
        **_object_list_topics("/object_list/lidar_01"),
        "/lidar_01/point_cloud": PointCloud2,
        **_camera_topics(6),
    },
    "truckscenes": {
        **_BASE_TOPICS,
        **_object_list_topics("/object_list/lidar_01", "/object_list/camera_01"),
        **_point_cloud_topics("lidar", 6),
        **_point_cloud_topics("radar", 6),
        **_camera_topics(4),
    },
    "tum_traffic": {
        "/clock": Clock,
        "/tf": TFMessage,
        "/tf_static": TFMessage,
        "/object_list/lidar_01": ObjectList,
        **_point_cloud_topics("lidar", 2),
        **_camera_topics(2),
    },
    "zenseact_open_dataset": {
        **_BASE_TOPICS,
        **_object_list_topics("/object_list/lidar_01", "/object_list/camera_01"),
        "/lidar_01/point_cloud": PointCloud2,
        **_camera_topics(1),
    },
    # The sensor suite differs between FZI-AURA scenes, so the topics depend on the scene the
    # test publishes: the six Ouster lidars every scene holds, but none of the Aeva lidars.
    "fzi_aura": {
        **_BASE_TOPICS,
        **_object_list_topics("/object_list/lidar_01", "/object_list/base_link"),
        **_point_cloud_topics("lidar", 6),
        **_point_cloud_topics("radar", 3),
        **_camera_topics(8),
    },
}

# Object lists holding detections, which are estimates without a covariance, rather than ground truth
_DETECTION_TOPICS = {"/object_list/detected"}


class PublishedTopicsTestBase(DatasetNodeTestBase):
    """Verify that the launched node advertises and publishes the requested topics.

    Subclasses set :attr:`DATASET` and :attr:`EXPECTED_TOPICS`.
    """

    EXPECTED_TOPICS: dict = {}
    PARAM_OVERRIDES: dict = {}

    def test_requested_topics_are_published(self):
        """The advertised topics match the request, each one delivers a message, and states carry valid covariances."""
        self._launch(param_overrides=self.PARAM_OVERRIDES)
        state_topics = [topic for topic, msg_type in self.EXPECTED_TOPICS.items() if msg_type in (EgoData, ObjectList)]
        first_messages = self._assert_topics_published(self.EXPECTED_TOPICS, capture_first=state_topics)
        for topic, msg in first_messages.items():
            set_variance = pmu.CONTINUOUS_STATE_COVARIANCE_UNKNOWN if topic in _DETECTION_TOPICS else 0.0
            for state in [msg.state] if isinstance(msg, EgoData) else [obj.state for obj in msg.objects]:
                self._assert_state_covariance(topic, state, set_variance)

    def _assert_state_covariance(self, topic, state, set_variance):
        """Assert that the set states of an object state have the given variance and all others are invalid.

        Every dataset provides the position and the yaw angle, and no dataset correlates states.
        """
        if state.model_id not in (EGO.MODEL_ID, HEXAMOTION.MODEL_ID):
            # Waymo publishes its 2D camera objects without a state if perception_msgs lacks the CAMERA2D model
            return
        size = len(state.continuous_state)
        covariance = np.reshape(state.continuous_state_covariance, (size, size))
        variances = np.diag(covariance)
        self.assertFalse(np.any(covariance - np.diag(variances)), msg=f"Correlated states on '{topic}'")
        self.assertLessEqual(
            set(variances),
            {set_variance, pmu.CONTINUOUS_STATE_COVARIANCE_INVALID},
            msg=f"Unexpected state variances on '{topic}'",
        )
        for index in (pmu.index_x, pmu.index_y, pmu.index_z, pmu.index_yaw):
            self.assertEqual(
                variances[index(state.model_id)],
                set_variance,
                msg=f"State {index(state.model_id)} on '{topic}' is not set as expected",
            )


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
    PARAM_OVERRIDES = {"publish_megvii_detections": True}


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
    PARAM_OVERRIDES = {"dataset_split": "dusk", "driving_auto_download": False}


class TestTruckScenes(PublishedTopicsTestBase):
    """Published-topics test for the MAN TruckScenes dataset."""

    __test__ = True
    DATASET = "truckscenes"
    EXPECTED_TOPICS = EXPECTED_TOPICS_BY_DATASET["truckscenes"]
    PARAM_OVERRIDES = {"dataset_split": "mini_val", "truckscenes_auto_download": False}


class TestTumTraffic(PublishedTopicsTestBase):
    """Published-topics test for the TUM Traffic Dataset."""

    __test__ = True
    DATASET = "tum_traffic"
    EXPECTED_TOPICS = EXPECTED_TOPICS_BY_DATASET["tum_traffic"]
    PARAM_OVERRIDES = {"dataset_split": "r02"}


class TestZenseactOpenDataset(PublishedTopicsTestBase):
    """Published-topics test for the Zenseact Open Dataset."""

    __test__ = True
    DATASET = "zenseact_open_dataset"
    EXPECTED_TOPICS = EXPECTED_TOPICS_BY_DATASET["zenseact_open_dataset"]
    # The frames subset publishes one sample per frame, so the test does not have to decode a
    # whole sequence; the 8 MP images are scaled down to keep the transported samples small.
    PARAM_OVERRIDES = {
        "dataset_split": "frames_mini_val",
        "zod_auto_download": False,
        "zod_image_scale": 0.25,
    }


class TestFziAura(PublishedTopicsTestBase):
    """Published-topics test for the FZI-AURA dataset."""

    __test__ = True
    DATASET = "fzi_aura"
    # One lidar is disabled to verify that the lidars are enabled individually and keep their topics.
    EXPECTED_TOPICS = {
        topic: msg_type for topic, msg_type in EXPECTED_TOPICS_BY_DATASET["fzi_aura"].items() if topic != "/lidar_03/point_cloud"
    }
    # The full sensor suite of a scene reaches 75 MB per sample, so the images are scaled down to
    # keep the transported samples small.
    PARAM_OVERRIDES = {"fzi_aura_auto_download": False, "fzi_aura_image_scale": 0.125, "publish_lidar_03_pointclouds": False}
