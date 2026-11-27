# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for the timeline of the samples of the NVIDIA PhysicalAI-AV adapter.

The tests run offline: stand-ins for the camera videos of a clip hold only their frame timestamps.
"""

import unittest
from types import SimpleNamespace

import numpy as np
from autonomy_datasets.datasets.nvidia_physicalai_av_dataset.nvidia_physicalai_av_dataset import (
    _compute_sample_timestamps,
    _timeline_camera_timestamps,
)

# ego motion is sampled more densely than the cameras record frames
EGO_TIMESTAMPS = np.arange(0, 1_000_000, 10_000)
FRONT_TELE_TIMESTAMPS = np.arange(0, 1_000_000, 33_333)
FRONT_WIDE_TIMESTAMPS = np.arange(5_000, 1_000_000, 33_333)


def _video(timestamps):
    """Return a stand-in for the video of a camera, holding its frame timestamps in microseconds."""
    return SimpleNamespace(timestamps=timestamps)


class TestTimelineCamera(unittest.TestCase):
    """Camera-only samples follow the frames of a published camera."""

    def test_prefers_front_camera(self):
        """The front camera forms the timeline if it is published together with other cameras."""
        videos = {
            "camera_front_tele_30fov": _video(FRONT_TELE_TIMESTAMPS),
            "camera_front_wide_120fov": _video(FRONT_WIDE_TIMESTAMPS),
        }
        np.testing.assert_array_equal(_timeline_camera_timestamps(videos), FRONT_TELE_TIMESTAMPS)

    def test_uses_published_camera_without_front_camera(self):
        """Without the front camera, e.g. with include_topics: ["camera_02"], the published camera forms the timeline."""
        videos = {"camera_front_wide_120fov": _video(FRONT_WIDE_TIMESTAMPS)}
        timeline = _timeline_camera_timestamps(videos)
        np.testing.assert_array_equal(timeline, FRONT_WIDE_TIMESTAMPS)
        samples = _compute_sample_timestamps(timeline, None, None, [], EGO_TIMESTAMPS)
        np.testing.assert_array_equal(samples, FRONT_WIDE_TIMESTAMPS)

    def test_uses_first_published_camera_in_topic_order(self):
        """Of several published cameras without the front camera, the first one in topic order forms the timeline."""
        videos = {
            "camera_front_wide_120fov": _video(FRONT_WIDE_TIMESTAMPS),
            "camera_rear_tele_30fov": _video(FRONT_TELE_TIMESTAMPS),
        }
        np.testing.assert_array_equal(_timeline_camera_timestamps(videos), FRONT_WIDE_TIMESTAMPS)

    def test_no_camera_published(self):
        """Without published cameras, there is no camera timeline, so that the ego motion forms it."""
        self.assertIsNone(_timeline_camera_timestamps({}))
        samples = _compute_sample_timestamps(None, None, None, [], EGO_TIMESTAMPS)
        np.testing.assert_array_equal(samples, EGO_TIMESTAMPS)


if __name__ == "__main__":
    unittest.main()
