# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for the playback control of the sample publishing loop.

Each test runs a stub of the node's publish loop in a background thread, which asks the
:class:`PlaybackController` for every sample of a synthetic dataset whether it may be published,
while the test issues sample requests as the ``request_samples`` service does.
"""

import threading
import time
import unittest
from typing import List, Optional

from autonomy_datasets.playback_control import (
    MODE_ALL_SAMPLES,
    MODE_NEXT_SAMPLES,
    MODE_SAMPLE_IDS,
    PlaybackController,
    SampleRequestResult,
)

# Number of samples the stubbed publish loop generates in total and per scene
NUM_SAMPLES = 10
SAMPLES_PER_SCENE = 5

# Number of samples and time in seconds between two of them, used by tests that need a playback
# that is still running while they issue their requests
NUM_SLOW_SAMPLES = 50
SLOW_SAMPLE_INTERVAL_S = 0.01

# Time in seconds to wait for the publish loop to reach an expected state, and interval at which
# the state is checked
TIMEOUT_S = 10.0
POLL_INTERVAL_S = 0.01


class PublishLoopStub:
    """Stub of the node's publish loop, publishing a fixed number of samples of a fake dataset."""

    def __init__(self, controller: PlaybackController, num_samples: int = NUM_SAMPLES, sample_interval_s: float = 0.0):
        """Constructor

        Args:
            controller (PlaybackController): playback control to ask for every sample
            num_samples (int, optional): number of samples to generate
            sample_interval_s (float, optional): time to spend on publishing a single sample
        """
        self.controller = controller
        self.num_samples = num_samples
        self.sample_interval_s = sample_interval_s
        self.published_sample_ids: List[int] = []
        self.skipped_sample_ids: List[int] = []
        self.finished = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        """Start publishing samples in a background thread."""
        self._thread.start()

    def join(self) -> None:
        """Wait until all samples of the fake dataset have been handled."""
        self._thread.join(timeout=TIMEOUT_S)
        assert not self._thread.is_alive(), "publish loop did not reach the end of the dataset"

    def wait_for_published(self, num_samples: int) -> None:
        """Wait until the given number of samples has been published.

        Args:
            num_samples (int): number of published samples to wait for
        """
        deadline = time.monotonic() + TIMEOUT_S
        while time.monotonic() < deadline:
            if len(self.published_sample_ids) >= num_samples:
                return
            time.sleep(POLL_INTERVAL_S)
        raise AssertionError(f"published {len(self.published_sample_ids)} instead of {num_samples} sample(s)")

    def _run(self) -> None:
        """Publish all samples of the fake dataset, as the node's publish loop does."""
        for sample_id in range(self.num_samples):
            if self.controller.await_sample(sample_id):
                self.published_sample_ids.append(sample_id)
                self.controller.sample_published(sample_id, f"scene_{sample_id // SAMPLES_PER_SCENE}")
                time.sleep(self.sample_interval_s)
            else:
                self.skipped_sample_ids.append(sample_id)
        self.controller.pass_finished()
        self.finished.set()


class SampleRequestStub:
    """Stub of the service callback, issuing a sample request from another thread."""

    def __init__(self, controller: PlaybackController, **request_arguments):
        """Constructor

        Args:
            controller (PlaybackController): playback control to request samples from
            **request_arguments: arguments of PlaybackController.request_samples
        """
        self.result: Optional[SampleRequestResult] = None
        self._thread = threading.Thread(target=lambda: self._request(controller, **request_arguments), daemon=True)
        self._thread.start()

    def await_result(self) -> SampleRequestResult:
        """Wait for the request to be answered.

        Returns:
            SampleRequestResult: outcome of the request
        """
        self._thread.join(timeout=TIMEOUT_S)
        assert self.result is not None, "sample request was not answered"
        return self.result

    def _request(self, controller: PlaybackController, **request_arguments) -> None:
        """Request samples and store the result.

        Args:
            controller (PlaybackController): playback control to request samples from
            **request_arguments: arguments of PlaybackController.request_samples
        """
        self.result = controller.request_samples(**request_arguments)


class TestPlaybackControl(unittest.TestCase):
    """Tests the states of the playback control and the sample requests driving it."""

    def test_free_running_playback_publishes_all_samples(self):
        """Without a request, playback publishes every sample of the dataset."""
        controller = PlaybackController()
        publish_loop = PublishLoopStub(controller)

        publish_loop.start()
        publish_loop.join()

        self.assertEqual(publish_loop.published_sample_ids, list(range(NUM_SAMPLES)))

    def test_playback_started_paused_publishes_no_sample_without_a_request(self):
        """A playback started paused waits for a request instead of publishing samples."""
        controller = PlaybackController(start_paused=True)
        publish_loop = PublishLoopStub(controller)

        publish_loop.start()

        self.assertFalse(publish_loop.finished.wait(10 * POLL_INTERVAL_S))
        self.assertEqual(publish_loop.published_sample_ids, [])

    def test_request_publishes_next_samples_and_pauses_afterwards(self):
        """A request for the next samples publishes them and then waits for the next request."""
        controller = PlaybackController(start_paused=True)
        publish_loop = PublishLoopStub(controller)
        publish_loop.start()

        result = controller.request_samples(mode=MODE_NEXT_SAMPLES, num_samples=3)
        self.assertTrue(result.success, result.message)
        self.assertEqual(result.published_sample_ids, [0, 1, 2])
        self.assertEqual(result.published_scene_ids, ["scene_0"] * 3)
        self.assertFalse(result.end_of_dataset)
        self.assertEqual(publish_loop.published_sample_ids, [0, 1, 2])

        result = controller.request_samples(mode=MODE_NEXT_SAMPLES, num_samples=1)
        self.assertTrue(result.success, result.message)
        self.assertEqual(result.published_sample_ids, [3])
        self.assertFalse(publish_loop.finished.is_set())

    def test_first_request_takes_control_of_a_free_running_playback(self):
        """Playback stops publishing samples on its own once a request has been received."""
        controller = PlaybackController()
        publish_loop = PublishLoopStub(controller, num_samples=NUM_SLOW_SAMPLES, sample_interval_s=SLOW_SAMPLE_INTERVAL_S)
        publish_loop.start()

        result = controller.request_samples(mode=MODE_NEXT_SAMPLES, num_samples=1)
        num_published = len(publish_loop.published_sample_ids)

        self.assertTrue(result.success, result.message)
        self.assertEqual(len(result.published_sample_ids), 1)
        self.assertFalse(publish_loop.finished.wait(10 * SLOW_SAMPLE_INTERVAL_S))
        self.assertEqual(len(publish_loop.published_sample_ids), num_published)

    def test_request_without_samples_takes_control_without_publishing(self):
        """A request for no samples takes control of the playback without publishing a sample."""
        controller = PlaybackController()
        publish_loop = PublishLoopStub(controller)

        result = controller.request_samples(mode=MODE_NEXT_SAMPLES, num_samples=0)
        publish_loop.start()

        self.assertTrue(result.success, result.message)
        self.assertEqual(result.published_sample_ids, [])
        self.assertFalse(publish_loop.finished.wait(10 * POLL_INTERVAL_S))
        self.assertEqual(publish_loop.published_sample_ids, [])

    def test_request_publishes_requested_sample_ids_only(self):
        """A request for specific samples skips all samples in between."""
        controller = PlaybackController(start_paused=True)
        publish_loop = PublishLoopStub(controller)
        publish_loop.start()

        result = controller.request_samples(mode=MODE_SAMPLE_IDS, sample_ids=[2, 5, 6])

        self.assertTrue(result.success, result.message)
        self.assertEqual(result.published_sample_ids, [2, 5, 6])
        self.assertEqual(result.published_scene_ids, ["scene_0", "scene_1", "scene_1"])
        self.assertEqual(publish_loop.published_sample_ids, [2, 5, 6])
        self.assertEqual(publish_loop.skipped_sample_ids, [0, 1, 3, 4])

    def test_request_for_passed_sample_ids_fails(self):
        """Samples that playback has already passed are reported as not published."""
        controller = PlaybackController(start_paused=True)
        publish_loop = PublishLoopStub(controller)
        publish_loop.start()

        controller.request_samples(mode=MODE_SAMPLE_IDS, sample_ids=[4])
        result = controller.request_samples(mode=MODE_SAMPLE_IDS, sample_ids=[1, 7])

        self.assertFalse(result.success)
        self.assertIn("had already been passed", result.message)
        self.assertEqual(result.published_sample_ids, [7])
        self.assertEqual(publish_loop.published_sample_ids, [4, 7])

    def test_request_for_all_samples_answers_at_the_end_of_the_dataset(self):
        """A request for all remaining samples publishes them and reports the end of the dataset."""
        controller = PlaybackController(start_paused=True)
        publish_loop = PublishLoopStub(controller)
        publish_loop.start()

        controller.request_samples(mode=MODE_NEXT_SAMPLES, num_samples=1)
        result = controller.request_samples(mode=MODE_ALL_SAMPLES)

        self.assertTrue(result.success, result.message)
        self.assertTrue(result.end_of_dataset)
        self.assertEqual(result.published_sample_ids, list(range(1, NUM_SAMPLES)))
        publish_loop.join()

    def test_request_answered_when_dataset_ends_before_all_samples_are_published(self):
        """A request that outlives the dataset is answered once the last sample has been published."""
        controller = PlaybackController(start_paused=True)
        publish_loop = PublishLoopStub(controller)
        publish_loop.start()

        result = controller.request_samples(mode=MODE_NEXT_SAMPLES, num_samples=NUM_SAMPLES + 5)

        self.assertFalse(result.success)
        self.assertTrue(result.end_of_dataset)
        self.assertEqual(result.published_sample_ids, list(range(NUM_SAMPLES)))
        publish_loop.join()

    def test_concurrent_request_is_rejected(self):
        """Only one request is processed at a time."""
        controller = PlaybackController(start_paused=True)
        publish_loop = PublishLoopStub(controller, num_samples=NUM_SLOW_SAMPLES, sample_interval_s=SLOW_SAMPLE_INTERVAL_S)
        publish_loop.start()

        pending_request = SampleRequestStub(controller, mode=MODE_NEXT_SAMPLES, num_samples=NUM_SLOW_SAMPLES)
        publish_loop.wait_for_published(1)
        rejected = controller.request_samples(mode=MODE_NEXT_SAMPLES, num_samples=1)

        self.assertFalse(rejected.success)
        self.assertIn("another sample request", rejected.message)
        self.assertTrue(pending_request.await_result().success)

    def test_stopped_playback_answers_pending_and_further_requests(self):
        """Stopping playback answers a pending request and rejects further ones."""
        controller = PlaybackController(start_paused=True)

        pending_request = SampleRequestStub(controller, mode=MODE_NEXT_SAMPLES, num_samples=1)
        time.sleep(10 * POLL_INTERVAL_S)
        controller.stop()

        self.assertFalse(pending_request.await_result().success)
        self.assertFalse(controller.request_samples(mode=MODE_ALL_SAMPLES).success)

    def test_unknown_request_mode_is_rejected(self):
        """A request with an unsupported mode does not take control of the playback."""
        controller = PlaybackController()
        publish_loop = PublishLoopStub(controller)

        result = controller.request_samples(mode=42)
        publish_loop.start()
        publish_loop.join()

        self.assertFalse(result.success)
        self.assertIn("42", result.message)
        self.assertEqual(publish_loop.published_sample_ids, list(range(NUM_SAMPLES)))


if __name__ == "__main__":
    unittest.main()
