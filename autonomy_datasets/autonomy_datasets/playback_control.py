# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

"""Control of which samples the publish loop publishes.

The publish loop of the node asks :class:`PlaybackController` for every sample of the dataset
whether it may be published, which blocks the loop for as long as playback is waiting for a
request. Requests arrive from the callback of the node's ``request_samples`` service, which runs
in another thread and is answered by the publish loop reporting the samples it published back to
the controller.
"""

import threading
from dataclasses import dataclass, field
from typing import Iterable, Optional, Sequence

# Request modes, mirroring the constants of autonomy_datasets_msgs/srv/RequestSamples
MODE_ALL_SAMPLES = 0
MODE_NEXT_SAMPLES = 1
MODE_SAMPLE_IDS = 2

REQUEST_MODES = (MODE_ALL_SAMPLES, MODE_NEXT_SAMPLES, MODE_SAMPLE_IDS)

# Interval in seconds at which a blocked thread re-checks the controller state, so that it stays
# responsive to a KeyboardInterrupt while waiting
WAIT_INTERVAL_S = 0.1

# Maximum number of sample IDs listed in a message before they are summarized
_MAX_LISTED_SAMPLE_IDS = 10


def format_sample_ids(sample_ids: Iterable[int]) -> str:
    """Format sample IDs for log messages, summarizing long lists.

    Args:
        sample_ids (Iterable[int]): sample IDs

    Returns:
        str: comma-separated sample IDs
    """
    ids = list(sample_ids)
    listed = ", ".join(str(sample_id) for sample_id in ids[:_MAX_LISTED_SAMPLE_IDS])
    if len(ids) > _MAX_LISTED_SAMPLE_IDS:
        listed += f", ... ({len(ids)} in total)"
    return listed


@dataclass
class SampleRequestResult:
    """Outcome of a sample request, mirroring the response of the RequestSamples service."""

    #: Whether all requested samples have been published
    success: bool
    #: Description of the outcome, stating which samples could not be published
    message: str
    #: IDs of the samples published while processing the request, in publishing order
    published_sample_ids: list[int] = field(default_factory=list)
    #: ID of the scene each published sample belongs to, parallel to published_sample_ids
    published_scene_ids: list[str] = field(default_factory=list)
    #: Whether the last sample of the playback pass has been published
    end_of_dataset: bool = False


@dataclass
class _SampleRequest:
    """A sample request that is being processed by the publish loop."""

    #: Requested playback mode, one of the MODE_* constants
    mode: int
    #: Number of samples still to be published in MODE_NEXT_SAMPLES
    remaining_samples: int = 0
    #: IDs still to be published in MODE_SAMPLE_IDS
    pending_sample_ids: set[int] = field(default_factory=set)
    #: Requested IDs that the playback has already passed and can therefore not publish
    missed_sample_ids: list[int] = field(default_factory=list)
    #: IDs of the samples published so far, in publishing order
    published_sample_ids: list[int] = field(default_factory=list)
    #: ID of the scene each published sample belongs to, parallel to published_sample_ids
    published_scene_ids: list[str] = field(default_factory=list)
    #: Result of the request, set once the request has been completed
    result: Optional[SampleRequestResult] = None


class PlaybackController:
    """Decides for every sample of the publish loop whether it is published, skipped or delayed.

    Playback is in one of two states:

    - *free-running*: every sample is published as soon as the publish loop reaches it, which is
      the state the node starts in unless it is started paused
    - *request-controlled*: samples are only published while a sample request is being processed;
      playback enters this state with the first request it receives and only leaves it while a
      request for all remaining samples is processed
    """

    def __init__(self, logger=None, start_paused: bool = False):
        """Constructor

        Args:
            logger (Optional[Any], optional): logger used to report state changes
            start_paused (bool, optional): whether playback waits for a request before publishing
        """
        self._logger = logger
        self._condition = threading.Condition()
        self._request_controlled = start_paused
        self._request: Optional[_SampleRequest] = None
        self._stopped = False

    # -- publish loop interface ---------------------------------------------------------------

    def await_sample(self, sample_id: int) -> bool:
        """Block until the publish loop may handle the given sample and report how to handle it.

        Blocks while playback is waiting for a sample request. Samples that are not part of the
        active request are skipped, so that the publish loop advances to the next sample without
        publishing this one.

        Args:
            sample_id (int): ID of the sample the publish loop is about to handle

        Returns:
            bool: whether the sample is to be published
        """
        with self._condition:
            waiting_logged = False
            while not self._stopped:
                request = self._request
                if request is not None:
                    if request.mode != MODE_SAMPLE_IDS:
                        return True
                    self._drop_passed_sample_ids(request, sample_id)
                    if self._request is None:
                        # none of the requested samples can be published anymore, request is completed
                        continue
                    return sample_id in request.pending_sample_ids
                if self._request_controlled:
                    if not waiting_logged:
                        self._log_info("Playback is waiting for a sample request")
                        waiting_logged = True
                    self._condition.wait(WAIT_INTERVAL_S)
                    continue
                return True
            return False

    def sample_published(self, sample_id: int, scene_id: str) -> None:
        """Report a sample that has been published to the active request.

        Completes the request as soon as all requested samples have been published.

        Args:
            sample_id (int): ID of the published sample
            scene_id (str): ID of the scene the published sample belongs to
        """
        with self._condition:
            request = self._request
            if request is None:
                return
            request.published_sample_ids.append(int(sample_id))
            request.published_scene_ids.append(str(scene_id))
            if request.mode == MODE_NEXT_SAMPLES:
                request.remaining_samples -= 1
            elif request.mode == MODE_SAMPLE_IDS:
                request.pending_sample_ids.discard(sample_id)
            if request.mode != MODE_ALL_SAMPLES and not request.remaining_samples and not request.pending_sample_ids:
                self._complete_request(f"published {len(request.published_sample_ids)} sample(s)")

    def pass_finished(self) -> None:
        """Report that the publish loop has reached the end of the dataset.

        Completes an active request, which either publishes all remaining samples or waits for
        samples that the playback pass will not deliver anymore.
        """
        with self._condition:
            request = self._request
            if request is None:
                return
            if request.mode == MODE_ALL_SAMPLES:
                message = f"published all {len(request.published_sample_ids)} remaining sample(s)"
            else:
                message = "reached the end of the dataset"
            self._complete_request(message, end_of_dataset=True)

    def stop(self) -> None:
        """Stop playback control, releasing the publish loop and any pending request."""
        with self._condition:
            self._stopped = True
            if self._request is not None:
                self._complete_request("playback stopped")
            self._condition.notify_all()

    # -- service interface --------------------------------------------------------------------

    def request_samples(self, mode: int, num_samples: int = 0, sample_ids: Sequence[int] = ()) -> SampleRequestResult:
        """Request samples to be published and block until they have been published.

        Takes control of the playback, so that samples are only published while a request is being
        processed.

        Args:
            mode (int): requested playback mode, one of the MODE_* constants
            num_samples (int, optional): number of samples to publish in MODE_NEXT_SAMPLES
            sample_ids (Sequence[int], optional): IDs of the samples to publish in MODE_SAMPLE_IDS

        Returns:
            SampleRequestResult: outcome of the request
        """
        with self._condition:
            if mode not in REQUEST_MODES:
                return SampleRequestResult(success=False, message=f"unknown request mode '{mode}'")
            if self._stopped:
                return SampleRequestResult(success=False, message="playback has finished, no more samples can be published")
            if self._request is not None:
                return SampleRequestResult(success=False, message="another sample request is currently being processed")

            self._request_controlled = True
            if mode == MODE_NEXT_SAMPLES and num_samples <= 0:
                return SampleRequestResult(success=True, message="took control of the playback without publishing a sample")
            if mode == MODE_SAMPLE_IDS and not sample_ids:
                return SampleRequestResult(success=True, message="took control of the playback without publishing a sample")

            request = _SampleRequest(
                mode=mode,
                remaining_samples=int(num_samples) if mode == MODE_NEXT_SAMPLES else 0,
                pending_sample_ids={int(sample_id) for sample_id in sample_ids} if mode == MODE_SAMPLE_IDS else set(),
            )
            self._request = request
            self._condition.notify_all()

            while request.result is None:
                self._condition.wait(WAIT_INTERVAL_S)
            return request.result

    # -- internal helpers ---------------------------------------------------------------------

    def _drop_passed_sample_ids(self, request: _SampleRequest, sample_id: int) -> None:
        """Drop requested sample IDs that the forward-only playback has already passed.

        Completes the request if none of the requested samples can be published anymore. Must be
        called with the lock held.

        Args:
            request (_SampleRequest): active request
            sample_id (int): ID of the sample the publish loop is about to handle
        """
        passed_sample_ids = sorted(pending for pending in request.pending_sample_ids if pending < sample_id)
        if not passed_sample_ids:
            return
        request.pending_sample_ids.difference_update(passed_sample_ids)
        request.missed_sample_ids.extend(passed_sample_ids)
        if not request.pending_sample_ids:
            self._complete_request(f"published {len(request.published_sample_ids)} sample(s)")

    def _complete_request(self, message: str, end_of_dataset: bool = False) -> None:
        """Complete the active request with the samples published for it.

        Must be called with the lock held.

        Args:
            message (str): description of the outcome
            end_of_dataset (bool, optional): whether the last sample of the pass has been published
        """
        request = self._request
        assert request is not None
        if request.missed_sample_ids:
            message += (
                f"; sample(s) {format_sample_ids(request.missed_sample_ids)} had already been passed, " "playback cannot rewind"
            )
        if request.pending_sample_ids:
            message += f"; sample(s) {format_sample_ids(sorted(request.pending_sample_ids))} were not published"
        if request.remaining_samples > 0:
            message += f"; {request.remaining_samples} requested sample(s) were not published"
        request.result = SampleRequestResult(
            success=not request.missed_sample_ids and not request.pending_sample_ids and request.remaining_samples <= 0,
            message=message,
            published_sample_ids=list(request.published_sample_ids),
            published_scene_ids=list(request.published_scene_ids),
            end_of_dataset=end_of_dataset,
        )
        self._request = None
        self._condition.notify_all()

    def _log_info(self, message: str) -> None:
        """Log a message if a logger is available.

        Args:
            message (str): message to log
        """
        if self._logger is not None:
            self._logger.info(message)
