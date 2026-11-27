# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

"""Report of the scene that is currently published and of the samples published within it.

On a terminal, the scene and its count of published samples are shown in a single line that is
updated in place with every published sample. The line is written to the controlling terminal
directly, as ``ros2 launch`` only forwards complete lines of the node's output. Without a terminal,
e.g. when the node runs in a container without a TTY, a log line is written when a scene starts
and once it has been published.
"""

import os
from typing import Any, Optional


class SceneProgress:
    """Reports the scene that is currently published and the number of samples published in it."""

    def __init__(self, logger: Any):
        """Constructor

        Args:
            logger (Any): logger used if no terminal is available
        """
        self._logger = logger
        try:
            self._tty: Optional[Any] = open("/dev/tty", "w")
        except OSError:
            self._tty = None
        self._scene_label: Optional[str] = None
        self._published_samples = 0

    def sample_published(self, scene_number: int, scene_id: str) -> None:
        """Report a published sample, starting the report of a new scene if the sample starts one.

        Args:
            scene_number (int): 1-based number of the scene the sample belongs to
            scene_id (str): ID of the scene the sample belongs to
        """
        scene_label = f"Scene {scene_number}: {scene_id}"
        if scene_label != self._scene_label:
            self.finish_scene()
            self._scene_label = scene_label
            if self._tty is None:
                self._logger.info(f"Publishing {scene_label}")
        self._published_samples += 1
        if self._tty is not None:
            line = f"{scene_label} | {self._published_samples} sample(s) published"
            try:
                # truncated to the terminal width, as a wrapped line can not be updated in place
                line = line[: os.get_terminal_size(self._tty.fileno()).columns - 1]
            except OSError:
                pass
            # return to the start of the line and clear it before rewriting it
            self._tty.write(f"\r\x1b[2K{line}")
            self._tty.flush()

    def finish_scene(self) -> None:
        """Complete the report of the current scene, if any."""
        if self._scene_label is None:
            return
        if self._tty is not None:
            self._tty.write("\n")
            self._tty.flush()
        else:
            self._logger.info(f"Published {self._published_samples} sample(s) of {self._scene_label}")
        self._scene_label = None
        self._published_samples = 0

    def close(self) -> None:
        """Complete the report of the current scene and release the terminal."""
        self.finish_scene()
        if self._tty is not None:
            self._tty.close()
            self._tty = None
