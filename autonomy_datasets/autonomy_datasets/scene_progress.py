# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

"""Report of the samples published within the scene that is currently published.

The number of samples published in a scene is logged once the scene has been published, so that it
reaches log files and ``/rosout``; the start of a scene is logged by the node. On a terminal, the
scene and its count of published samples are shown in addition in a status line that is updated with
every published sample. It is not shown without a terminal, e.g. when the node runs in a container
without a TTY.

The status line is written to the controlling terminal directly, as ``ros2 launch`` only forwards
complete lines of the node's output. As the log lines reach the terminal through ``ros2 launch`` at
an unknown time, the status line is kept on the bottom row of the terminal, which is excluded from
the area the log lines scroll in. Every update of the status line is a single write that restores
the cursor of the log output, and writes to a terminal are not interleaved with those of other
processes, so that log lines never end up in the status line, whenever they arrive.
"""

import os
from typing import Any, Optional

# escape sequences of VT100 compatible terminals
_SAVE_CURSOR = "\x1b7"
_RESTORE_CURSOR = "\x1b8"
_CURSOR_UP = "\x1b[1A"
_CLEAR_LINE = "\x1b[2K"
_CLEAR_BELOW_CURSOR = "\x1b[J"


def _scroll_area(rows: int) -> str:
    """Return the sequence restricting scrolling to the first rows of the terminal, which keeps the cursor."""
    # setting the scroll area moves the cursor to the top left of the terminal
    return f"{_SAVE_CURSOR}\x1b[1;{rows}r{_RESTORE_CURSOR}"


def _move_to_row(row: int) -> str:
    """Return the sequence moving the cursor to the start of a row of the terminal."""
    return f"\x1b[{row};1H"


class SceneProgress:
    """Reports the scene that is currently published and the number of samples published in it."""

    def __init__(self, logger: Any):
        """Constructor

        Args:
            logger (Any): logger reporting the samples published in a scene
        """
        self._logger = logger
        try:
            # unbuffered, so that every sequence is written by a single write call
            self._tty: Optional[Any] = open("/dev/tty", "wb", buffering=0)
        except OSError:
            self._tty = None
        # number of rows of the terminal the scroll area has been set up for, if any
        self._terminal_rows: Optional[int] = None
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
        self._published_samples += 1
        self._draw_status_line(f"{scene_label} | {self._published_samples} sample(s) published")

    def finish_scene(self) -> None:
        """Complete the report of the current scene, if any."""
        if self._scene_label is None:
            return
        self._draw_status_line("")
        self._logger.info(f"Published {self._published_samples} sample(s) of {self._scene_label}")
        self._scene_label = None
        self._published_samples = 0

    def close(self) -> None:
        """Complete the report of the current scene and release the terminal."""
        self.finish_scene()
        if self._tty is None:
            return
        if self._terminal_rows is not None:
            try:
                # the whole terminal scrolls again, and the empty status line below the cursor is removed
                self._write(f"{_scroll_area(self._terminal_rows)}{_SAVE_CURSOR}{_CLEAR_BELOW_CURSOR}{_RESTORE_CURSOR}")
            except OSError:
                pass
            self._terminal_rows = None
        self._tty.close()
        self._tty = None

    def _draw_status_line(self, line: str) -> None:
        """Draw the status line on the bottom row of the terminal, if there is one."""
        if self._tty is None:
            return
        try:
            columns, rows = os.get_terminal_size(self._tty.fileno())
            if rows < 2:
                return
            prefix = ""
            if rows != self._terminal_rows:
                if self._terminal_rows is None:
                    # scrolls the log output up by a row, so that the cursor is above the bottom row
                    # once it is excluded from the scroll area
                    prefix = f"\n{_CURSOR_UP}"
                # set up again if the terminal is resized
                prefix += _scroll_area(rows - 1)
                self._terminal_rows = rows
            # truncated to the terminal width, as a wrapped line would scroll the terminal
            self._write(f"{prefix}{_SAVE_CURSOR}{_move_to_row(rows)}{_CLEAR_LINE}{line[: columns - 1]}{_RESTORE_CURSOR}")
        except OSError:
            pass

    def _write(self, sequence: str) -> None:
        """Write a sequence to the terminal at once, so that the output of other processes does not interrupt it."""
        assert self._tty is not None
        self._tty.write(sequence.encode())
