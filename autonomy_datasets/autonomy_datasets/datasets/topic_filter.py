# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

"""Selection of the topics a dataset is published on by patterns of topic names.

The topics of all datasets follow the same naming scheme, e.g. ``camera_01/image_raw``,
``lidar_01/point_cloud`` or ``object_list/lidar_01``, so that the published topics are selected by
the parameters ``include_topics`` and ``exclude_topics`` instead of a parameter per topic or
sensor. A topic is published if it matches any include pattern and no exclude pattern.

Patterns are shell-style wildcards (see :mod:`fnmatch`) matched against the complete topic name
without leading slash, where ``*`` also matches slashes. A pattern also matches the topics below
the namespace it names, e.g. ``object_list/lidar_01`` matches ``object_list/lidar_01/meta_info``,
and ``camera_01`` matches ``camera_01/image_raw`` and ``camera_01/camera_info``.
"""

from fnmatch import fnmatchcase
from typing import Iterable, List


def _normalize(name: str) -> str:
    """Returns a topic name or pattern without leading and trailing slashes."""
    return name.strip().strip("/")


class TopicFilter:
    """Decides whether a topic is published by include and exclude patterns of topic names."""

    def __init__(self, include: Iterable[str] = ("*",), exclude: Iterable[str] = ()):
        """Creates a topic filter.

        Args:
            include: Patterns of the topics to publish; empty patterns are ignored.
            exclude: Patterns of the topics not to publish, even if they match an include
                pattern; empty patterns are ignored.
        """
        self.include: List[str] = [_normalize(pattern) for pattern in include if _normalize(pattern)]
        self.exclude: List[str] = [_normalize(pattern) for pattern in exclude if _normalize(pattern)]

    @staticmethod
    def matches(topic: str, pattern: str) -> bool:
        """Returns whether a topic matches a pattern, or lies below the namespace the pattern matches."""
        topic, pattern = _normalize(topic), _normalize(pattern)
        return fnmatchcase(topic, pattern) or fnmatchcase(topic, f"{pattern}/*")

    def __call__(self, topic: str) -> bool:
        """Returns whether a topic is published."""
        return any(self.matches(topic, pattern) for pattern in self.include) and not any(
            self.matches(topic, pattern) for pattern in self.exclude
        )
