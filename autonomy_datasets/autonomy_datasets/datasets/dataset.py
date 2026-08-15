# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

from abc import ABC, abstractmethod
from typing import Any, Dict, Iterator, Tuple


class DatasetAdapter(ABC):
    """Abstract base class for dataset adapters that convert datasets to ROS 2 messages.

    Subclasses declare the version of their ROS conversion via the class attributes
    :attr:`VERSION` and :attr:`RELEASE_NOTES`. They are class attributes so that the version
    can be resolved before an adapter is instantiated, which is required to locate the
    version-specific rosbag directory of a dataset.
    """

    #: Version of the ROS conversion implemented by the adapter.
    VERSION: str = "0.0.0"

    #: Mapping of version strings to their release notes.
    RELEASE_NOTES: Dict[str, str] = {}

    def __init__(self, data_publishers: Dict[str, Any]) -> None:
        """Initialize the dataset adapter metadata and publisher mapping.

        Args:
            data_publishers: Mapping of topic names to publisher instances.
        """
        self.data_publishers = data_publishers
        self.version = self.VERSION
        self.release_notes = self.RELEASE_NOTES

    @abstractmethod
    def generate_samples(self) -> Iterator[Tuple[int, Dict[str, Any]]]:
        """Generate samples from the dataset as ROS messages.

        Yields:
            Tuples of (sample_index, sample_dict) where sample_dict maps
            topic names to ROS messages.
        """
        pass
