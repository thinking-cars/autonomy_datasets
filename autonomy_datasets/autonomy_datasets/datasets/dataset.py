# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

from abc import ABC, abstractmethod
from typing import Any, Dict, Iterator, Tuple


class DatasetAdapter(ABC):
    """Abstract base class for dataset adapters that convert datasets to ROS 2 messages."""

    def __init__(self, data_publishers: Dict[str, Any], version: str, release_notes: Dict[str, str]) -> None:
        """Initialize the dataset adapter metadata and publisher mapping.

        Args:
            data_publishers: Mapping of topic names to publisher instances.
            version: Dataset version string.
            release_notes: Mapping of release note keys to descriptions.
        """
        self.data_publishers = data_publishers
        self.version = version
        self.release_notes = release_notes

    @abstractmethod
    def generate_samples(self) -> Iterator[Tuple[int, Dict[str, Any]]]:
        """Generate samples from the dataset as ROS messages.

        Yields:
            Tuples of (sample_index, sample_dict) where sample_dict maps
            topic names to ROS messages.
        """
        pass
