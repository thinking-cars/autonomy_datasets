from abc import ABC, abstractmethod
from typing import Any, Dict, Iterator, Tuple


class DatasetAdapter(ABC):
    """Abstract base class for dataset adapters that convert datasets to ROS 2 messages."""

    def __init__(self, data_publishers: Dict[str, Any]) -> None:
        self.data_publishers = data_publishers

    @abstractmethod
    def generate_samples(self) -> Iterator[Tuple[int, Dict[str, Any]]]:
        """Generate samples from the dataset as ROS messages.

        Yields:
            Tuples of (sample_index, sample_dict) where sample_dict maps
            topic names to ROS messages.
        """
        pass
