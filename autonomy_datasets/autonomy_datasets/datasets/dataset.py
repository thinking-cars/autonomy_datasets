# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

from abc import ABC, abstractmethod
from typing import Any, Callable, Dict, Iterable, Iterator, Optional, Tuple

from autonomy_datasets.datasets.meta_info import meta_info_topic


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

    def __init__(self, data_publishers: Dict[str, Any], topic_filter: Optional[Callable[[str], bool]] = None) -> None:
        """Initialize the dataset adapter metadata and publisher mapping.

        Args:
            data_publishers: Mapping of topic names to publisher instances.
            topic_filter: Returns whether a topic the adapter provides is published, e.g. a
                :class:`~autonomy_datasets.datasets.topic_filter.TopicFilter`; all topics are
                published if not given.
        """
        self.data_publishers = data_publishers
        self.topic_filter = topic_filter if topic_filter is not None else (lambda topic: True)
        self.version = self.VERSION
        self.release_notes = self.RELEASE_NOTES

    def register_topic(self, topic: str) -> bool:
        """Registers a topic the adapter provides for publishing, unless the topic filter rejects it.

        The publisher of a registered topic is created by the node; the adapter adds messages to
        its samples only for the topics it has registered, and skips loading the data of the
        others where possible.

        Returns:
            Whether the topic is published.
        """
        if not self.topic_filter(topic):
            return False
        self.data_publishers.setdefault(topic, None)
        return True

    def register_topics(self, topics: Iterable[str]) -> bool:
        """Registers several topics for publishing, see :meth:`register_topic`.

        Returns:
            Whether any of the topics is published.
        """
        # every topic is registered before checking whether any is published, which a generator
        # passed to any() would stop doing at the first published topic
        registered = [self.register_topic(topic) for topic in topics]
        return any(registered)

    def register_object_list(self, object_list_topic: str) -> bool:
        """Registers an object list topic and its meta information topic, see :meth:`register_topic`.

        Returns:
            Whether the object list or its meta information is published.
        """
        return self.register_topics([object_list_topic, meta_info_topic(object_list_topic)])

    def publishes(self, topic: str) -> bool:
        """Returns whether a topic has been registered for publishing."""
        return topic in self.data_publishers

    @abstractmethod
    def generate_samples(self) -> Iterator[Tuple[int, Dict[str, Any]]]:
        """Generate samples from the dataset as ROS messages.

        A sample holds the topics it carries data for; a topic a sample leaves out is neither
        published nor written to the rosbag for that sample.

        Yields:
            Tuples of (sample_index, sample_dict) where sample_dict maps
            topic names to ROS messages.
        """
        pass
