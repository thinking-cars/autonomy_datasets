# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

"""Helpers for publishing dataset meta information next to object lists.

perception_msgs/Object has no field for dataset-specific annotations such as the original class
name of a dataset or the number of lidar points inside a bounding box. Those annotations are
therefore published as autonomy_datasets_msgs/ObjectListMetaInfo on a separate
"<object list topic>/meta_info" topic; consumers correlate both topics via the header stamp of
the object list and the object id.
"""

from typing import Any, Dict, Iterable, Tuple

from autonomy_datasets_msgs.msg import ObjectListMetaInfo, ObjectMetaInfo
from diagnostic_msgs.msg import KeyValue
from perception_msgs.msg import ObjectList
from std_msgs.msg import Header

META_INFO_TOPIC_SUFFIX = "meta_info"


def meta_info_topic(object_list_topic: str) -> str:
    """Return the meta information topic belonging to an object list topic."""
    return f"{object_list_topic}/{META_INFO_TOPIC_SUFFIX}"


def is_meta_info_topic(topic: str) -> bool:
    """Return whether a topic carries object list meta information."""
    return topic.rsplit("/", 1)[-1] == META_INFO_TOPIC_SUFFIX


def add_object_list_publishers(data_publishers: Dict[str, Any], object_list_topic: str) -> None:
    """Register an object list topic and its meta information topic for publisher creation."""
    data_publishers[object_list_topic] = None
    data_publishers[meta_info_topic(object_list_topic)] = None


def set_object_list_sample(
    sample: Dict[str, Any],
    object_list_topic: str,
    object_list_msg: ObjectList,
    meta_info_msg: ObjectListMetaInfo,
) -> None:
    """Store an object list and its meta information under their respective topics in a sample."""
    sample[object_list_topic] = object_list_msg
    sample[meta_info_topic(object_list_topic)] = meta_info_msg


def create_object_list_meta_info(object_list_msg: ObjectList, scene_id: str) -> ObjectListMetaInfo:
    """Create the meta information message belonging to an object list, without object entries.

    Args:
        object_list_msg: Object list the meta information belongs to; its header must be filled
            already, as it is copied to associate both messages.
        scene_id: ID of the dataset scene/sequence/clip the object list was taken from.
    """
    return ObjectListMetaInfo(
        header=Header(
            frame_id=object_list_msg.header.frame_id,
            stamp=object_list_msg.header.stamp,
        ),
        scene_id=str(scene_id),
    )


def add_object_meta_info(meta_info_msg: ObjectListMetaInfo, object_id: int, info: Iterable[Tuple[str, Any]]) -> None:
    """Append the meta information of a single object.

    Args:
        meta_info_msg: Meta information message of the object list the object belongs to.
        object_id: ID of the object, matching perception_msgs/Object.id.
        info: Key/value pairs of dataset-specific annotations. Values are converted to strings and
            keys may repeat, e.g. for objects carrying multiple attributes.
    """
    meta_info_msg.objects.append(
        ObjectMetaInfo(
            id=int(object_id),
            info=[KeyValue(key=key, value=str(value)) for key, value in info],
        )
    )
