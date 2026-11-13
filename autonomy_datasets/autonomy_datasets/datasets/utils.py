# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

from builtin_interfaces.msg import Time
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.logging import get_logger
from rosgraph_msgs.msg import Clock


class DatasetLogger:
    """Logger of a dataset module, named as a child of the node's logger, e.g. 'datasets.rosbag'.

    Dataset modules create their logger at import time, before the node and therefore the name of
    its logger are known, so the name is resolved whenever a message is logged.
    """

    #: Name of the parent logger, set to the name of the node's logger once the node is created
    parent_name = "autonomy_datasets"

    def __init__(self, name: str):
        """Constructor

        Args:
            name (str): name of the logger below the parent logger
        """
        self._name = name
        # one logger per resolved name, as rclpy rejects a call site whose logger name changes
        self._loggers: dict[str, RcutilsLogger] = {}

    def __getattr__(self, attribute: str):
        """Forward attribute access, e.g. the logging methods, to the logger of the resolved name."""
        name = f"{DatasetLogger.parent_name}.{self._name}"
        if name not in self._loggers:
            self._loggers[name] = get_logger(name)
        return getattr(self._loggers[name], attribute)


def timestamp_micros_to_clock(timestamp_micros: int) -> Clock:
    """Convert microsecond timestamp to ROS Clock message."""
    timestamp_micros = max(0, timestamp_micros)
    sec = int(timestamp_micros // 1_000_000)
    nanosec = int((timestamp_micros % 1_000_000) * 1_000)
    return Clock(clock=Time(sec=sec, nanosec=nanosec))
