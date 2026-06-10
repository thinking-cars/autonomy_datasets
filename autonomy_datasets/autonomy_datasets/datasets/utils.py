# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock


def timestamp_micros_to_clock(timestamp_micros: int) -> Clock:
    """Convert microsecond timestamp to ROS Clock message."""
    timestamp_micros = max(0, timestamp_micros)
    sec = int(timestamp_micros // 1_000_000)
    nanosec = int((timestamp_micros % 1_000_000) * 1_000)
    return Clock(clock=Time(sec=sec, nanosec=nanosec))
