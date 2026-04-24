#!/usr/bin/env python3

# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    """Generate and return the launch description for autonomy_datasets and optional RViz."""

    remappable_topics = []
    args = [
        DeclareLaunchArgument(
            "dataset",
            default_value="nuscenes",
            description="dataset name",
            choices=["nvidia_physicalai_av_dataset", "waymo_open_dataset", "nuscenes"],
        ),
        DeclareLaunchArgument("name", default_value="datasets", description="node name"),
        DeclareLaunchArgument("namespace", default_value="", description="node namespace"),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="ROS logging level (debug, info, warn, error, fatal)",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="use simulation clock"),
        DeclareLaunchArgument("datasets_path", default_value="/datasets"),
        DeclareLaunchArgument(
            "start_paused",
            default_value="false",
            description="start playback in paused mode",
        ),
        DeclareLaunchArgument(
            "target_frame_rate",
            default_value="1.0",
            description="target frame rate for publishing samples in Hz (0 = unlimited)",
        ),
        DeclareLaunchArgument(
            "publish_samples",
            default_value="true",
            description="publish samples to ROS topics",
        ),
        DeclareLaunchArgument("write_rosbag", default_value="true", description="write samples to rosbag"),
        DeclareLaunchArgument(
            "overwrite_rosbag",
            default_value="false",
            description="overwrite existing rosbags instead of replaying them",
        ),
        DeclareLaunchArgument(
            "wait_for_ack",
            default_value="true",
            description="wait for subscriber acknowledgement after publishing",
        ),
        DeclareLaunchArgument(
            "start_zenoh_router",
            default_value="true",
            description="start a local Zenoh router alongside the launched nodes",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="yes",
            choices=["no", "yes", "only"],
            description="launch rviz for visualization",
        ),
        *remappable_topics,
    ]

    nodes = [
        ExecuteProcess(
            cmd=["bash", "-lc", "ros2 run rmw_zenoh_cpp rmw_zenohd"],
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_zenoh_router")),
        ),
        Node(
            package="autonomy_datasets",
            executable="autonomy_datasets",
            namespace=LaunchConfiguration("namespace"),
            name=LaunchConfiguration("name"),
            parameters=[
                [
                    os.path.join(get_package_share_directory("autonomy_datasets"), "config"),
                    "/params_",
                    LaunchConfiguration("dataset"),
                    ".yml",
                ],
                {"datasets_path": LaunchConfiguration("datasets_path")},
                {"start_paused": LaunchConfiguration("start_paused")},
                {"target_frame_rate": LaunchConfiguration("target_frame_rate")},
                {"publish_samples": LaunchConfiguration("publish_samples")},
                {"write_rosbag": LaunchConfiguration("write_rosbag")},
                {"overwrite_rosbag": LaunchConfiguration("overwrite_rosbag")},
                {"wait_for_ack": LaunchConfiguration("wait_for_ack")},
            ],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            remappings=[(la.default_value[0].text, LaunchConfiguration(la.name)) for la in remappable_topics],
            output="screen",
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration("rviz"), "' != 'only'"])),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            namespace=LaunchConfiguration("namespace"),
            name=PythonExpression(["'", LaunchConfiguration("name"), "_rviz'"]),
            arguments=[
                "--display-config",
                os.path.join(
                    get_package_share_directory("autonomy_datasets"),
                    "config",
                    "config.rviz",
                ),
                "--ros-args",
                "--log-level",
                LaunchConfiguration("log_level"),
            ],
            remappings=[(la.default_value[0].text, LaunchConfiguration(la.name)) for la in remappable_topics],
            output="screen",
            condition=IfCondition(
                PythonExpression(
                    [
                        "'",
                        LaunchConfiguration("rviz"),
                        "' == 'yes' or '",
                        LaunchConfiguration("rviz"),
                        "' == 'only'",
                    ]
                )
            ),
        ),
    ]

    return LaunchDescription(
        [
            *args,
            SetParameter("use_sim_time", LaunchConfiguration("use_sim_time")),
            *nodes,
        ]
    )
