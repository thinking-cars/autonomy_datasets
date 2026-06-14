#!/usr/bin/env python3

# Copyright Thinking Cars GmbH
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    """Generate and return the launch description for autonomy_datasets and optional RViz."""

    remappable_topics = []
    args = [
        DeclareLaunchArgument(
            "dataset",
            default_value="nvidia_physicalai_av_dataset",
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
            "loop",
            default_value="false",
            description="restart from the beginning after publishing all samples",
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
                {"loop": LaunchConfiguration("loop")},
            ],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            remappings=[(la.default_value[0].text, LaunchConfiguration(la.name)) for la in remappable_topics],
            output="screen",
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration("rviz"), "' != 'only'"])),
        ),
        ExecuteProcess(
            cmd=[
                "rviz2",
                "--display-config",
                os.path.join(
                    get_package_share_directory("autonomy_datasets"),
                    "config",
                    "config.rviz",
                ),
                "--ros-args",
                "--log-level",
                LaunchConfiguration("log_level"),
                "-p",
                "use_sim_time:=" + str(bool(LaunchConfiguration("use_sim_time"))),
            ],
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
            # disable xet [https://github.com/huggingface/hf_transfer/issues/30#issuecomment-2878604131]
            SetEnvironmentVariable(name="HF_HUB_DISABLE_XET", value="1"),
            *nodes,
        ]
    )
