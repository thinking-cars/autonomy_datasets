#!/usr/bin/env python3

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter


def generate_launch_description():

    remappable_topics = [
        DeclareLaunchArgument("image_topic", default_value="/autonomy_datasets/camera/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/autonomy_datasets/camera/camera_info"),
        DeclareLaunchArgument("point_cloud_topic", default_value="/autonomy_datasets/point_cloud"),
        DeclareLaunchArgument("object_list_2d_topic", default_value="/autonomy_datasets/object_list_2d"),
        DeclareLaunchArgument("object_list_3d_topic", default_value="/autonomy_datasets/object_list_3d"),
    ]
    args = [
        DeclareLaunchArgument("name", default_value="datasets", description="node name"),
        DeclareLaunchArgument("namespace", default_value="autonomy", description="node namespace"),
        DeclareLaunchArgument("params", default_value=os.path.join(get_package_share_directory("autonomy_datasets"), "config", "params.yml"), description="path to parameter file"),
        DeclareLaunchArgument("log_level", default_value="info", description="ROS logging level (debug, info, warn, error, fatal)"),
        DeclareLaunchArgument("use_sim_time", default_value="false", description="use simulation clock"),
        DeclareLaunchArgument("datasets_path", default_value="/datasets"),
        DeclareLaunchArgument("start_paused", default_value="true", description="start playback in paused mode"),
        DeclareLaunchArgument("rviz", default_value="true", description="launch rviz for visualization"),
        *remappable_topics,
    ]

    nodes = [
        Node(
            package="autonomy_datasets",
            executable="autonomy_datasets",
            namespace=LaunchConfiguration("namespace"),
            name=LaunchConfiguration("name"),
            parameters=[
                LaunchConfiguration("params"),
                {"datasets_path": LaunchConfiguration("datasets_path")},
                {"start_paused": LaunchConfiguration("start_paused")},

            ],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            remappings=[(la.default_value[0].text, LaunchConfiguration(la.name)) for la in remappable_topics],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            namespace=LaunchConfiguration("namespace"),
            name=PythonExpression(["'", LaunchConfiguration("name"), "_rviz'"]),
            arguments=["--display-config", os.path.join(get_package_share_directory("autonomy_datasets"), "config", "config.rviz"), "--ros-args", "--log-level", LaunchConfiguration("log_level")],
            remappings=[(la.default_value[0].text, LaunchConfiguration(la.name)) for la in remappable_topics],
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
        )
    ]

    return LaunchDescription([
        *args,
        SetParameter("use_sim_time", LaunchConfiguration("use_sim_time")),
        *nodes,
    ])
