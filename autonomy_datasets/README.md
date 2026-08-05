# `autonomy_datasets`

Integrates automated driving datasets into the ROS 2 ecosystem

## Nodes

### `autonomy_datasets`

#### Parameters

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `datasets_path` | `string` | `/datasets` | path to datasets directory |
| `dataset` | `string` | `waymo_open_dataset` | name of the dataset to use |
| `dataset_split` | `string` | `validation_mini` | split of the dataset to use |
| `start_paused` | `bool` | `false` | whether to start playback in paused mode |
| `target_frame_rate` | `float` | `0.0` | playback speed multiplier based on recorded timestamps (1.0 = real-time, 2.0 = double speed, 0.0 = unlimited) |
| `publish_samples` | `bool` | `true` | whether to publish samples to ROS topics |
| `write_rosbag` | `bool` | `true` | whether to write samples to rosbag |
| `overwrite_rosbag` | `bool` | `false` | whether to overwrite existing rosbags instead of replaying them |
| `continue` | `bool` | `false` | whether to continue writing rosbags after the latest stored scene |
| `wait_for_ack` | `bool` | `true` | whether to wait for subscriber acknowledgement after publishing |
| `loop` | `bool` | `false` | restart from the beginning after publishing all samples |
| `waymo_lidar_object_list_filter_cam_front` | `bool` | `false` | use only objects covered by front camera |
| `waymo_min_lidar_points_in_bbox` | `int` | `1` | minimum number of lidar points required in a bounding box |
| `publish_ego_data` | `bool` | `true` | whether to publish ego data |
| `publish_camera_images` | `bool` | `true` | whether to publish camera images |
| `publish_camera_all_object_lists` | `bool` | `true` | whether to publish object lists for all cameras |
| `publish_camera_01_object_lists` | `bool` | `true` | whether to publish camera_01 (front) object lists |
| `publish_lidar_01_pointclouds` | `bool` | `true` | whether to publish lidar_01 (top) point clouds |
| `publish_lidar_01_object_lists` | `bool` | `true` | whether to publish lidar_01 (top) object lists |
| `publish_ego_data` | `bool` | `true` | whether to publish ego data |
| `publish_camera_images` | `bool` | `true` | whether to publish camera images |
| `publish_lidar_pointclouds` | `bool` | `true` | whether to publish lidar point clouds |
| `publish_lidar_object_lists` | `bool` | `true` | whether to publish lidar object lists |
| `publish_camera_01_object_lists` | `bool` | `true` | whether to publish camera_01 (front) object lists |
| `publish_ego_data` | `bool` | `true` | whether to publish ego data |
| `publish_camera_images` | `bool` | `true` | whether to publish camera images |
| `publish_lidar_pointclouds` | `bool` | `true` | whether to publish lidar point clouds |
| `publish_lidar_object_lists` | `bool` | `true` | whether to publish lidar object lists |
| `publish_radar_pointclouds` | `bool` | `true` | whether to publish radar point clouds |
| `nvidia_filter_countries` | `string` | - | comma-separated list of countries to include (e.g. 'Germany,France'); if empty, includes all countries |
| `publish_ego_data` | `bool` | `true` | whether to publish ego data |
| `publish_camera_images` | `bool` | `true` | whether to publish camera images |
| `publish_lidar_pointclouds` | `bool` | `true` | whether to publish lidar point clouds |
| `publish_lidar_object_lists` | `bool` | `true` | whether to publish lidar object lists |
| `driving_auto_download` | `bool` | `true` | whether to download and extract DrivIng when it is not available locally |
| `driving_download_workers` | `int` | `8` | number of DrivIng archive chunks to download concurrently |
| `driving_rosbag_duration_seconds` | `float` | `20.0` | duration of each DrivIng rosbag scene in seconds |

## Launch Files

### [`autonomy_datasets.launch.py`](launch/autonomy_datasets.launch.py)

| Argument | Default | Description |
| --- | --- | --- |
| `dataset` | `"nvidia_physicalai_av_dataset"` | dataset to be used |
| `config` | `""` | path to a parameter file (inferred from 'dataset' if empty) |
| `name` | `"datasets"` | node name |
| `namespace` | `""` | node namespace |
| `log_level` | `"info"` | ros logging level |
| `use_sim_time` | `"true"` | use sim time |
| `datasets_path` | `"/datasets"` | path where raw datasets are stored |
| `start_paused` | `"false"` | wait for pressing space to start |
| `target_frame_rate` | `"1.0"` | target frame rate |
| `publish_samples` | `"true"` | publish dataset samples as ros messages |
| `write_rosbag` | `"true"` | write dataset samples to rosbag |
| `continue` | `"false"` | continue writing rosbags after the latest stored scene without replaying existing rosbags |
| `overwrite_rosbag` | `"false"` | overwrite existing rosbag instead of replaying |
| `wait_for_ack` | `"true"` | wait for acknowledged receipt of sample data before publishing next sample |
| `loop` | `"false"` | restart from the beginning after publishing all samples |
| `rviz` | `"yes"` | start rviz for visualization |
