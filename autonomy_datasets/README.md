# `autonomy_datasets`

Integrates automated driving datasets into the ROS 2 ecosystem

- [Launch Files](#launch-files)

## Launch Files

### [`autonomy_datasets.launch.py`](launch/autonomy_datasets.launch.py)

| Argument | Default | Description |
| --- | --- | --- |
| `image_topic` | `"/autonomy_datasets/camera/image_raw"` | TODO |
| `camera_info_topic` | `"/autonomy_datasets/camera/camera_info"` | TODO |
| `lidar_point_cloud_topic` | `"/autonomy_datasets/lidar/point_cloud"` | TODO |
| `radar_point_cloud_topic` | `"/autonomy_datasets/radar/point_cloud"` | TODO |
| `object_list_2d_topic` | `"/autonomy_datasets/object_list_2d"` | TODO |
| `object_list_3d_topic` | `"/autonomy_datasets/object_list_3d"` | TODO |
| `name` | `"datasets"` | node name |
| `namespace` | `""` | TODO |
| `params` | `os.path.join(get_package_share_directory("autonomy_datasets"), "config", "params.yml")` | path to parameter file |
| `log_level` | `"info"` | ROS logging level (debug, info, warn, error, fatal) |
| `use_sim_time` | `"true"` | use simulation clock |
| `datasets_path` | `"/datasets"` | TODO |
| `start_paused` | `"false"` | start playback in paused mode |
| `target_frame_rate` | `"1.0"` | target frame rate for publishing samples in Hz (0 = unlimited) |
| `publish_samples` | `"true"` | TODO |
| `write_rosbag` | `"true"` | TODO |
| `wait_for_ack` | `"true"` | TODO |
| `rviz` | `"yes"` | TODO |
