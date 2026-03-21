# `autonomy_datasets`

Integrates automated driving datasets into the ROS 2 ecosystem

- [Launch Files](#launch-files)

## Launch Files

### [`autonomy_datasets.launch.py`](launch/autonomy_datasets.launch.py)

| Argument | Default | Description |
| --- | --- | --- |
| `image_topic` | `"/autonomy_datasets/camera/image_raw"` |  |
| `camera_info_topic` | `"/autonomy_datasets/camera/camera_info"` |  |
| `point_cloud_topic` | `"/autonomy_datasets/point_cloud"` |  |
| `object_list_2d_topic` | `"/autonomy_datasets/object_list_2d"` |  |
| `object_list_3d_topic` | `"/autonomy_datasets/object_list_3d"` |  |
| `name` | `"datasets"` | node name |
| `namespace` | `"autonomy"` | node namespace |
| `params` | `os.path.join(get_package_share_directory("autonomy_datasets"), "config", "params.yml")` | path to parameter file |
| `log_level` | `"info"` | ROS logging level (debug, info, warn, error, fatal) |
| `use_sim_time` | `"true"` | use simulation clock |
| `datasets_path` | `"/datasets"` |  |
| `start_paused` | `"false"` | start playback in paused mode |
| `target_frame_rate` | `"1.0"` | target frame rate for publishing samples in Hz (0 = unlimited) |
| `rviz` | `"no"` | launch rviz for visualization |
