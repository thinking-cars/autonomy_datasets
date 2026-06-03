# `autonomy_datasets`

Integrates automated driving datasets into the ROS 2 ecosystem

## Launch Files

### [`autonomy_datasets.launch.py`](launch/autonomy_datasets.launch.py)

| Argument | Default | Description |
| --- | --- | --- |
| `dataset` | `"nvidia_physicalai_av_dataset"` | dataset to be used |
| `name` | `"datasets"` | node name |
| `namespace` | `""` | node namespace |
| `log_level` | `"info"` | ros logging level |
| `use_sim_time` | `"true"` | use sim time |
| `datasets_path` | `"/datasets"` | path where raw datasets are stored |
| `start_paused` | `"false"` | wait for pressing space to start |
| `target_frame_rate` | `"1.0"` | target frame rate |
| `publish_samples` | `"true"` | publish dataset samples as ros messages |
| `write_rosbag` | `"true"` | write dataset samples to rosbag |
| `overwrite_rosbag` | `"false"` | overwrite existing rosbag instead of replaying |
| `wait_for_ack` | `"true"` | wait for acknowledged receipt of sample data before publishing next sample |
| `rviz` | `"yes"` | start rviz for visualization |
