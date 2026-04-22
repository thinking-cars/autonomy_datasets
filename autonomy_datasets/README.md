# `autonomy_datasets`

Integrates automated driving datasets into the ROS 2 ecosystem

- [Launch Files](#launch-files)

## Launch Files

### [`autonomy_datasets_launch.py`](launch/autonomy_datasets_launch.py)

| Argument | Default | Description |
| --- | --- | --- |
| `image_topic` | `"~/image"` |  |
| `pointcloud_topic` | `"~/pointcloud"` |  |
| `objects_topic` | `"~/objects"` |  |
| `name` | `"autonomy_datasets"` | node name |
| `namespace` | `"autonomy_hub"` | node namespace |
| `params` | `os.path.join(get_package_share_directory("autonomy_datasets"), "config", "params.yml")` | path to parameter file |
| `log_level` | `"info"` | ROS logging level (debug, info, warn, error, fatal) |
| `use_sim_time` | `"false"` | use simulation clock |
