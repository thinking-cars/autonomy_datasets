# `ros2_demo_package`

TODO

- [Nodes](#nodes)
  - [ros2_demo_node](#ros2_demo_node)
- [Launch Files](#launch-files)

## Nodes

### `ros2_demo_node`

```mermaid
flowchart LR
    NODE("ros2_demo_node")
    S0:::hidden -->|~/input| NODE
    SS0:::hidden o--o|~/service| NODE
    NODE -->|~/output| P0:::hidden
    AS0:::hidden o-.-o|~/action| NODE
    classDef hidden display: none;
```

#### Subscribed Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/input` | `geometry_msgs/msg/PointStamped` | |

#### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/output` | `geometry_msgs/msg/PointStamped` | |

#### Service Servers

| Service | Type | Description |
| --- | --- | --- |
| `~/service` | `std_srvs/srv/SetBool` | |

#### Action Servers

| Action | Type | Description |
| --- | --- | --- |
| `~/action` | `ros2_demo_package_interfaces/action/Fibonacci` | |

#### Parameters

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `param` | `float` | `1.0` | TODO |

## Launch Files

### [`ros2_demo_node_launch.py`](launch/ros2_demo_node_launch.py)

| Argument | Default | Description |
| --- | --- | --- |
| `input_topic` | `"~/input"` |  |
| `output_topic` | `"~/output"` |  |
| `service` | `"~/service"` |  |
| `name` | `"ros2_demo_node"` | node name |
| `namespace` | `""` | node namespace |
| `params` | `os.path.join(get_package_share_directory("ros2_demo_package"), "config", "params.yml")` | path to parameter file |
| `log_level` | `"info"` | ROS logging level (debug, info, warn, error, fatal) |
| `use_sim_time` | `"false"` | use simulation clock |
