# ros2_demo_repository

- [Auto-generated Package Documentation](#auto-generated-package-documentation)

TODO
- README badges, including license, version, CI status
- note that this is a component of the stack, link to main repo
- short description and purpose of the package
- screenshot/catchy media showing package's output or similar (if applicable)
- container image built by docker-ros
- link to package's source code documentation (doxygen)
- list of nodes, including
- file/description table of launch files
- example launch command including launch args
- implementation details: arbitrarily long description of how the package/node works internally; useful for someone working on the source code
- hardware requirements (amd/arm, CPU, GPU, RAM, ...) (CGE)
- safety requirements/guarantees: "ODD"-boundaries, fallback behavior, ..  (CGE)
- project acknowledgements

## Auto-generated Package Documentation

### `ros2_demo_package`

#### Launch Files

| Launch File | Description |
| --- | --- |
| [`ros2_demo_node_launch.py`](ros2_demo_package/launch/ros2_demo_node_launch.py) | |

#### `ros2_demo_node`

##### Subscribed Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/input` | `geometry_msgs/msg/PointStamped` | |

##### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/output` | `geometry_msgs/msg/PointStamped` | |

##### Actions

| Action | Type | Description |
| --- | --- | --- |
| `~/action` | `ros2_demo_package_interfaces/action/Fibonacci` | |

##### Parameters

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `param` | `float` | `1.0` | TODO |