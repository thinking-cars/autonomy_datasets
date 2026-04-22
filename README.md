# openads_ros2_demo_repository

<p align="center">
  <a href="https://github.com/openads-project"><img src="https://img.shields.io/badge/OpenADS-ffff00"/></a>
  <a href="https://www.ros.org"><img src="https://img.shields.io/badge/ROS 2-jazzy-22314e"/></a>
  <a href="https://github.com/openads-project/openads_ros2_demo_repository/releases/latest"><img src="https://img.shields.io/github/v/release/openads-project/openads_ros2_demo_repository"/></a>
  <a href="https://github.com/openads-project/openads_ros2_demo_repository/blob/main/LICENSE"><img src="https://img.shields.io/github/license/openads-project/openads_ros2_demo_repository"/></a>
  <br>
  <a href="https://github.com/openads-project/openads_ros2_demo_repository/actions/workflows/docker-ros.yml"><img src="https://github.com/openads-project/openads_ros2_demo_repository/actions/workflows/docker-ros.yml/badge.svg"/></a>
  <a href="https://github.com/openads-project/openads_ros2_demo_repository/actions/workflows/industrial_ci.yml"><img src="https://github.com/openads-project/openads_ros2_demo_repository/actions/workflows/industrial_ci.yml/badge.svg"/></a>
  <a href="https://openads-project.github.io/openads_ros2_demo_repository"><img src="https://github.com/openads-project/openads_ros2_demo_repository/actions/workflows/docs.yml/badge.svg"/></a>
  <a href="https://github.com/openads-project/openads_ros2_demo_repository/actions/workflows/consistency.yml"><img src="https://github.com/openads-project/openads_ros2_demo_repository/actions/workflows/consistency.yml/badge.svg"/></a>
</p>

**Demo repository for an OpenADS module**

This repository serves as a demo for an OpenADS module, showcasing the structure and documentation style for OpenADS packages. It includes a simple ROS 2 node that subscribes to a topic, processes the data, and publishes the result. This is a short description of the repository and its purpose.

**🚀 [Quick Start](#-quick-start)** | **🧑‍💻 [Development](#-development)** | **📝 [Documentation](#-documentation)** | **🙏 [Acknowledgements](#-acknowledgements)**

> [!IMPORTANT]  
> This repository is part of [🚗 ***OpenADS***](https://github.com/openads-project), the *Open Automated Driving Stack*.


<img src="https://raw.githubusercontent.com/ika-rwth-aachen/etsi_its_messages/refs/heads/main/assets/teaser.gif" width=800>


## 🚀 Quick Start

1. Start a container of the pre-built runtime image.
    ```bash
    docker run --rm -it ghcr.io/openads-project/openads_ros2_demo_repository:latest bash
    ```
1. Inside the container, launch the pre-built nodes.
    ```bash
    ros2 launch ros2_demo_package ros2_demo_node_launch.py
    ```

## 🧑‍💻 Development

### Set up Development Environment

1. Clone the repository.
    ```bash
    git clone https://github.com/openads-project/openads_ros2_demo_repository.git
    ```
1. Initialize the [`.openads-dev-environment`](https://github.com/openads-project/openads-dev-environment) submodule containing development environment configuration.
    ```bash
    cd openads_ros2_demo_repository
    git submodule update --init --recursive
    ```
1. Open the repository in [Visual Studio Code](https://code.visualstudio.com).
    ```bash
    code .
    ```
1. Install the recommended VS Code extensions.  
    > *Ctrl+Shift+P / Extensions: Show Recommended Extensions / Install Workspace Recommended Extensions (Cloud Download Icon)*
1. Reopen the repository in a [Dev Container](https://code.visualstudio.com/docs/devcontainers/containers).
    > *Ctrl+Shift+P / Dev Containers: Rebuild and Reopen in Container*

### Build

> *Ctrl+Shift+B*

or

```bash
colcon build
```

### Run Tests

> *Ctrl+Shift+P / Tasks: Run Test Task*

or

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
colcon test
colcon test-result --verbose
```


## 📝 Documentation

- [Implementation Details](./docs/IMPLEMENTATION.md)
- [Source Code Documentation](https://openads-project.github.io/openads_ros2_demo_repository)
- Package Documentation
  - [ros2_demo_package](ros2_demo_package/README.md)
  - [ros2_demo_package_interfaces](ros2_demo_package_interfaces/README.md)


## 🙏 Acknowledgements

This work is accomplished within the projects TODO (FKZ TODO). We acknowledge the financial support by the 🇩🇪 Federal Ministry of Research, Technology and Space (BMFTR).
