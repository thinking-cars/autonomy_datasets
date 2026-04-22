# autonomy_datasets

<p align="center">
  <a href="https://github.com/thinking-cars"><img src="https://img.shields.io/badge/OpenADS-ffff00"/></a>
  <a href="https://github.com/thinking-cars/autonomy_datasets/releases/latest"><img src="https://img.shields.io/github/v/release/thinking-cars/autonomy_datasets"/></a>
  <a href="https://github.com/thinking-cars/autonomy_datasets/blob/main/LICENSE"><img src="https://img.shields.io/github/license/thinking-cars/autonomy_datasets"/></a>
  <a href="https://www.ros.org"><img src="https://img.shields.io/badge/ROS 2-jazzy-22314e"/></a>
  <br>
  <a href="https://github.com/thinking-cars/autonomy_datasets/actions/workflows/docker-ros.yml"><img src="https://github.com/thinking-cars/autonomy_datasets/actions/workflows/docker-ros.yml/badge.svg"/></a>
  <a href="https://github.com/thinking-cars/autonomy_datasets/actions/workflows/industrial_ci.yml"><img src="https://github.com/thinking-cars/autonomy_datasets/actions/workflows/industrial_ci.yml/badge.svg"/></a>
  <a href="https://thinking-cars.github.io/autonomy_datasets"><img src="https://github.com/thinking-cars/autonomy_datasets/actions/workflows/docs.yml/badge.svg"/></a>
  <a href="https://github.com/thinking-cars/autonomy_datasets/actions/workflows/consistency.yml"><img src="https://github.com/thinking-cars/autonomy_datasets/actions/workflows/consistency.yml/badge.svg"/></a>
</p>

**This repository is part of the [Autonomy.Hub](http://autonomyhub.de) Ecosystem**

AutonomyHub enables users to easily benchmark their automated driving building blocks across different tasks and datasets. It provides a unified framework for:

- **Dataset Integration**: Convert various automated driving datasets into ROS 2 messages with standardized interfaces
- **Performance Benchmarking**: Evaluate automated driving modules on multiple datasets with consistent metrics

## Key Features of this Repository

- 🔄 **Unified ROS 2 Interface**: Work with multiple datasets using the benefits of the ROS 2 ecosystem
- 📊 **Comprehensive Benchmarks**: Use the provided datasets with [Autonomy.Benchmarks](https://github.com/thinking-cars/autonomy_benchmarks) to benchmark building blocks across different automated driving tasks (object detection, tracking, segmentation, etc.)
- ⚡ **Optimized Pipeline**: Preprocessed Rosbag files ensure fast execution during development
- 🐳 **Dockerized Environment**: Reproducible setup with all dependencies included
- 🔌 **Modular Architecture**: Easy integration with other ROS 2 packages

## Supported Datasets

This repository supports various automated driving datasets including:
- [**nuScenes**](docs/IMPLEMENTATION.md#nuscenes-dataset): Lidar + 3D Objects, Camera + 2D Objects, Camera + 3D Objects
- [**Waymo Open Dataset**](docs/IMPLEMENTATION.md#waymo-open-dataset): Lidar + 3D Objects, Camera + 2D Objects, Camera + 3D Objects
- [**Thinking Cars Datasets**](docs/IMPLEMENTATION.md#thinking-cars-dataset) available on request for **commercial use and custom data**
- [**Contributions**](docs/IMPLEMENTATION.md#adding-a-new-dataset) adding more open datasets are very welcome

> **⚠️ IMPORTANT DATASET LICENSE DISCLAIMER**
> 
> This repository provides tools and interfaces for working with autonomous driving datasets. **The actual datasets (nuScenes, Waymo Open Dataset, etc.) are NOT included and must be obtained separately.**
>
> **Before using any dataset, you MUST:**
> - Register and accept the terms of use for each dataset you wish to use
> - Download the datasets from their official sources
> - Comply with all licensing terms and conditions of the respective dataset providers
>
> **Dataset-specific requirements:**
> - **nuScenes**: Register at [nuScenes.org](https://www.nuscenes.org/nuscenes) and agree to the [nuScenes Terms of Use](https://www.nuscenes.org/terms-of-use)
> - **Waymo Open Dataset**: Register at [Waymo Open Dataset](https://waymo.com/open) and agree to their [License Agreement](https://waymo.com/open/terms)
>
> The Apache-2.0 License of this repository applies ONLY to the code and tools provided here, NOT to the datasets themselves. Users are solely responsible for ensuring compliance with all dataset licenses.

**🚀 [Quick Start](#-quick-start)** | **🧑‍💻 [Development](#-development)** | **📝 [Documentation](#-documentation)** | **🙏 [Acknowledgements](#-acknowledgements)**

> [!IMPORTANT]  
> This repository is part of [🚗 ***OpenADS***](https://github.com/thinking-cars), the *Open Automated Driving Stack*.

**🚀 [Quick Start](#-quick-start)** | **🧑‍💻 [Development](#-development)** | **📝 [Documentation](#-documentation)** | **🙏 [Acknowledgements](#-acknowledgements)**


## 🚀 Quick Start

1. Start a container of the pre-built runtime image.
    ```bash
    docker run --rm -it ghcr.io/thinking-cars/autonomy_datasets:latest bash
    ```
1. Inside the container, launch the pre-built nodes.
    ```bash
    ros2 launch autonomy_datasets autonomy_datasets_launch.py
    ```

## 🧑‍💻 Development

### Set up Development Environment

1. Clone the repository.
    ```bash
    git clone https://github.com/thinking-cars/autonomy_datasets.git
    ```
1. Initialize the [`.openads-dev-environment`](https://github.com/thinking-cars/openads-dev-environment) submodule containing development environment configuration.
    ```bash
    cd autonomy_datasets
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
- [Source Code Documentation](https://thinking-cars.github.io/autonomy_datasets)
- Package Documentation
  - [autonomy_datasets](autonomy_datasets/README.md)


## 🙏 Acknowledgements

TODO: Project/funding acknowledgements
