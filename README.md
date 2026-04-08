# autonomy_datasets

<p align="center">
  <a href="https://www.ros.org"><img src="https://img.shields.io/badge/ROS 2-jazzy-22314e"/></a>
  <a href="https://github.com/thinking-cars/autonomy_datasets/releases/latest"><img src="https://img.shields.io/github/v/release/thinking-cars/autonomy_datasets"/></a>
  <a href="https://github.com/thinking-cars/autonomy_datasets/blob/main/LICENSE"><img src="https://img.shields.io/github/license/thinking-cars/autonomy_datasets"/></a>
  <br>
  <a href="https://github.com/thinking-cars/autonomy_datasets/actions/workflows/docker-ros.yml"><img src="https://github.com/thinking-cars/autonomy_datasets/actions/workflows/docker-ros.yml/badge.svg"/></a>
  <a href="https://openads-project.github.io/autonomy_datasets"><img src="https://github.com/thinking-cars/autonomy_datasets/actions/workflows/docs.yml/badge.svg"/></a>
  <a href="https://github.com/thinking-cars/autonomy_datasets/actions/workflows/consistency.yml"><img src="https://github.com/thinking-cars/autonomy_datasets/actions/workflows/consistency.yml/badge.svg"/></a>
</p>

**This repository is part of the [Autonomy.Hub](http://autonomyhub.de) Ecosystem**

**Autonomy.Hub** enables users to easily benchmark their automated driving building blocks across different tasks and datasets:

- 🔄 **Unified ROS 2 Interface**: Work with multiple datasets using the benefits of the ROS 2 ecosystem
- 📊 **Comprehensive Benchmarks**: Use the provided datasets with [Autonomy.Benchmarks](https://github.com/thinking-cars/autonomy_benchmarks) to benchmark building blocks across different automated driving tasks (object detection, tracking, segmentation, etc.)
- ⚡ **Optimized Pipeline**: Preprocessed Rosbag files ensure fast execution during development
- 🐳 **Dockerized Environment**: Reproducible setup with all dependencies included
- 🔌 **Modular Architecture**: Easy integration with other ROS 2 packages

## Supported Datasets

This repository supports various automated driving datasets.

> [**Contributions**](docs/IMPLEMENTATION.md#adding-a-new-dataset) adding more datasets are welcome

| Dataset | License | Samples | Preview |
|-------- | -------- | ------- | ------ |
| [**Waymo Open Dataset**](docs/IMPLEMENTATION.md#waymo-open-dataset) | [![non-commercial](https://img.shields.io/badge/license-non--commercial-red)](https://waymo.com/open/terms) [![Waymo Open Dataset](https://img.shields.io/badge/origin-Waymo_Open_Dataset-green)](https://waymo.com/open) | 158.081 Training</br>39.987 Validation | ![Rviz Screenshot Waymo Open Dataset](./docs/assets/rviz_waymo_open_dataset.png) |
| WIP: [**nuScenes**](docs/IMPLEMENTATION.md#nuscenes-dataset) | [![non-commercial](https://img.shields.io/badge/license-non--commercial-red)](https://www.nuscenes.org/terms-of-use) [![nuScenes](https://img.shields.io/badge/origin-nuScenes-green)](https://www.nuscenes.org/nuscenes) | 28.130 Training</br>6.019 Validation |  |
| [**NVIDIA Physical AI AV Dataset (Alpamayo)**](docs/IMPLEMENTATION.md#nvidia-physicalai-av-dataset) | [![commercial](https://img.shields.io/badge/license-commercial-green)](https://huggingface.co/datasets/nvidia/PhysicalAI-Autonomous-Vehicles) [![Hugging Face](https://img.shields.io/badge/origin-Hugging_Face-green)](https://huggingface.co/datasets/nvidia/PhysicalAI-Autonomous-Vehicles) | 298.326 clips, each 20 seconds with lidar (10 Hz) and 7 cameras (30 Hz) | ![Rviz Screenshot PhysicalAI AV Dataset](./docs/assets/rviz_nvidia_physicalai_av_dataset.png) |
| [**Thinking Cars Datasets**](docs/IMPLEMENTATION.md#thinking-cars-dataset) | [![non-commercial](https://img.shields.io/badge/license-non--commercial-red)](https://waymo.com/open/terms) ![commercial](https://img.shields.io/badge/license-commercial-green) [![Thinking Cars](https://img.shields.io/badge/origin-Thinking_Cars-green)](https://thinking-cars.de/)
 |  |  |

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

<p align="center">
  <strong>🚀 <a href="#-quick-start">Quick Start</a></strong> • <strong>💻 <a href="#-development">Development</a></strong> • <strong>📝 <a href="#-documentation">Documentation</a></strong>
</p>

> [!IMPORTANT]
> This repository is part of [***OpenADS***](https://github.com/openads-project), the *Open Automated Driving Stack*.


**🚀 [Quick Start](#-quick-start)** | **🧑‍💻 [Development](#-development)** | **📝 [Documentation](#-documentation)** | **🙏 [Acknowledgements](#-acknowledgements)**


## 🚀 Quick Start

1. Start a container of the pre-built runtime image.
    ```bash
    docker run --rm -it ghcr.io/thinking-cars/autonomy_datasets:latest bash
    ```
1. Inside the container, launch the pre-built nodes.
    ```bash
    ros2 launch autonomy_datasets autonomy_datasets.launch.py
    ```

## 💻 Development

### Set up Development Environment

1. Clone the repository.
    ```bash
    git clone https://github.com/thinking-cars/autonomy_datasets.git
    ```
1. Initialize the [`.openads-dev-environment`](https://github.com/openads-project/openads-dev-environment) submodule containing development environment configuration.
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

```bash
colcon build
```

### Run Tests

> *Ctrl+Shift+P / Tasks: Run Test Task*

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
colcon test
colcon test-result --verbose
```


## 📝 Documentation

Package and node interfaces are documented in the respective package READMEs listed below. Implementation details are found in the [Source Code Documentation](https://openads-project.github.io/autonomy_datasets).

| Package | Description |
| --- | --- |
| [autonomy_datasets](autonomy_datasets/README.md) | Integrates automated driving datasets into the ROS 2 ecosystem |

## ⚖️ Licensing

The source code in this repository is licensed under Apache-2.0, see [LICENSE](LICENSE). Container images provided by this repository may contain third-party software shipped with their own license terms.

## 🙏 Acknowledgements

This project is maintained by [Thinking Cars](mailto:info@thinking-cars.de). We appreciate contributions and are happy to discuss potential collaborations.
