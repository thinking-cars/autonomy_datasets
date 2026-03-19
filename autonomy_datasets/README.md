# Autonomy.Datasets

Integrates automated driving datasets into the ROS 2 ecosystem.

**Part of the [Autonomy.Hub](http://autonomyhub.de) Ecosystem**

- [Supported Datasets](#supported-datasets)
- [Container Images](#container-images)
- [autonomy_datasets](#autonomy_datasets)

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

AutonomyHub enables users to easily benchmark their automated driving building blocks across different tasks and datasets. It provides a unified framework for:

- **Dataset Integration**: Convert various automated driving datasets into ROS 2 messages with standardized interfaces
- **Performance Benchmarking**: Evaluate automated driving modules on multiple datasets with consistent metrics

## Supported Datasets

This repository supports various automated driving datasets including:
- [**nuScenes**](#nuscenes-dataset): Lidar + 3D Objects, Camera + 2D Objects, Camera + 3D Objects
- [**Waymo Open Dataset**](#waymo-open-dataset): Lidar + 3D Objects, Camera + 2D Objects, Camera + 3D Objects
- [**Thinking Cars Datasets**](#thinking-cars-dataset) available on request for **commercial use and custom data**
- [**Contributions**](#adding-a-new-dataset) adding more open datasets are very welcome

The following visualizations show sample data from different dataset configurations. These are automatically generated during CI testing.

### nuScenes Dataset

[![non-commercial](https://img.shields.io/badge/license-non--commercial-red)](https://www.nuscenes.org/terms-of-use)
[![nuScenes](https://img.shields.io/badge/origin-nuScenes-green)](https://www.nuscenes.org/nuscenes)

| Configuration & Splits | Sample Visualization | Features & Shape |
|--------------|--------|-------------|
| **lidar_objects**<br/>  - `train`: 28.130 Samples<br/>  - `valid`: 6.019 Samples<br/>**lidar_objects_mini**<br/>  - `train`: 25 Samples<br/>  - `valid`: 17 Samples | ![](TODO) | `point_cloud[N,5]`: Lidar point cloud with N points (x=front, y=left, z=up, intensity, timestamp)<br/><br/>`objects[M,8]`: List of 3D bounding boxes (class_id, x, y, z, yaw, length, width, height) in lidar frame |
| **2d_camera_objects**<br/>  - `train`: 28.130 Samples<br/>  - `valid`: 6.019 Samples<br/>**2d_camera_objects_mini**<br/>  - `train`: 25 Samples<br/>  - `valid`: 17 Samples | ![](TODO) | `image_front[900,1600,3]`: Front camera image with height=900px, width=1600px and RGB color channels<br/><br/>`objects[N,5]`: List of 2D bounding boxes (class_id, xmin, ymin, xmax, ymax) where (xmin, ymin) is the upper-left corner |
| **3d_camera_objects**<br/>  - `train`: 28.130 Samples<br/>  - `valid`: 6.019 Samples<br/>**3d_camera_objects_mini**<br/>  - `train`: 25 Samples<br/>  - `valid`: 17 Samples | ![](TODO) | `image_front[900,1600,3]`: Front camera image with height=900px, width=1600px and RGB color channels<br/><br/>`objects[M,8]`: List of 3D bounding boxes (class_id, x, y, z, yaw, length, width, height) in camera frame |

### Waymo Open Dataset

[![non-commercial](https://img.shields.io/badge/license-non--commercial-red)](https://waymo.com/open/terms)
[![Waymo Open Dataset](https://img.shields.io/badge/origin-Waymo_Open_Dataset-green)](https://waymo.com/open)

| Configuration & Splits | Sample | Features & Shape |
|--------------|--------|-------------|
| **lidar_objects**<br/>  - `train`: 158.081 Samples<br/>  - `valid`: 39.987 Samples<br/>**lidar_objects_mini**<br/>  - `train`: 20 Samples<br/>  - `valid`: 20 Samples | ![](TODO) | `point_cloud[N,5]`: Lidar point cloud with N points (x=forward, y=left, z=up, intensity, elongation)<br/><br/>`objects[M,8]`: List of 3D bounding boxes (class_id, x, y, z, yaw, length, width, height) in lidar frame |
| **2d_camera_objects**<br/>  - `train`: 158.081 Samples<br/>  - `valid`: 39.987 Samples<br/>**2d_camera_objects_mini**<br/>  - `train`: 20 Samples<br/>  - `valid`: 20 Samples | ![](TODO) | `image_front[1280,1920,3]`: Front camera image with height=1280px, width=1920px and RGB color channels<br/><br/>`objects[N,5]`: List of 2D bounding boxes (class_id, xmin, ymin, xmax, ymax) where (xmin, ymin) is the upper-left corner |
| **3d_camera_objects**<br/>  - `train`: 158.081 Samples<br/>  - `valid`: 39.987 Samples<br/>**3d_camera_objects_mini**<br/>  - `train`: 20 Samples<br/>  - `valid`: 20 Samples | ![](TODO) | `image_front[1280,1920,3]`: Front camera image with height=1280px, width=1920px and RGB color channels<br/><br/>`objects[M,8]`: List of 3D bounding boxes (class_id, x, y, z, yaw, length, width, height) in camera frame |

### Thinking Cars Dataset

[![non-commercial](https://img.shields.io/badge/license-non--commercial-red)](https://waymo.com/open/terms)
![commercial](https://img.shields.io/badge/license-commercial-green)
[![Thinking Cars](https://img.shields.io/badge/origin-Thinking_Cars-green)](https://thinking-cars.de/)

**Custom datasets** according to your needs and suitable for **commercial use** are available via an expanding network of partners [on request](mailto:info@thinking-cars.de), for example:

- Sensor data from (stereo) cameras, lidars, radars and IMU
- Object annotations
- V2X Data (e.g. [ETSI ITS Messages](https://forge.etsi.org/rep/ITS/asn1))
- Driving Trajectories and Scenarios

TODO: add some sample images

### Container Images

| Description | Image:Tag | Default Command |
| --- | --- | -- |
|  |  |  |


## `autonomy_datasets`

### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
|  |  |  |

### Parameters

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
|  |  |  |  |

## Dataset Usage

Download the original dataset and ensure the folder structure is correct as shown below.

- **Waymo Open Dataset:** [Download](https://waymo.com/open/)

    ```bash
    $DATASET_DIR/
        waymo_open_dataset/
            training/
                camera_box/
                    *.parquet
                    ...
                ...
            validation/
                camera_box/
                    *.parquet
                    ...
                ...
    ```

- **nuScenes Dataset:** [Download](https://www.nuscenes.org/nuscenes#download)

    ```bash
    $DATASET_DIR/
        nuscenes/
            basemap/
                *.png
            ...
            samples/
                CAM_BACK/
                    *.jpg
                ...
            sweeps/
                CAM_BACK/
                    *.jpg
                ...
            v1.0-mini/
                *.json
            v1.0-test/
                *.json
            v1.0-trainval/
                *.json
    ```

Run the `autonomy_datasets` ROS 2 node using the provided docker image. Make sure to mount your local `$DATASET_DIR` into the container:

```bash
TODO docker run command
```

## Dataset Visualization

TODO: update to ROS 2 nodes

We provide two powerful visualization tools:

### 1. Static Image Generation (`save_visualizations.py`)

Generate and save visualization images for batch processing and documentation as shown in the [Supported Datasets](#supported-datasets) section:

```bash
python3 save_visualizations.py \
  --dataset nuscenes/lidar_objects_mini/train \
  --start 0 \
  --stop 10
```

#### Visualization Types

**Lidar Objects (`lidar_objects`):**
- Bird's eye view showing point cloud and 3D bounding boxes
- Front view showing point cloud and projected bounding boxes
- Color-coded by object class

**2D Camera Objects (`2d_camera_objects`):**
- Camera image with 2D bounding boxes overlaid
- Color-coded by object class

**3D Camera Objects (`3d_camera_objects`):**
- Camera image
- 3D visualization showing 3D bounding boxes in camera frame
- Color-coded by object class

### 2. Interactive 3D Viewer (`view_lidar_3d.py`)

![3D Viewer Demo](./assets/lidar_3d_viewer.png)

Explore samples with lidar point clouds and 3D bounding boxes interactively using Open3D:

```bash
python3 view_lidar_3d.py \
  --dataset nuscenes/lidar_objects_mini/train \
  --index 0
```

**Interactive controls:**
- Left mouse: Rotate view
- Right mouse: Pan view
- Mouse wheel: Zoom
- Q/ESC: Close

### Adding a New Dataset

Extending the framework with new datasets is straightforward and allows you to benchmark your building blocks on additional data sources.

TODO

## Contributing

We welcome contributions to expand dataset support and improve the benchmarking framework! 

- Add new datasets following the guidelines above
- Report bugs and suggest features via GitHub issues

## License

This software framework is released under the [Apache License 2.0](../LICENSE).

**IMPORTANT**: The Apache License 2.0 License applies ONLY to the code in this repository (the dataset loading tools, preprocessing scripts, and integration framework). It does NOT apply to the actual datasets themselves.

Each dataset has its own license and terms of use:
- **nuScenes Dataset**: Licensed under [Creative Commons Attribution-NonCommercial-ShareAlike 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/) - See [nuScenes Terms of Use](https://www.nuscenes.org/terms-of-use)
- **Waymo Open Dataset**: Licensed under the [Waymo Dataset License Agreement](https://waymo.com/open/terms/)

You must comply with the license terms of any dataset you download and use.

## Citation

If you use this repository in your research, please cite:

```bibtex
@misc{autonomyhub,
    title = {Autonomy.Hub: Modular Automated Driving Development},
    year = {2025},
    url = {https://autonomyhub.de},
}
```

## Contact & Support

- **Documentation**: [Autonomy.Hub Documentation](https://autonomyhub.de)
- **Issues**: [GitHub Issues](https://github.com/thinking-cars/autonomy_datasets/issues)
- **Individual Support**: [Thinking Cars](mailto:info@thinking-cars.de)
