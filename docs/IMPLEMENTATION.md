# Implementation Details

## Usage

The `autonomy_datasets` package is available in a pre-compiled Docker image. Start a container mounting your local dataset directory. Alternatively, use VS Code to open this repository in a Devcontainer.

> Follow the instructions in the [Supported Datasets](#supported-datasets) section to obtain the dataset.

```bash
DATASET_DIR="~/datasets"  # adapt this to your dataset location
docker run --rm -it --gpus all --env=DISPLAY --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw --volume $DATASET_DIR:/datasets ghcr.io/thinking-cars/autonomy_datasets:latest bash
```

Run the following command in the container to visualize samples from the *NVIDIA PhysicalAI AV Dataset*:

```bash
hf auth login  # login with your HuggingFace access token
ros2 launch autonomy_datasets autonomy_datasets.launch.py rviz:=yes
```

This will download all selected scenes sequentially, write samples into Rosbags at `$DATASET_DIR/nvidia_physicalai_av_dataset/bags` while visualizing samples in Rviz.

The following command will only write samples to rosbags without publishing as ROS messages:

```bash
ros2 launch autonomy_datasets autonomy_datasets.launch.py publish_samples:=false
```

## Supported Datasets

This repository supports various automated driving datasets including:
- [**nuScenes**](#nuscenes-dataset)
- [**Waymo Open Dataset**](#waymo-open-dataset)
- [**Thinking Cars Datasets**](#thinking-cars-dataset) available on request for **commercial use and custom data**
- [**Contributions**](#adding-a-new-dataset) adding more open datasets are welcome


### nuScenes Dataset

[![non-commercial](https://img.shields.io/badge/license-non--commercial-red)](https://www.nuscenes.org/terms-of-use)
[![nuScenes](https://img.shields.io/badge/origin-nuScenes-green)](https://www.nuscenes.org/nuscenes)

| Split | Samples |
| ------ | ------ |
| `training` | 28.130 |
| `validation` | 6.019 |
| `training_mini` | 25 |
| `validation_mini` | 17 |

| Source | Topic | Type | Description |
| ----- | ----- | ----- |---------- |
| Sensor: Lidar | `/autonomy_datasets/point_cloud` | `sensor_msgs/PointCloud2` | Raw sensor data from top lidar as point cloud with fields (`x`, `y`, `z`, ...). |
| Sensor: Front Camera | `/autonomy_datasets/camera/image_raw` | `sensor_msgs/Image` | Raw RGB images (height=900px, width=1600px) from front camera. |
| Annotation: 3D Objects | `/autonomy_datasets/object_list_3d` | `perception_msgs/ObjectList` | Annotated 3D objects in HEXAMOTION model. |

#### Usage

[Download](https://www.nuscenes.org/nuscenes#download) the dataset and ensure the following folder structure is correct:

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

Uncomment the `NuScenes` section in the [config file](../autonomy_datasets/config/params.yml).

Run `ros2 launch autonomy_datasets autonomy_datasets.launch.py rviz:=yes` to visualize the dataset.

### Waymo Open Dataset

[![non-commercial](https://img.shields.io/badge/license-non--commercial-red)](https://waymo.com/open/terms)
[![Waymo Open Dataset](https://img.shields.io/badge/origin-Waymo_Open_Dataset-green)](https://waymo.com/open)

![Rviz Screenshot Waymo Open Dataset](./assets/rviz_waymo_open_dataset.png)

| Split | Samples |
| ------ | ------ |
| `training` | 158.081 |
| `validation` | 39.987 |
| `training_mini` | ? |
| `validation_mini` | ? |

| Source | Topic | Type | Description |
| ----- | ----- | ----- |---------- |
| Sensor: Lidar | `/autonomy_datasets/point_cloud` | `sensor_msgs/PointCloud2` | Raw sensor data from top lidar as point cloud with fields (`x`, `y`, `z`, `intensity`, `elongation`) in `lidar_top` frame. |
| Annotation: 3D Lidar Objects | `/autonomy_datasets/object_list_3d` | `perception_msgs/ObjectList` | Annotated 3D objects (HEXAMOTION model) in `base_link` frame. Default: Only objects with min. 1 point in top lidar point cloud. |
| Sensor: Front Camera | `/autonomy_datasets/camera/image_raw` | `sensor_msgs/Image` | Raw RGB images (height=1280px, width=1920px) from front camera. |
| Annotation: 2D Camera Objects | `/autonomy_datasets/object_list_2d` | `perception_msgs/ObjectList` | Annotated 2D objects (CAMERA_2D model) in `cam_front` frame. |

#### Usage

[Download](https://waymo.com/open/) the dataset and ensure the following folder structure is correct:

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

Uncomment the `Waymo Open Dataset` section in the [config file](../autonomy_datasets/config/params.yml).

Run `ros2 launch autonomy_datasets autonomy_datasets.launch.py rviz:=yes` to visualize the dataset.

### NVIDIA PhysicalAI AV Dataset

[![commercial](https://img.shields.io/badge/license-commercial-green)](https://huggingface.co/datasets/nvidia/PhysicalAI-Autonomous-Vehicles)
[![Hugging Face](https://img.shields.io/badge/origin-Hugging_Face-green)](https://huggingface.co/datasets/nvidia/PhysicalAI-Autonomous-Vehicles)

![Rviz Screenshot NVIDIA PhysicalAI AV Dataset](./assets/rviz_nvidia_physicalai_av_dataset.png)

The number of samples depends on the configurable selected sensor modalities:

| Sensor Modalities | Sensor Setup | Samples |
| ------ | ------ | ---- |
| **Camera** | 7 cameras at 30 Hz | 306.152 (20 seconds each) | 183.691.200 |
| **Camera + Lidar** | 7 cameras + 360 deg lidar at 10 Hz | 298.326 (20 seconds each) | 59.665.200 |
| **Camera + Radar** | 7 camera + up to 10 radars at 10 Hz | 160.761 (20 seconds each) | 32.152.200 |
| **Camera + Lidar + Radar** | 7 camera + 360 deg lidar at 10 Hz + up to 10 radars at 10 Hz | TODO (20 seconds each) | TODO |

The provided **default splits** contain only samples including all sensor modalities (**Camera + Lidar + Radar**).

| Split | Scenes | Samples |
| ------ | ------ | ---- |
| `all_germany` | 7.247 (20 seconds each) | 1.449.400 |
| `train_germany` | 3.694 (20 seconds each) | 738.800 |
| `valid_germany` | 2.044 (20 seconds each) | 408.800 |
| `test_germany` | 1.509 (20 seconds each) | 301.800 |
| `train_*` | TODO (20 seconds each) | TODO |

#### Usage

Login using your [HuggingFace Token](https://huggingface.co/docs/hub/security-tokens) with `hf auth login` to access the dataset.

Uncomment the `NVIDIA PhysicalAI AV Dataset` section in the [config file](../autonomy_datasets/config/params.yml).

Run `ros2 launch autonomy_datasets autonomy_datasets.launch.py rviz:=yes` to visualize the dataset.

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

### Adding a new dataset

Create a new dataset adapter based on the existing files [here](../autonomy_datasets/autonomy_datasets/datasets/).

Add documentation for the new dataset to this README and add it to the table in the [top-level README](../README.md).

Create a Pull Request on GitHub and wait for maintainer's feedback.
