# Implementation Details


## Supported Datasets

This repository supports various automated driving datasets including:
- [**NVIDIA PhysicalAI AV Dataset**](#nvidia-physicalai-av-dataset)
- [**nuScenes**](#nuscenes-dataset)
- [**Waymo Open Dataset**](#waymo-open-dataset)
- [**Thinking Cars Datasets**](#thinking-cars-dataset) available on request for **commercial use and custom data**
- [**Contributions**](#adding-a-new-dataset) adding more open datasets are welcome


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

| Split | Country | Scenes | Samples |
| ----- | ------- | ------ | ---- |
| `all` | All | 85.082 | approx. 17.016.400 |
| `all` | Germany | 7.247 | approx. 1.449.400 |
| `train` | Germany | 3.694 | approx. 738.800 |
| `valid` | Germany | 2.044 | approx. 408.800 |
| `test` | Germany | 1.509 | approx. 301.800 |

| Source | Topic | Type | Description |
| ----- | ----- | ----- |---------- |
| **Sensor:** Top Lidar | `/lidar_01/point_cloud` | `sensor_msgs/msg/PointCloud2` | Raw sensor data from top lidar as point cloud with fields (`x`, `y`, `z`, `intensity`) in sensor frame. |
| **Sensor:** Front Tele Camera (30° FOV) | `/camera_01/image_raw`</br>`/camera_01/camera_info` | `sensor_msgs/msg/Image`</br>`sensor_msgs/msg/CameraInfo` | Raw RGB images (height=1080px, width=1920px) from front tele camera. |
| **Sensor:** Front Wide Camera (120° FOV) | `/camera_02/image_raw`</br>`/camera_02/camera_info` | `sensor_msgs/msg/Image`</br>`sensor_msgs/msg/CameraInfo` | Raw RGB images (height=1080px, width=1920px) from front wide camera. |
| **Sensor:** Left Cross Camera (120° FOV) | `/camera_03/image_raw`</br>`/camera_03/camera_info` | `sensor_msgs/msg/Image`</br>`sensor_msgs/msg/CameraInfo` | Raw RGB images (height=1080px, width=1920px) from left cross camera. |
| **Sensor:** Right Cross Camera (120° FOV) | `/camera_04/image_raw`</br>`/camera_04/camera_info` | `sensor_msgs/msg/Image`</br>`sensor_msgs/msg/CameraInfo` | Raw RGB images (height=1080px, width=1920px) from right cross camera. |
| **Sensor:** Rear-Left Camera (70° FOV) | `/camera_05/image_raw`</br>`/camera_05/camera_info` | `sensor_msgs/msg/Image`</br>`sensor_msgs/msg/CameraInfo` | Raw RGB images (height=1080px, width=1920px) from rear-left camera. |
| **Sensor:** Rear-Right Camera (70° FOV) | `/camera_06/image_raw`</br>`/camera_06/camera_info` | `sensor_msgs/msg/Image`</br>`sensor_msgs/msg/CameraInfo` | Raw RGB images (height=1080px, width=1920px) from rear-right camera. |
| **Sensor:** Rear Tele Camera (30° FOV) | `/camera_07/image_raw`</br>`/camera_07/camera_info` | `sensor_msgs/msg/Image`</br>`sensor_msgs/msg/CameraInfo` | Raw RGB images (height=1080px, width=1920px) from rear tele camera. |
| **EgoData** | `/ego_data` | `perception_msgs/msg/EgoData`| Ego-vehicle's dimensions and dynamics state in `map` frame. |
| **Annotation:** 3D Lidar Objects | `/object_list/lidar_01` | `perception_msgs/msg/ObjectList` | Annotated 3D objects (`HEXAMOTION` model) in vehicle frame. |
| **Transformations** | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Static transformations to all sensor frames and dynamic transformation from `map` to vehicle frame. |

#### Usage

Login using your [HuggingFace Token](https://huggingface.co/docs/hub/security-tokens) to access the dataset and run the ROS node to download and store the data to rosbags while visualizing it in Rviz.

```bash
hf auth login
ros2 launch autonomy_datasets autonomy_datasets.launch.py dataset:=nvidia_physicalai_av_dataset
```


### nuScenes Dataset

[![non-commercial](https://img.shields.io/badge/license-non--commercial-red)](https://www.nuscenes.org/terms-of-use)
[![nuScenes](https://img.shields.io/badge/origin-nuScenes-green)](https://www.nuscenes.org/nuscenes)

![Rviz Screenshot nuScenes Dataset](./assets/rviz_nuscenes.png)

| Split | Samples |
| ------ | ------ |
| `training` | 28.130 |
| `validation` | 6.019 |
| `training_mini` | 25 |
| `validation_mini` | 17 |

| Source | Topic | Type | Description |
| ----- | ----- | ----- |---------- |
| **Sensor:** Top Lidar (Velodyne HDL-32E) | `/lidar_01/point_cloud` | `sensor_msgs/msg/PointCloud2` | Raw sensor data from top lidar as point cloud with fields (`x`, `y`, `z`, `intensity`, `timestamp`). |
| **Sensor:** Front Camera (Basler acA1600-60gc) | `/camera_01/image_raw`</br>`/camera_01/camera_info` | `sensor_msgs/msg/Image`</br>`sensor_msgs/msg/CameraInfo` | Raw RGB images (height=900px, width=1600px) from front camera. |
| **Sensor:** Front-Right Camera (Basler acA1600-60gc) | `/camera_02/image_raw`</br>`/camera_02/camera_info` | `sensor_msgs/msg/Image`</br>`sensor_msgs/msg/CameraInfo` | Raw RGB images (height=900px, width=1600px) from front-right camera. |
| **Sensor:** Back-Right Camera (Basler acA1600-60gc) | `/camera_03/image_raw`</br>`/camera_03/camera_info` | `sensor_msgs/msg/Image`</br>`sensor_msgs/msg/CameraInfo` | Raw RGB images (height=900px, width=1600px) from back-right camera. |
| **Sensor:** Back Camera (Basler acA1600-60gc) | `/camera_04/image_raw`</br>`/camera_04/camera_info` | `sensor_msgs/msg/Image`</br>`sensor_msgs/msg/CameraInfo` | Raw RGB images (height=900px, width=1600px) from back camera. |
| **Sensor:** Back-Left Camera (Basler acA1600-60gc) | `/camera_05/image_raw`</br>`/camera_05/camera_info` | `sensor_msgs/msg/Image`</br>`sensor_msgs/msg/CameraInfo` | Raw RGB images (height=900px, width=1600px) from back-left camera. |
| **Sensor:** Front-Left Camera (Basler acA1600-60gc) | `/camera_06/image_raw`</br>`/camera_06/camera_info` | `sensor_msgs/msg/Image`</br>`sensor_msgs/msg/CameraInfo` | Raw RGB images (height=900px, width=1600px) from front-left camera. |
| **EgoData** | `/ego_data` | `perception_msgs/msg/EgoData`| Ego-vehicle's dimensions and dynamics state in `map` frame. |
| **Annotation:** 3D Lidar Objects | `/object_list/lidar_01` | `perception_msgs/msg/ObjectList` | Annotated 3D objects (`HEXAMOTION` model) visible in lidar scan. |
| **Annotation:** 3D Front Camera Objects | `/object_list/camera_01` | `perception_msgs/msg/ObjectList` | Annotated 3D objects (`HEXAMOTION` model) visible in front camera image. |
| **Transformations** | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Static transformations to all sensor frames and dynamic transformation from `map` to vehicle frame. |

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

Run the ROS node to convert and store the data to rosbags while visualizing it in Rviz.

```bash
ros2 launch autonomy_datasets autonomy_datasets.launch.py dataset:=nuscenes
```


### Waymo Open Dataset

[![non-commercial](https://img.shields.io/badge/license-non--commercial-red)](https://waymo.com/open/terms)
[![Waymo Open Dataset](https://img.shields.io/badge/origin-Waymo_Open_Dataset-green)](https://waymo.com/open)

![Rviz Screenshot Waymo Open Dataset](./assets/rviz_waymo_open_dataset.png)

| Split | Samples |
| ------ | ------ |
| `all` | 198.068 |
| `training` | 158.081 |
| `validation` | 39.987 |

| Source | Topic | Type | Description |
| ----- | ----- | ----- |---------- |
| **Sensor:** Top Lidar | `/lidar_01/point_cloud` | `sensor_msgs/msg/PointCloud2` | Raw sensor data from top lidar as point cloud with fields (`x`, `y`, `z`, `intensity`, `elongation`) in sensor frame. |
| **Sensor:** Front Camera | `/camera_01/image_raw`</br>`/camera_01/camera_info` | `sensor_msgs/msg/Image`</br>`sensor_msgs/msg/CameraInfo` | Raw RGB images (height=1280px, width=1920px) from front camera. |
| **Sensor:** Front-Left Camera | `/camera_02/image_raw`</br>`/camera_02/camera_info` | `sensor_msgs/msg/Image`</br>`sensor_msgs/msg/CameraInfo` | Raw RGB images (height=1280px, width=1920px) from front-left camera. |
| **Sensor:** Front-Right Camera | `/camera_03/image_raw`</br>`/camera_03/camera_info` | `sensor_msgs/msg/Image`</br>`sensor_msgs/msg/CameraInfo` | Raw RGB images (height=1280px, width=1920px) from front-right camera. |
| **Sensor:** Side-Left Camera | `/camera_04/image_raw`</br>`/camera_04/camera_info` | `sensor_msgs/msg/Image`</br>`sensor_msgs/msg/CameraInfo` | Raw RGB images (height=886px, width=1920px) from side-left camera. |
| **Sensor:** Side-Right Camera | `/camera_05/image_raw`</br>`/camera_05/camera_info` | `sensor_msgs/msg/Image`</br>`sensor_msgs/msg/CameraInfo` | Raw RGB images (height=886px, width=1920px) from side-right camera. |
| **EgoData** | `/ego_data` | `perception_msgs/msg/EgoData`| Ego-vehicle's dimensions and dynamics state in `map` frame. |
| **Annotation:** 3D Lidar Objects | `/object_list/lidar_01` | `perception_msgs/msg/ObjectList` | Annotated 3D objects (`HEXAMOTION` model) in vehicle frame. *Default: Only objects with min. 1 point in top lidar point cloud.* |
| **Annotation:** 2D Camera Objects | `/object_list/cameras` | `perception_msgs/msg/ObjectList` | Annotated 2D objects (`CAMERA2D` model) in camera frame. *Note: Currently no visualization is shown for this data type in RViz.* |
| **Transformations** | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Static transformations to all sensor frames and dynamic transformation from `map` to vehicle frame. |

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

Run the ROS node to convert and store the data to rosbags while visualizing it in Rviz.

```bash
ros2 launch autonomy_datasets autonomy_datasets.launch.py dataset:=waymo_open_dataset
```


### Thinking Cars Dataset

![commercial](https://img.shields.io/badge/license-commercial-green)
[![Thinking Cars](https://img.shields.io/badge/origin-Thinking_Cars-green)](https://thinking-cars.de/)

**Custom datasets** according to your needs and suitable for **commercial use** are available via an expanding network of partners [on request](mailto:info@thinking-cars.de), for example:

- Sensor data from (stereo) cameras, lidars, radars and IMU
- Object annotations
- V2X Data (e.g. [ETSI ITS Messages](https://forge.etsi.org/rep/ITS/asn1))
- Driving Trajectories and Scenarios

### Adding a new dataset

1. Create a new dataset adapter based on the existing files [here](../autonomy_datasets/autonomy_datasets/datasets/).
2. Add documentation for the new dataset to this README and add it to the table in the [top-level README](../README.md).
3. Create a [Pull Request](https://github.com/thinking-cars/autonomy_datasets/pulls) on GitHub and wait for maintainer's feedback.
