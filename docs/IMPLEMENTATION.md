# Implementation Details

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

### NVIDIA PhysicalAI AV Dataset

[![commercial](https://img.shields.io/badge/license-commercial-green)](https://huggingface.co/datasets/nvidia/PhysicalAI-Autonomous-Vehicles)
[![Hugging Face](https://img.shields.io/badge/origin-Hugging_Face-green)](https://huggingface.co/datasets/nvidia/PhysicalAI-Autonomous-Vehicles)

![Rviz Screenshot NVIDIA PhysicalAI AV Dataset](./assets/rviz_nvidia_physicalai_av_dataset.png)

| Split | Sensors | Scenes | Samples |
| ------ | ------ | ------ | ---- |
| `camera` | 7 cameras at 30 Hz | 306.152 (20 seconds each) | 183.691.200 |
| `camera_lidar` | 7 cameras + 360 deg lidar at 10 Hz | 298.326 (20 seconds each) | 59.665.200 |
| `camera_radar` | 7 camera + up to 10 radars at 10 Hz | 160.761 (20 seconds each) | 32.152.200 |

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
docker run --rm -it --name autonomy_datasets --volume $DATASET_DIR:/datasets \
    --gpus=all --env=DISPLAY --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
    ghcr.io/thinking-cars/autonomy_datasets:latest
```

## Dataset Visualization

Use Rviz2 to visualize the data being published from the selected dataset:

```bash
docker exec -it autonomy_datasets ros2 launch autonomy_datasets autonomy_datasets.launch.py rviz:=only
```

### Adding a New Dataset

Extending the framework with new datasets is straightforward and allows you to benchmark your building blocks on additional data sources.

TODO
