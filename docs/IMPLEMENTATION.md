# Implementation Details


## Supported Datasets

This repository supports various automated driving datasets including:
- [**Waymo Open Dataset**](#waymo-open-dataset)
- [**nuScenes**](#nuscenes-dataset)
- [**MAN TruckScenes**](#man-truckscenes-dataset)
- [**NVIDIA PhysicalAI AV Dataset**](#nvidia-physicalai-av-dataset)
- [**DrivIng**](#driving-dataset)
- [**TUM Traffic**](#tum-traffic-dataset)
- [**Zenseact Open Dataset**](#zenseact-open-dataset)
- [**Thinking Cars Datasets**](#thinking-cars-dataset) available on request for **commercial use and custom data**
- [**Contributions**](#adding-a-new-dataset) adding more open datasets are welcome


### Waymo Open Dataset

[![Waymo](https://img.shields.io/badge/license-Waymo-orange?style=for-the-badge)](https://waymo.com/open/terms)
[![Waymo Open Dataset](https://img.shields.io/badge/origin-Waymo_Open_Dataset-green?style=for-the-badge)](https://waymo.com/open)

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
| **Meta Information:** Object Annotations | `/object_list/lidar_01/meta_info`</br>`/object_list/camera_01/meta_info`</br>`/object_list/camera_all/meta_info` | `autonomy_datasets_msgs/msg/ObjectListMetaInfo` | Annotations without a representation in `perception_msgs/msg/Object`: `original_class`, `num_lidar_pts` and `difficulty_level`. Associated with the object list via the header stamp and the object id. |
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


### nuScenes Dataset

[![nuScenes](https://img.shields.io/badge/license-nuScenes-orange?style=for-the-badge)](https://www.nuscenes.org/terms-of-use)
[![nuScenes](https://img.shields.io/badge/origin-nuScenes-green?style=for-the-badge)](https://www.nuscenes.org/nuscenes)

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
| **Sensor:** Front Radar (Continental ARS 408-21) | `/radar_01/point_cloud` | `sensor_msgs/msg/PointCloud2` | Radar detections from front radar as point cloud with fields (`x`, `y`, `z`, `radial_velocity`, `rcs`). |
| **Sensor:** Front-Right Radar (Continental ARS 408-21) | `/radar_02/point_cloud` | `sensor_msgs/msg/PointCloud2` | Radar detections from front-right radar as point cloud with fields (`x`, `y`, `z`, `radial_velocity`, `rcs`). |
| **Sensor:** Back-Right Radar (Continental ARS 408-21) | `/radar_03/point_cloud` | `sensor_msgs/msg/PointCloud2` | Radar detections from back-right radar as point cloud with fields (`x`, `y`, `z`, `radial_velocity`, `rcs`). |
| **Sensor:** Back-Left Radar (Continental ARS 408-21) | `/radar_04/point_cloud` | `sensor_msgs/msg/PointCloud2` | Radar detections from back-left radar as point cloud with fields (`x`, `y`, `z`, `radial_velocity`, `rcs`). |
| **Sensor:** Front-Left Radar (Continental ARS 408-21) | `/radar_05/point_cloud` | `sensor_msgs/msg/PointCloud2` | Radar detections from front-left radar as point cloud with fields (`x`, `y`, `z`, `radial_velocity`, `rcs`). |
| **EgoData** | `/ego_data` | `perception_msgs/msg/EgoData`| Ego-vehicle's dimensions and dynamics state (`EGO` model) in `map` frame. Pose from the dataset's ego poses, velocity, acceleration, yaw rate, steering angle, standstill flag, turn indicator and brake light from the CAN bus expansion. |
| **Annotation:** 3D Lidar Objects | `/object_list/lidar_01` | `perception_msgs/msg/ObjectList` | Annotated 3D objects (`HEXAMOTION` model) visible in lidar scan. As nuScenes stores no object dynamics, the absolute velocity, acceleration and yaw rate are differentiated from the annotation positions and yaw angles of the neighboring keyframes. |
| **Annotation:** 3D Front Camera Objects | `/object_list/camera_01` | `perception_msgs/msg/ObjectList` | Annotated 3D objects (`HEXAMOTION` model) visible in front camera image. As nuScenes stores no object dynamics, the absolute velocity, acceleration and yaw rate are differentiated from the annotation positions and yaw angles of the neighboring keyframes. |
| **Transformations** | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Static transformations to all sensor frames and dynamic transformation from `map` to vehicle frame. |
| **Detection:** Detected 3D Objects | `/object_list/detected` | `perception_msgs/msg/ObjectList` | Detected 3D objects (`HEXAMOTION` model) from [Megvii](https://www.nuscenes.org/data/detection-megvii.zip) baseline. |
| **Meta Information:** Object Annotations | `/object_list/lidar_01/meta_info`</br>`/object_list/camera_01/meta_info`</br>`/object_list/detected/meta_info` | `autonomy_datasets_msgs/msg/ObjectListMetaInfo` | Annotations without a representation in `perception_msgs/msg/Object`: `original_class`, `num_lidar_pts`, `num_radar_pts`, `num_points`, `attribute` and `detection_score`. Associated with the object list via the header stamp and the object id. |
| **Map** | `map_contents` (parameter) | `string` | Lanelet2 map (OSM XML) of the current scene's location, converted from the nuScenes [map expansion](https://github.com/nutonomy/nuscenes-devkit/blob/master/docs/schema_nuscenes.md). Updated on every scene change, analogous to [`lanelet2_map_server`](https://github.com/openads-project/lanelet2_map_server). |

The Lanelet2 conversion is controlled via the `publish_lanelet2_map` (enable/disable) and `nuscenes_lanelet2_lane_width` (assumed lane width in meters) parameters. Lanes and lane connectors are converted to `road` lanelets (boundaries synthesized by offsetting the centerline by half the lane width), and pedestrian crossings to `crosswalk` lanelets. Conversion requires the nuScenes map-expansion data under `maps/expansion/`.

The converted map is stored next to the rosbag data of the scene it belongs to: the map as `map.osm` and its origin as `map.yaml` inside the scene's rosbag directory. Replaying a rosbag restores the map from there instead of converting it again, unless `publish_lanelet2_map` is disabled.

#### Usage

[Download](https://www.nuscenes.org/nuscenes#download) the dataset (including CAN Bus and Map Expansion) and ensure the following folder structure is correct:

```bash
$DATASET_DIR/
    nuscenes/
        can_bus/
        detection-megvii/  # optional
            megvii_*.json
        maps/
            basemap/
                *.png
            expansion/
                *.json
            prediciton/
                prediction_scenes.json
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


### MAN TruckScenes Dataset

[![CC BY-NC-SA](https://img.shields.io/badge/license-CC_BY--NC--SA-orange?style=for-the-badge)](https://creativecommons.org/licenses/by-nc-sa/4.0/)
[![AWS Open Data](https://img.shields.io/badge/origin-AWS_Open_Data-green?style=for-the-badge)](https://registry.opendata.aws/man-truckscenes/)

![Rviz Screenshot MAN TruckScenes Dataset](./assets/rviz_truckscenes.png)

[MAN TruckScenes](https://www.man.eu/truckscenes) is a public dataset recorded from a heavy truck. It comprises 747 scenes of 20 seconds each, annotated at 2 Hz, recorded with 6 lidars, 6 radars, 4 cameras and a high-precision GNSS. The dataset reuses the nuScenes database schema and is licensed under [CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/).

| Split | Scenes | Samples |
| ----- | ------ | ------- |
| `train` | 523 | approx. 20.900 |
| `val` | 75 | approx. 3.000 |
| `test` | 149 | approx. 5.900 |
| `mini_train` | 8 | approx. 320 |
| `mini_val` | 2 | approx. 80 |

> The `test` split is released without object annotations, so the object list topics are published empty for it.

| Source | Topic | Type | Description |
| ----- | ----- | ----- |---------- |
| **Sensor:** 6 Lidars | `/lidar_01/point_cloud` ... `/lidar_06/point_cloud` | `sensor_msgs/msg/PointCloud2` | Point clouds in the respective sensor frame with float32 fields (`x`, `y`, `z`, `intensity`) and a float64 absolute-seconds `timestamp`, preserving native per-point timing. Topic order is left, right, top-front, top-left, top-right, and rear. |
| **Sensor:** 6 Radars | `/radar_01/point_cloud` ... `/radar_06/point_cloud` | `sensor_msgs/msg/PointCloud2` | Radar detections in the respective sensor frame with float32 fields (`x`, `y`, `z`, `vrel_x`, `vrel_y`, `vrel_z`, `rcs`). Topic order is left-front, right-front, right-side, right-back, left-back, and left-side. |
| **Sensor:** 4 Cameras | `/camera_01/image_raw` ... `/camera_04/image_raw`</br>`/camera_01/camera_info` ... `/camera_04/camera_info` | `sensor_msgs/msg/Image`</br>`sensor_msgs/msg/CameraInfo` | Undistorted and rectified RGB images with native calibration; topic order is left-front, right-front, right-back, and left-back. |
| **EgoData** | `/ego_data` | `perception_msgs/msg/EgoData` | Ego-vehicle's dimensions and dynamics state in the UTM-WGS84 (zone U32) `map` frame, with velocities, accelerations, and yaw rate taken from the native `ego_motion_chassis` table. |
| **Annotation:** 3D Lidar Objects | `/object_list/lidar_01` | `perception_msgs/msg/ObjectList` | Annotated 3D objects (`HEXAMOTION` model) in the left lidar frame. *Default: Only objects with min. 1 point in the lidar point cloud.* |
| **Annotation:** 3D Camera Objects | `/object_list/camera_01` | `perception_msgs/msg/ObjectList` | Annotated 3D objects (`HEXAMOTION` model) visible in the left front camera image. |
| **Meta Information:** Object Annotations | `/object_list/lidar_01/meta_info`</br>`/object_list/camera_01/meta_info` | `autonomy_datasets_msgs/msg/ObjectListMetaInfo` | Annotations without a representation in `perception_msgs/msg/Object`: `original_class`, `num_lidar_pts`, `num_radar_pts`, `num_points` and `attribute`. Associated with the object list via the header stamp and the object id. |
| **Transformations** | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Static transformations to all sensor frames and dynamic transformation from `map` to vehicle frame. |

#### Usage

Select the split using `dataset_split` in `params_truckscenes.yml`. Missing data is downloaded automatically from the [AWS Open Data registry](https://registry.opendata.aws/man-truckscenes/) without requiring credentials; alternatively [download](https://www.man.eu/truckscenes) the archives manually and unpack them into the following folder structure:

```bash
$DATASET_DIR/
    truckscenes/
        samples/
            CAMERA_LEFT_FRONT/
                *.jpg
            LIDAR_LEFT/
                *.pcd
            RADAR_LEFT_FRONT/
                *.pcd
            ...
        v1.2-mini/
            *.json
        v1.2-test/
            *.json
        v1.2-trainval/
            *.json
```

> Only keyframe sensor data (`samples/`) is downloaded, because the adapter publishes annotated keyframes only. The unannotated `sweeps/` are skipped to keep the required disk space low.

Run the ROS node to download, convert, and store the data to rosbags while visualizing it in Rviz.

```bash
ros2 launch autonomy_datasets autonomy_datasets.launch.py dataset:=truckscenes
```


### NVIDIA PhysicalAI AV Dataset

[![NVIDIA License](https://img.shields.io/badge/license-NVIDIA-orange?style=for-the-badge)](https://huggingface.co/datasets/nvidia/PhysicalAI-Autonomous-Vehicles)
[![Hugging Face](https://img.shields.io/badge/origin-Hugging_Face-green?style=for-the-badge)](https://huggingface.co/datasets/nvidia/PhysicalAI-Autonomous-Vehicles)

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
| **Meta Information:** Object Annotations | `/object_list/lidar_01/meta_info` | `autonomy_datasets_msgs/msg/ObjectListMetaInfo` | Annotations without a representation in `perception_msgs/msg/Object`: `original_class`. Associated with the object list via the header stamp and the object id. |
| **Transformations** | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Static transformations to all sensor frames and dynamic transformation from `map` to vehicle frame. |

#### Usage

Login using your [HuggingFace Token](https://huggingface.co/docs/hub/security-tokens) to access the dataset and run the ROS node to download and store the data to rosbags while visualizing it in Rviz.

```bash
hf auth login
ros2 launch autonomy_datasets autonomy_datasets.launch.py dataset:=nvidia_physicalai_av_dataset
```


### DrivIng Dataset

[![CC BY-NC-ND](https://img.shields.io/badge/license-CC_BY--NC--ND-orange?style=for-the-badge)](https://creativecommons.org/licenses/by-nc-nd/4.0/)
[![Harvard Dataverse](https://img.shields.io/badge/origin-Harvard_Dataverse-green?style=for-the-badge)](https://doi.org/10.7910/DVN/VBZKDY)

![Rviz Screenshot DrivIng Dataset](./assets/rviz_driving.png)

[DrivIng](https://github.com/cvims/DrivIng) is a multimodal driving dataset recorded in Ingolstadt, Germany. The native data comprises the `day`, `dusk`, and `night` sequences, each synchronized at 10 Hz with a middle lidar, six vehicle cameras, vehicle state, calibration, and 3D track annotations. The dataset is licensed under [CC BY-NC-ND 4.0](https://creativecommons.org/licenses/by-nc-nd/4.0/).

| Split | Sequences |
| ----- | --------- |
| `all` | `night`, `day`, `dusk` |
| `day` | `day` |
| `dusk` | `dusk` |
| `night` | `night` |

| Source | Topic | Type | Description |
| ----- | ----- | ---- | ----------- |
| **Sensor:** Middle Lidar | `/lidar_01/point_cloud` | `sensor_msgs/msg/PointCloud2` | Point cloud in the middle-lidar frame with float32 fields (`x`, `y`, `z`, `intensity`) and a float64 absolute-seconds `timestamp`, preserving native per-point timing. |
| **Sensor:** Six Vehicle Cameras | `/camera_01/image_raw` ... `/camera_06/image_raw`</br>`/camera_01/camera_info` ... `/camera_06/camera_info` | `sensor_msgs/msg/Image`</br>`sensor_msgs/msg/CameraInfo` | RGB images and native calibration; topic order is front-left, front-right, left, right, back-left, and back-right. |
| **EgoData** | `/ego_data` | `perception_msgs/msg/EgoData` | Ego-vehicle pose in a local ENU `map` frame, derived from native relative north/east positions, north-referenced yaw, and the calibrated ADMA-to-vehicle lever arm. Velocity and standstill flag are differentiated from consecutive poses, as the native vehicle state holds no velocity. |
| **Annotation:** 3D Lidar Objects | `/object_list/lidar_01` | `perception_msgs/msg/ObjectList` | Track annotations as 3D objects in the middle-lidar frame. |
| **Meta Information:** Object Annotations | `/object_list/lidar_01/meta_info` | `autonomy_datasets_msgs/msg/ObjectListMetaInfo` | Annotations without a representation in `perception_msgs/msg/Object`: `original_class`. Associated with the object list via the header stamp and the object id. |
| **Transformations** | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Dynamic `map` to `base_link` pose plus calibrated static transforms to all sensors. |

#### Usage

Select `day`, `dusk`, `night`, or `all` using `dataset_split` in `params_driving.yml`. Missing data is downloaded automatically from [Harvard Dataverse](https://doi.org/10.7910/DVN/VBZKDY) and stored using the following folder structure:

```bash
$DATASET_DIR/
    driving/
        day/
            annotations.json
            calibration.json
            timesync_info.csv
            middle_lidar/
            front_left_camera/
            ...
        dusk/
        night/
```

Run the ROS node to download, convert, and store the data to rosbags while visualizing it in Rviz.

```bash
ros2 launch autonomy_datasets autonomy_datasets.launch.py dataset:=driving
```

### TUM Traffic Dataset

[![CC BY-NC-ND](https://img.shields.io/badge/license-CC_BY--NC--ND-orange?style=for-the-badge)](https://creativecommons.org/licenses/by-nc-nd/4.0/)
[![TUM Traffic Dataset](https://img.shields.io/badge/origin-TUM_Traffic_Dataset-green?style=for-the-badge)](https://innovation-mobility.com/en/project-providentia/a9-dataset/)

![Rviz Screenshot TUM Traffic Dataset](./assets/rviz_tum_traffic.png)

The [TUM Traffic Dataset](https://innovation-mobility.com/en/project-providentia/a9-dataset/) (`TUMTraf`) is recorded by roadside sensors mounted on the gantry bridges of the [Providentia++](https://innovation-mobility.com/en/project-providentia/) test field along the A9 motorway and the S110 intersection near Munich, Germany. It is an **infrastructure dataset without an ego vehicle**, so no `/ego_data` is published; the sensor station is published as a static `base_link`. The dataset is licensed under [CC BY-NC-ND 4.0](https://creativecommons.org/licenses/by-nc-nd/4.0/).

The dataset is released as one archive per release and subset. All releases share a common file layout but differ in their sensors, directory names and label formats, so the adapter discovers the recordings, sensors and frame timestamps from the file names instead of hard-coding each release:

| Release | Subsets | Sensors | Annotations |
| ------- | ------- | ------- | ----------- |
| `R00` TUMTraf A9 Highway (image subsets) | `r00_s00` ... `r00_s02` | 4 A9 gantry cameras (`s040`, `s050`) | 3D box corners projected into the image — no object list published, see below |
| `R00` TUMTraf A9 Highway (lidar subsets) | `r00_s03`, `r00_s04` | Roadside lidars | Native pre-OpenLABEL 3D cuboids (yaw-only orientation, no persistent track IDs) |
| `R01` TUMTraf A9 Highway Extended | `r01_s01` ... `r01_s04` | 4 A9 gantry cameras (`s040`, `s050`) | 3D box corners projected into the image — no object list published, see below |
| `R02` TUMTraf Intersection | `r02_s01` ... `r02_s04` | 2 S110 cameras, 2 S110 Ouster lidars | OpenLABEL 3D cuboids with track IDs |

> **Only `R00` to `R02` of the TUM Traffic Dataset, which contains `R00` to `R05`, are supported.**

> **No object list for the `R00` image subsets and `R01`:** These releases annotate a 3D box only as its 8 corners projected into the 2D image (`box3d_projected`), without releasing the 3D pose (position, dimensions, orientation) that produced the projection. The dataset adapter does not attempt to recover that 3D pose. These recordings still publish their raw camera images, calibration and transforms. Only recordings with real 3D cuboids (the `R00` lidar subsets, native pre-OpenLABEL format, and `R02`, OpenLABEL) publish `/object_list/lidar_01`.
>
> The `R00` lidar subsets ship no calibration source at all, so `base_link` is aliased to their single lidar's frame with an identity transform rather than leaving `/tf_static` unresolved.

Sensors are mapped onto the canonical topics in a fixed order, so `camera_01` and `lidar_01` are the sensors the object list is annotated in. The example below lists the topics of the intersection subsets (`R02`):

| Source | Topic | Type | Description |
| ----- | ----- | ----- | ---------- |
| **Sensor:** South Lidar (Ouster OS1-64) | `/lidar_01/point_cloud` | `sensor_msgs/msg/PointCloud2` | Point cloud in the sensor frame with float32 fields (`x`, `y`, `z`, `intensity`) and a float64 absolute-seconds `timestamp`, preserving the native per-point timing. |
| **Sensor:** North Lidar (Ouster OS1-64) | `/lidar_02/point_cloud` | `sensor_msgs/msg/PointCloud2` | Point cloud in the sensor frame, fields as above. |
| **Sensor:** South1 Camera (Basler 8mm) | `/camera_01/image_raw`</br>`/camera_01/camera_info` | `sensor_msgs/msg/Image`</br>`sensor_msgs/msg/CameraInfo` | RGB images (height=1200px, width=1920px) with the native calibration. |
| **Sensor:** South2 Camera (Basler 8mm) | `/camera_02/image_raw`</br>`/camera_02/camera_info` | `sensor_msgs/msg/Image`</br>`sensor_msgs/msg/CameraInfo` | RGB images (height=1200px, width=1920px) with the native calibration. |
| **Annotation:** 3D Lidar Objects | `/object_list/lidar_01` | `perception_msgs/msg/ObjectList` | Annotated 3D objects (`HEXAMOTION` model) in the lidar_01 frame, with the track UUID, the native class and the native attributes (occlusion level, body color, number of points) in `meta_info`. |
| **Transformations** | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Static transformations from the sensor station (`base_link`) to all sensor frames, and the station's static pose in the `map` frame. |

> **The object list is only geometrically accurate against `lidar_01`:** On releases with more than one lidar (`R02` and newer), the objects are annotated directly in the reference lidar's own frame (`lidar_01`) and republished as-is; they are not re-derived per sensor. Overlaying `/object_list/lidar_01` onto `/lidar_02/point_cloud` (or any other non-reference lidar) will show a visible offset, because the dataset's own released extrinsic calibration between its lidars is imprecise (e.g. for `s110_lidar_ouster_south`/`s110_lidar_ouster_north` in `R02`), which is why the dev kit ships a dedicated `src/registration/point_cloud_registration.py` to refine this pairing via ICP. This adapter does not run that registration step. So, only `lidar_01` is aligned with the published objects.
>
> **Some tracked objects visibly float above or sink into the ground:** On `R02` and newer, a track's `z` and `height` are often set once and held constant for its whole lifetime while only `x`/`y`/`yaw` keep updating — confirmed directly in the raw label files, where most multi-frame tracks in a sample recording had byte-identical `z`/`height` despite moving tens of meters. This can leave a track that started well aligned drifting out of alignment later (e.g. over a stretch with different road elevation), or leave it wrong for its entire length if the frozen value was never accurate to begin with (e.g. estimated from only a handful of lidar points at long range and never revisited, even once the object is later observed with far denser support). This is a property of the dataset's own annotations, not of this adapter: `cuboid.val` is passed through per frame unmodified.

The sensors are triggered independently and the dataset ships no synchronization table, so each sample is built from the frames closest in time to the reference sensor (`lidar_01`, or `camera_01` for the camera-only releases). Frames without a match within `tum_traffic_sync_tolerance_seconds` are skipped. Long recordings are split into rosbag scenes of `tum_traffic_rosbag_duration_seconds`.

All calibration is read from the dataset itself: from the `_calibration` directory of a recording if it ships one (`R00`/`R01`), otherwise from the `coordinate_systems` and `streams` sections of its OpenLABEL label files (`R02` and newer).

#### Usage

The dataset **cannot be downloaded automatically**. [Register](https://a9-dataset.innovation-mobility.com/en/register), accept the license, and [download](https://a9-dataset.innovation-mobility.com/downloads) the archives of the releases you want to use. Place the downloaded ZIP archives in the dataset directory; they are extracted on the first run into a directory named after the archive:

```bash
$DATASET_DIR/
    tum_traffic/
        a9_dataset_r02_s04.zip     # placed here manually, extracted on the first run
        a9_dataset_r02_s04/
            images/
                s110_camera_basler_south1_8mm/
                    *.jpg
                ...
            point_clouds/
                s110_lidar_ouster_south/
                    *.pcd
                ...
            labels_point_clouds/
                s110_lidar_ouster_south/
                    *.json
                ...
        a9_dataset_r00_s00/        # or extracted manually
            _images/
            _labels/
            _calibration/
```

Select the recordings to publish using `dataset_split` in `params_tum_traffic.yml`: `all` publishes every recording found in the dataset directory, any other value selects the recordings whose path contains it, e.g. a release (`r02`), a subset (`r02_s04`) or a split directory of a release (`train`). Because the releases ship different sensors, prefer a release-specific split; recordings of a mixed split publish their missing sensors as empty messages.

Run the ROS node to convert and store the data to rosbags while visualizing it in Rviz.

```bash
ros2 launch autonomy_datasets autonomy_datasets.launch.py dataset:=tum_traffic
```

### Zenseact Open Dataset

[![CC BY-SA](https://img.shields.io/badge/license-CC_BY--SA-orange?style=for-the-badge)](https://creativecommons.org/licenses/by-sa/4.0/)
[![Zenseact Open Dataset](https://img.shields.io/badge/origin-Zenseact_Open_Dataset-green?style=for-the-badge)](https://zod.zenseact.com)

*Rviz screenshot pending: `docs/assets/rviz_zenseact_open_dataset.png`*

The [Zenseact Open Dataset](https://zod.zenseact.com) (`ZOD`) is a multimodal driving dataset recorded by [Zenseact](https://zenseact.com) over two years in 14 European countries, covering a geographical area nine times larger than comparable datasets. Its sensor suite is a single forward-looking 8 MP fisheye camera, three roof-mounted Velodyne lidars (one VLS128 and two VLP16) merged into one point cloud per scan, and an OxTS RT3000 GNSS/IMU. It is the only large-scale automated driving dataset released under a permissive license ([CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/)), which allows both research and commercial use.

ZOD is published as three sub-datasets, which are selected together with the version and the split through `dataset_split` in the form `<subset>_<version>_<split>`:

| Sub-dataset | Content | Annotations |
| ----------- | ------- | ----------- |
| `frames` | 100.000 independent keyframes from all over Europe, each with one camera image, one second of surrounding lidar scans in either direction and GNSS/IMU data | Fully annotated |
| `sequences` | 1.473 clips of 20 seconds with the complete sensor suite at 10 Hz | Keyframe (middle frame) only |
| `drives` | 29 clips of a few minutes with the complete sensor suite at 10 Hz | Not annotated |

| Split | Scenes | Samples |
| ----- | ------ | ------- |
| `frames_full_<train\|val\|all>` | 100.000 frames | 1 per frame |
| `sequences_full_<train\|val\|all>` | 1.473 sequences of 20 seconds | approx. 200 per sequence |
| `drives_full_<train\|val\|all>` | 29 drives of a few minutes | approx. 10 per second |
| `frames_mini_all` | 12 frames (10 train, 2 validation) | 12 |
| `sequences_mini_all` | 2 sequences (1 train, 1 validation) | approx. 400 |
| `drives_mini_all` | 2 drives (1 train, 1 validation) | approx. 4.700 |

ZOD calibrates its sensors against an ISO-8855 reference frame at the center of the rear axle at ground level, which is published as `base_link`.

| Source | Topic | Type | Description |
| ----- | ----- | ----- | ---------- |
| **Sensor:** Roof Lidars (1x Velodyne VLS128, 2x Velodyne VLP16) | `/lidar_01/point_cloud` | `sensor_msgs/msg/PointCloud2` | Point cloud in the lidar frame (approx. 254.000 points per scan) with float32 fields (`x`, `y`, `z`, `intensity`), a float64 absolute-seconds `timestamp` preserving the native per-point timing, and the uint8 `diode_index` identifying the emitter, and therefore the lidar, a point was measured by. ZOD merges the returns of all three lidars into a single scan. |
| **Sensor:** Front Camera (8 MP fisheye, 120° HFOV) | `/camera_01/image_raw`</br>`/camera_01/camera_info` | `sensor_msgs/msg/Image`</br>`sensor_msgs/msg/CameraInfo` | Anonymized RGB images (height=2168px, width=3848px) with the native Kannala-Brandt calibration, published as the `equidistant` distortion model. |
| **EgoData** | `/ego_data` | `perception_msgs/msg/EgoData` | Ego-vehicle pose in a local ENU `map` frame, plus the velocity, acceleration and yaw rate of the high-precision GNSS/IMU interpolated onto the sample's timestamp. |
| **Annotation:** 3D Lidar Objects | `/object_list/lidar_01` | `perception_msgs/msg/ObjectList` | Annotated 3D objects (`HEXAMOTION` model) in the `lidar_01` frame they are annotated in. |
| **Annotation:** 3D Camera Objects | `/object_list/camera_01` | `perception_msgs/msg/ObjectList` | The same objects, transformed into the `camera_01` frame. |
| **Meta Information:** Object Annotations | `/object_list/lidar_01/meta_info`</br>`/object_list/camera_01/meta_info` | `autonomy_datasets_msgs/msg/ObjectListMetaInfo` | Annotations without a representation in `perception_msgs/msg/Object`: `original_class`, `original_subclass`, `annotation_uuid`, `unclear`, and `object_type`, `occlusion_level`, `with_rider`, `emergency`, `artificial` and `traffic_content_visible` wherever ZOD annotates them. Associated with the object list via the header stamp and the object id. |
| **Transformations** | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Static transformations from the ISO-8855 vehicle frame (`base_link`) to the sensor frames, and the dynamic pose of `base_link` in the `map` frame. |

> **Only keyframes are annotated:** ZOD annotates one keyframe per frame and per sequence, so exactly the sample recorded closest to that keyframe publishes the object lists; every other sample of a sequence publishes an empty object list, and the drives are not annotated at all. Set `dataset_split` to a `frames` split to obtain annotated samples only.
>
> **Objects annotated in 2D only are not published:** Every ZOD object carries a 2D box in the camera image, but roughly 70% of them additionally carry a 3D cuboid. Objects without a released 3D cuboid cannot be expressed as a `perception_msgs/msg/Object` and are left out of the object lists.
>
> **Static roadside objects are published as `UNKNOWN`:** ZOD annotates poles, traffic signs, traffic signals, traffic guides and dynamic barriers alongside vehicles and vulnerable road users. `perception_msgs/msg/ObjectClassification` has no class for them, so they are classified as `UNKNOWN`, i.e. "definitely none of the other defined classes"; their ZOD class is preserved in `original_class` and `original_subclass`. Objects that ZOD flags as unclear are published as `UNCLASSIFIED`.
>
> **The remaining annotation projects are not published:** ZOD also ships lane marking and ego road segmentation, a traffic sign taxonomy of 156 classes, and road condition labels. These have no representation in `perception_msgs` and are not converted. Radar, which later ZOD releases add for sequences and drives, is not converted either.
>
> **The ego vehicle dimensions are an approximation:** ZOD publishes no dimensions for its collection vehicles, so `EgoData` reports the dimensions of a large passenger estate car, consistent with the released calibration and with the ego-return box of the development kit.

The camera runs at 10.1 Hz and the lidar at 9 Hz, and ZOD ships no synchronization table, so each sample is built from the frames closest in time to the reference sensor, which is the camera because ZOD defines the camera images as its keyframes. Frames without a match within `zod_sync_tolerance_seconds` are skipped, which typically drops the first sample of a sequence. Point clouds are motion-compensated onto the sample's timestamp, so that lidar, camera and annotations describe the same instant.

The poses ZOD publishes are relative to the first GNSS/IMU sample of a scene, with the x axis along the ego vehicle's heading at that sample. They are rotated by that heading, which is read from the scene's `oxts.hdf5`, so that `map` is an ENU frame anchored at that first sample. Scenes of the `frames` sub-dataset are independent recordings from different places, so their `map` frames are unrelated to each other.

> **Rosbags of this dataset are large.** The 8 MP images and the 254.000-point clouds amount to roughly 40 MB per sample, i.e. about 700 MB per 10 seconds of a sequence or drive. Use `zod_image_scale` to publish scaled-down camera images (the calibration is scaled with them), and `zod_rosbag_duration_seconds` to control how long a single rosbag scene gets.

#### Usage

The dataset **requires registration**: [apply for access](https://zod.zenseact.com) to receive a personal download link. Set it via the `zod_download_url` parameter in `params_zenseact_open_dataset.yml` or via the `ZOD_DOWNLOAD_URL` environment variable, and the node downloads and extracts the selected sub-dataset on the first run. Alternatively, download it manually with the CLI of the [development kit](https://github.com/zenseact/zod):

```bash
zod download -y --url="<download-link>" --output-dir=$DATASET_DIR/zenseact_open_dataset --subset=frames --version=mini
```

Both ways produce the following folder structure, in which all three sub-datasets live next to each other:

```bash
$DATASET_DIR/
    zenseact_open_dataset/
        trainval-frames-mini.json
        single_frames/
            044953/
                calibration.json
                ego_motion.json
                metadata.json
                oxts.hdf5
                annotations/
                    *.json
                camera_front_blur/
                    *.jpg
                camera_front_dnat/
                    *.jpg
                lidar_velodyne/
                    *.npy
            ...
        trainval-sequences-mini.json
        sequences/
            000002/
                ...
        trainval-drives-mini.json
        drives/
            000005/
                ...
```

> Sub-datasets downloaded into separate directories are picked up as well: an index that is not found in the dataset directory itself is also looked up one level below it, e.g. `$DATASET_DIR/zenseact_open_dataset/frames_mini/trainval-frames-mini.json`.

Select the sub-dataset, version and split using `dataset_split` in `params_zenseact_open_dataset.yml`, e.g. `frames_mini_val`, `sequences_full_train` or `drives_mini_all`. ZOD provides two anonymizations of its camera images, deep fake anonymization (`dnat`) and blurring (`blur`); the `frames` sub-dataset ships both and is selected via `zod_anonymization`, while sequences and drives ship the blurred images only.

Run the ROS node to download, convert, and store the data to rosbags while visualizing it in Rviz.

```bash
ros2 launch autonomy_datasets autonomy_datasets.launch.py dataset:=zenseact_open_dataset
```

### Thinking Cars Dataset

![commercial](https://img.shields.io/badge/license-commercial-green?style=for-the-badge)
[![Thinking Cars](https://img.shields.io/badge/origin-Thinking_Cars-green?style=for-the-badge)](https://thinking-cars.de/)

**Custom datasets** according to your needs and suitable for **commercial use** are available via an expanding network of partners [on request](mailto:info@thinking-cars.de), for example:

- Sensor data from (stereo) cameras, lidars, radars and IMU
- Object annotations
- V2X Data (e.g. [ETSI ITS Messages](https://forge.etsi.org/rep/ITS/asn1))
- Driving Trajectories and Scenarios

### Adding a new dataset

1. Create a new dataset adapter based on the existing files [here](../autonomy_datasets/autonomy_datasets/datasets/).
2. Publish annotations that have no representation in `perception_msgs/msg/Object` (e.g. the dataset's original class name) as `autonomy_datasets_msgs/msg/ObjectListMetaInfo` on the object list's `meta_info` topic, using the helpers in [`meta_info.py`](../autonomy_datasets/autonomy_datasets/datasets/meta_info.py).
3. Add documentation for the new dataset to this README and add it to the table in the [top-level README](../README.md).
4. Create a [Pull Request](https://github.com/thinking-cars/autonomy_datasets/pulls) on GitHub and wait for maintainer's feedback.
