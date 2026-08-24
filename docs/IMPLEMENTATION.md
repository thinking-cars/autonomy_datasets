# Implementation Details


## Supported Datasets

This repository supports various automated driving datasets including:
- [**NVIDIA PhysicalAI AV Dataset**](#nvidia-physicalai-av-dataset)
- [**nuScenes**](#nuscenes-dataset)
- [**Waymo Open Dataset**](#waymo-open-dataset)
- [**DrivIng**](#driving-dataset)
- [**MAN TruckScenes**](#man-truckscenes-dataset)
- [**TUM Traffic**](#tum-traffic-dataset)
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
| **Sensor:** Front Radar (Continental ARS 408-21) | `/radar_01/point_cloud` | `sensor_msgs/msg/PointCloud2` | Radar detections from front radar as point cloud with fields (`x`, `y`, `z`, `radial_velocity`, `rcs`). |
| **Sensor:** Front-Right Radar (Continental ARS 408-21) | `/radar_02/point_cloud` | `sensor_msgs/msg/PointCloud2` | Radar detections from front-right radar as point cloud with fields (`x`, `y`, `z`, `radial_velocity`, `rcs`). |
| **Sensor:** Back-Right Radar (Continental ARS 408-21) | `/radar_03/point_cloud` | `sensor_msgs/msg/PointCloud2` | Radar detections from back-right radar as point cloud with fields (`x`, `y`, `z`, `radial_velocity`, `rcs`). |
| **Sensor:** Back-Left Radar (Continental ARS 408-21) | `/radar_04/point_cloud` | `sensor_msgs/msg/PointCloud2` | Radar detections from back-left radar as point cloud with fields (`x`, `y`, `z`, `radial_velocity`, `rcs`). |
| **Sensor:** Front-Left Radar (Continental ARS 408-21) | `/radar_05/point_cloud` | `sensor_msgs/msg/PointCloud2` | Radar detections from front-left radar as point cloud with fields (`x`, `y`, `z`, `radial_velocity`, `rcs`). |
| **EgoData** | `/ego_data` | `perception_msgs/msg/EgoData`| Ego-vehicle's dimensions and dynamics state (`EGO` model) in `map` frame. Pose from the dataset's ego poses, velocity, acceleration, yaw rate, steering angle, standstill flag, turn indicator and brake light from the CAN bus expansion. |
| **Annotation:** 3D Lidar Objects | `/object_list/lidar_01` | `perception_msgs/msg/ObjectList` | Annotated 3D objects (`HEXAMOTION` model) visible in lidar scan. |
| **Annotation:** 3D Front Camera Objects | `/object_list/camera_01` | `perception_msgs/msg/ObjectList` | Annotated 3D objects (`HEXAMOTION` model) visible in front camera image. |
| **Transformations** | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Static transformations to all sensor frames and dynamic transformation from `map` to vehicle frame. |

#### Usage

[Download](https://www.nuscenes.org/nuscenes#download) the dataset (including CAN Bus and Map Expansion) and ensure the following folder structure is correct:

```bash
$DATASET_DIR/
    nuscenes/
        can_bus/
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


### DrivIng Dataset

[![non-commercial](https://img.shields.io/badge/license-non--commercial-red)](https://creativecommons.org/licenses/by-nc-nd/4.0/)
[![Harvard Dataverse](https://img.shields.io/badge/origin-Harvard_Dataverse-green)](https://doi.org/10.7910/DVN/VBZKDY)

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


### MAN TruckScenes Dataset

[![non-commercial](https://img.shields.io/badge/license-non--commercial-red)](https://creativecommons.org/licenses/by-nc-sa/4.0/)
[![AWS Open Data](https://img.shields.io/badge/origin-AWS_Open_Data-green)](https://registry.opendata.aws/man-truckscenes/)

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


### TUM Traffic Dataset

[![non-commercial](https://img.shields.io/badge/license-non--commercial-red)](https://creativecommons.org/licenses/by-nc-nd/4.0/)
[![TUM Traffic Dataset](https://img.shields.io/badge/origin-TUM_Traffic_Dataset-green)](https://tum-traffic-dataset.github.io)

![Rviz Screenshot TUM Traffic Dataset](./assets/rviz_tum_traffic.png)

The [TUM Traffic Dataset](https://innovation-mobility.com/en/project-providentia/a9-dataset/) (`TUMTraf`) is recorded by roadside sensors mounted on the gantry bridges of the [Providentia++](https://innovation-mobility.com/en/project-providentia/) test field along the A9 motorway and the S110 intersection near Munich, Germany. It is an **infrastructure dataset without an ego vehicle**, so no `/ego_data` is published; the sensor station is published as a static `base_link`. The dataset is licensed under [CC BY-NC-ND 4.0](https://creativecommons.org/licenses/by-nc-nd/4.0/).

The dataset is released as one archive per release and subset. All releases share a common file layout but differ in their sensors, directory names and label formats, so the adapter discovers the recordings, sensors and frame timestamps from the file names instead of hard-coding each release:

| Release | Subsets | Sensors | Annotations |
| ------- | ------- | ------- | ----------- |
| `R00` TUMTraf A9 Highway (image subsets) | `r00_s00` ... `r00_s02` | 4 A9 gantry cameras (`s040`, `s050`) | 3D box corners projected into the image — no object list published, see below |
| `R00` TUMTraf A9 Highway (lidar subsets) | `r00_s03`, `r00_s04` | Roadside lidars | Native pre-OpenLABEL 3D cuboids (yaw-only orientation, no persistent track IDs) |
| `R01` TUMTraf A9 Highway Extended | `r01_s01` ... `r01_s04` | 4 A9 gantry cameras (`s040`, `s050`) | 3D box corners projected into the image — no object list published, see below |
| `R02` TUMTraf Intersection | `r02_s01` ... `r02_s04` | 2 S110 cameras, 2 S110 Ouster lidars | OpenLABEL 3D cuboids with track IDs |

> **Only `R00` to `R02` are supported.**

> **No object list for the `R00` image subsets and `R01`:** these releases annotate a 3D box only as its 8 corners projected into the 2D image (`box3d_projected`), without releasing the 3D pose (position, dimensions, orientation) that produced the projection. Recovering that pose from a single monocular projection is an ill-posed inverse problem without extra assumptions (an assumed object size and ground plane to resolve the missing depth), so this adapter does not attempt it. These recordings still publish their raw camera images, calibration and transforms — just no `object_list` topic. Only recordings with real 3D cuboids (the `R00` lidar subsets, native pre-OpenLABEL format, and `R02`, OpenLABEL) publish `/object_list/lidar_01`.
>
> The `R00` lidar subsets ship no `_calibration` directory and their native label format carries no coordinate system information either (unlike OpenLABEL), so no calibration source exists in the dataset for them. Since these recordings have exactly one sensor (a lidar) and no calibrated pose to fall back to, `base_link` is aliased to that sensor's frame with an identity transform instead of publishing no static transform at all — `/tf_static` therefore still resolves, just without a real physical offset.

Sensors are mapped onto the canonical topics in a fixed order, so `camera_01` and `lidar_01` are the sensors the object list is annotated in. The example below lists the topics of the intersection subsets (`R02`):

| Source | Topic | Type | Description |
| ----- | ----- | ----- | ---------- |
| **Sensor:** South Lidar (Ouster OS1-64) | `/lidar_01/point_cloud` | `sensor_msgs/msg/PointCloud2` | Point cloud in the sensor frame with float32 fields (`x`, `y`, `z`, `intensity`) and a float64 absolute-seconds `timestamp`, preserving the native per-point timing. |
| **Sensor:** North Lidar (Ouster OS1-64) | `/lidar_02/point_cloud` | `sensor_msgs/msg/PointCloud2` | Point cloud in the sensor frame, fields as above. |
| **Sensor:** South1 Camera (Basler 8mm) | `/camera_01/image_raw`</br>`/camera_01/camera_info` | `sensor_msgs/msg/Image`</br>`sensor_msgs/msg/CameraInfo` | RGB images (height=1200px, width=1920px) with the native calibration. |
| **Sensor:** South2 Camera (Basler 8mm) | `/camera_02/image_raw`</br>`/camera_02/camera_info` | `sensor_msgs/msg/Image`</br>`sensor_msgs/msg/CameraInfo` | RGB images (height=1200px, width=1920px) with the native calibration. |
| **Annotation:** 3D Lidar Objects | `/object_list/lidar_01` | `perception_msgs/msg/ObjectList` | Annotated 3D objects (`HEXAMOTION` model) in the lidar_01 frame, with the track UUID, the native class and the native attributes (occlusion level, body color, number of points) in `meta_info`. |
| **Transformations** | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Static transformations from the sensor station (`base_link`) to all sensor frames, and the station's static pose in the `map` frame. |

> **The object list is only geometrically accurate against `lidar_01`:** on releases with more than one lidar (`R02` and newer), the objects are annotated directly in the reference lidar's own frame (`lidar_01`) and republished as-is; they are not re-derived per sensor. Overlaying `/object_list/lidar_01` onto `/lidar_02/point_cloud` (or any other non-reference lidar) will show a visible offset, because the dataset's own released extrinsic calibration between its lidars is imprecise (e.g. for `s110_lidar_ouster_south`/`s110_lidar_ouster_north` in `R02`), which is why the dev kit ships a dedicated `src/registration/point_cloud_registration.py` to refine this pairing via ICP. This adapter does not run that registration step, so it is not a bug in this pipeline nor something a synchronization or calibration-parsing fix here could correct; only `lidar_01` is guaranteed to align with the published objects.
>
> **Some tracked objects visibly float above or sink into the ground:** on `R02` and newer, a track's `z` and `height` are often set once and held constant for its whole lifetime while only `x`/`y`/`yaw` keep updating — confirmed directly in the raw label files, where most multi-frame tracks in a sample recording had byte-identical `z`/`height` despite moving tens of meters. This can leave a track that started well aligned drifting out of alignment later (e.g. over a stretch with different road elevation), or leave it wrong for its entire length if the frozen value was never accurate to begin with (e.g. estimated from only a handful of lidar points at long range and never revisited, even once the object is later observed with far denser support). This is a property of the dataset's own annotations, not of this adapter: `cuboid.val` is passed through per frame unmodified.

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
