# local_map_generator

## Introduction
`local_map_generator` is a ROS 2 package that handles the local map generation stage of the Side Scan Sonar SLAM pipeline.

The package is currently built as a multi-stage processing pipeline with three implemented nodes: `swath_processing_node`, `map_generation_node`, and `feature_extraction_node`.

`swath_processing_node` synchronizes raw side scan sonar pings with the current vehicle state estimate and DVL altitude, then transforms them into a cleaner and more geometry-aware swath representation for downstream mapping and feature extraction. The node receives the estimated vehicle pose from the state estimator, receives seabed altitude from the DVL, and receives raw sonar intensity data from the side scan sonar topic. It then performs pose and altitude handling, geometric correction, blind-zone removal, and intensity normalization before publishing the processed swath and associated intermediate geometry information.

`map_generation_node` receives the intermediate swath-processing outputs and uses them to build a local probabilistic map. The node buffers processed swaths into a sparse chunked map representation, performs probabilistic pruning of candidate map cells, estimates map-cell intensities from the processed sonar data, merges the port and starboard contributions, and periodically generates and publishes a dense local map together with the corresponding map pose, altitude, and map origin.

`feature_extraction_node` receives the generated local map together with the map pose, map origin, and altitude estimate. It applies image filtering, semantic bright/shadow segmentation, connected-component labelling, landmark measurement estimation, optional shadow-based height estimation, and descriptor extraction. The resulting landmark set is published as a standard ROS2 message type for later SLAM, matching, debugging, and logging.

**Subscribed Topics**

```text
/sss_slam/data_processing/estimate/state    [nav_msgs/Odometry]
/hardware/dvl                               [marine_acoustic_msgs/Dvl]
/hardware/side_scan_sonar                   [marine_acoustic_msgs/RawSonarImage]
```

These topics are the external inputs to `swath_processing_node`. The state estimate provides the vehicle pose used to place and interpret each sonar ping. The DVL provides bottom-track altitude used to estimate transducer height above the seabed. The side scan sonar provides the raw port and starboard intensity samples that are processed into corrected swaths.

**Intermediate Topics**

```text
/sss_slam/data_processing/swath/pose                  [geometry_msgs/PoseStamped]
/sss_slam/data_processing/swath/geometric_correction  [geometry_msgs/PolygonStamped]
/sss_slam/data_processing/swath/processed             [marine_acoustic_msgs/RawSonarImage]

/sss_slam/data_processing/map_generation/altitude     [geometry_msgs/PointStamped]
/sss_slam/data_processing/map_generation/pose         [geometry_msgs/PoseStamped]
/sss_slam/data_processing/map_generation/map_origin   [geometry_msgs/PoseStamped]
/sss_slam/data_processing/map_generation/map          [sensor_msgs/Image]
```

These are intermediate outputs produced between the internal pipeline stages. The swath-processing stage publishes the timestamped pose for each processed ping, the geometric correction result, and the processed swath after blind-zone removal and intensity normalization. The map-generation stage publishes the altitude, pose, map origin, and generated dense local map used by the feature-extraction stage.

Some of these topics use standard ROS 2 message types in a slightly project-specific way. The important cases are explained in more detail in a later section.

**Published Topics**

```text
/sss_slam/data_processing/feature_extraction/pose      [geometry_msgs/PoseStamped]
/sss_slam/data_processing/feature_extraction/landmarks [sensor_msgs/PointCloud2]
```

These are the output topics from `feature_extraction_node`. The pose topic publishes the pose used for the current extraction step, while the landmarks topic publishes the detected landmark set from the current generated map.

The landmark message structure is explained in more detail in a later section, since it uses `sensor_msgs/PointCloud2` as a compact standard ROS 2 container rather than as a normal XYZ point cloud.

---

<br>
<br>
<br>

## Topic Message Structure

Most topics in this package use standard ROS 2 messages in their normal intended way. For example, `PoseStamped` is used for timestamped poses, `RawSonarImage` is used for sonar intensity data, and `sensor_msgs/Image` is used for the generated map image.

A few topics carry project-specific data inside standard ROS 2 message types. This was done intentionally to avoid custom message definitions. Custom messages would be more semantically exact, but they also make the package more annoying to use on new machines, since every PC, laptop, logging setup, or bag playback setup must first build and source the custom interface package. Using standard ROS 2 messages makes the pipeline easier to inspect with normal tools such as `ros2 topic echo`, `ros2 topic hz`, `ros2 bag`, plotting scripts, and quick debugging setups.

### `/sss_slam/data_processing/swath/geometric_correction`

```text
Type: geometry_msgs/PolygonStamped

header:
  stamp: Timestamp of the processed sonar ping
  frame_id: Reference frame used for the swath geometry

polygon.points[0]:
  x: port corrected transducer height above seabed [m]
  y: port first-bottom-return slant range [m]
  z: N/A

polygon.points[1]:
  x: starboard corrected transducer height above seabed [m]
  y: starboard first-bottom-return slant range [m]
  z: N/A
```

This topic stores the compact geometric correction result from `swath_processing_node`. The important values are the corrected transducer height and the estimated first-bottom-return range for each side of the sonar. These values are needed by downstream stages to know which part of the raw sonar ping corresponds to the water column and which part corresponds to the seabed.

`PolygonStamped` is used because the message already provides a timestamped list of 3D points. The data is not really a geometric polygon in the normal visualization sense, but it is a simple standard container for grouped floating-point geometry data.

### `/sss_slam/data_processing/map_generation/altitude`

```text
Type: geometry_msgs/PointStamped

header:
  stamp: Timestamp associated with the map update
  frame_id: Reference frame of the altitude estimate

point:
  x: N/A
  y: N/A
  z: altitude above seabed [m]
```

This topic publishes the altitude used by the map-generation and feature-extraction stages. Only one scalar value is needed, but `PointStamped` is used so the value still carries a timestamp and frame ID.

The altitude is stored in `point.z` because altitude is a vertical quantity. The `x` and `y` fields are unused.

### `/sss_slam/data_processing/map_generation/map_origin`

```text
Type: geometry_msgs/PoseStamped

header:
  stamp: Timestamp of the generated map
  frame_id: Local map frame

pose.position:
  x: local map origin x-position [pixel]
  y: local map origin y-position [pixel]
  z: N/A

pose.orientation:
  quaternion describing the local map/image orientation
```

This topic describes the pixel-space origin of the generated dense map image relative to the local map frame. The map is published as a normal `sensor_msgs/Image`, so the image alone only contains pixel intensities. The origin topic tells downstream nodes how the image pixel coordinates are placed inside the local map coordinate system.

In practice, this is what links the local map frame and the generated picture together. A landmark detected at a given pixel location in the image can be shifted by this origin, scaled by the map resolution, and then interpreted as a position in the local map frame.

`PoseStamped` is used because the origin is still a timestamped pose-like object: it has an x/y offset and may also carry orientation. It is not a full physical vehicle pose, but the standard message is convenient and easy to inspect without needing a custom map-metadata message.

### `/sss_slam/data_processing/feature_extraction/landmarks`

```text
Type: sensor_msgs/PointCloud2

header:
  stamp: Timestamp of the map frame used for feature extraction
  frame_id: map

layout:
  height: 1
  width: number of detected landmarks
  point_step: number of fields * 8 bytes
  row_step: point_step * width
  is_dense: true

Each PointCloud2 record represents one detected landmark.
All fields are stored as FLOAT64.

Fields:
  label_id: Landmark label/index
  z_r: Landmark range measurement [m]
  z_theta: Landmark bearing measurement [rad]
  R_z_rr: Measurement covariance element
  R_z_rtheta: Measurement covariance element
  R_z_thetar: Measurement covariance element
  R_z_thetatheta: Measurement covariance element
  estimated_height: Shadow-based estimated landmark height [m]
  estimated_height_std: Standard deviation of estimated height [m]
  mean_intensity: Mean landmark intensity
  std: Intensity standard deviation
  contrast: Landmark contrast descriptor
  entropy: Landmark entropy descriptor
  area: Landmark area [pixels]
  weak_polar_r: Weak polar range descriptor
  weak_polar_theta: Weak polar bearing descriptor
  height_value: Height descriptor value
  height_std: Height descriptor uncertainty
  radial_intensity_gradient: Radial intensity gradient descriptor
```

This topic publishes the complete landmark set extracted from one generated map frame. It is not used as a normal XYZ point cloud. Instead, `PointCloud2` is used as a standard ROS 2 table-like binary container, where each landmark is one record and each named field stores one scalar measurement, uncertainty, or descriptor.

All fields are packed as `FLOAT64`, including values such as `label_id` and `area`. This makes the binary layout simple: every field has the same 8-byte size, and field `i` starts at byte offset `i * 8`.

Keeping all landmarks in one message is useful because the whole set belongs to the same map frame and timestamp. This avoids publishing one small message per landmark, avoids ambiguity about which landmarks were extracted together, and keeps synchronization with the published extraction pose simple.

A custom `LandmarkSet.msg` would be cleaner semantically, but it would make testing, logging, replay, and quick inspection more annoying because every machine would need to build and source the custom message package first. `PointCloud2` is already supported by standard ROS 2 tooling, making it more portable for development and debugging.

---

<br>
<br>
<br>

## Dependencies

This package depends on standard ROS 2 Jazzy tooling, in addition the Rust ROS 2 client library `r2r`, and `marine_acoustic_msgs` for acoustic sensor message types. 

The swath-processing node itself is implemented in Rust and uses `cargo` together with the crates defined in the package, including `r2r`, `tokio`, `nalgebra`, `csv`, `serde`, `ta`, and `sysinfo`. The workspace as a whole targets **Ubuntu 24.04 LTS** and **ROS 2 Jazzy**. The acoustic message dependency is the same one already used elsewhere in the SSS SLAM pipeline.   

Dependency:

```bash
sudo apt install ros-jazzy-marine-acoustic-msgs
```

---

<br>
<br>
<br>

## Running

The package can be launched through the normal ROS 2 launch system:

```bash
ros2 launch local_map_generator local_map_generator.launch.py
```

If logging is needed for debugging, evaluation, or plotting:

```bash
ros2 launch local_map_generator local_map_generator.launch.py log:=true
```

This matches the same logging pattern used elsewhere in the workspace, where internal data is optionally recorded for later analysis at `logs/`.  

---

<br>
<br>
<br>

## Analysis and Evaluation

Each Rust node inside the package is organized so it can be tested in isolation before full deployment. Inside each `rust/*_node/` folder there is a dedicated `test/` folder with its own `bin` target, test dataset, and local utilities for verifying that node independently. This is useful for checking correctness of the individual processing stage before running the full package.

For lower-level runtime and performance inspection, the isolated test folders can be used together with normal Rust profiling tools such as `cargo flamegraph` or similar profiling workflows. This makes it possible to inspect callback cost and hot paths without launching the full ROS 2 pipeline.

At package level, the `logs/` folder is used for full-system analysis. When logging is enabled during launch, runtime logs, swath logs, altitude logs, geometric-correction logs, and processed-swath logs are written there for later plotting and debugging. The Python scripts in `logs/` are used to analyze how the different parts behave together, including runtime, resource use, and the results form each node before and after processing. 

---

<br>
<br>
<br>

## Package Structure

```text
local_map_generator/
├── config/         ROS 2 parameters for sonar geometry, transducer mounting, and processing settings
├── launch/         ROS 2 launch files for starting the package
├── logs/           Python plotting tools and logged output data for analysis
└── rust/
    └── *_node/
        ├── src/    main Rust node implementation and processing logic
        └── test/   isolated test binaries, datasets, and debugging utilities
```

* `config/` stores the parameter files that define the sonar setup, including transducer position, transducer orientation, beam angle, range, sample order, and blind-zone tuning values read by the node at runtime.  
* `launch/` contains the ROS 2 launch entry points used to start the package in normal or logging-enabled mode.
* `logs/` contains the Python scripts used to inspect the logged swath-processing outputs and the generated CSV log files.
* `rust/*_node/src/` contains the actual Rust node and processing implementation, including the ROS 2 subscriptions/publications and the swath-processing pipeline.  
* `rust/*_node/test/` contains isolated tests for node functionality before deployment into the full package workflow.

