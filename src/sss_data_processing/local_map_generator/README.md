# local_map_generator

## Introduction
`local_map_generator` is a ROS 2 package that handles the local map generation stage of the Side Scan Sonar SLAM pipeline.

The package is currently built as a multi-stage processing pipeline. At the moment, two nodes are implemented: `swath_processing_node` and `map_generation_node`. A future feature-extraction node will be added later, so the package is not yet fully complete.

`swath_processing_node` synchronizes raw side scan sonar pings with the current vehicle state estimate and DVL altitude, then transforms them into a cleaner and more geometry-aware swath representation for downstream mapping and feature extraction. The node receives the estimated vehicle pose from the state estimator, receives seabed altitude from the DVL, and receives raw sonar intensity data from the side scan sonar topic. It then performs a sequence of operations on each ping: pose and altitude handling, geometric correction, blind-zone removal, and intensity normalization before publishing the processed swath and associated intermediate geometry information.

`map_generation_node` receives the intermediate swath-processing outputs and uses them to build a local probabilistic map. The node buffers processed swaths into a sparse chunked map representation, performs probabilistic pruning of candidate map cells, estimates map-cell intensities from the processed sonar data, merges the port and starboard contributions, and periodically generates and publishes a dense local map together with the corresponding map pose and map origin. This output is intended to serve as the input for future downstream stages such as feature extraction.

**Subscribed Topics**
```text
/sss_slam/data_processing/estimate/state    [nav_msgs/Odometry]
/hardware/dvl                               [marine_acoustic_msgs/Dvl]
/hardware/side_scan_sonar                   [marine_acoustic_msgs/RawSonarImage]
```

These topics are subscribed to by `swath_processing_node`. The state estimate provides the vehicle pose used to place and interpret each sonar ping. The DVL provides bottom-track altitude used to estimate transducer height above the seabed. The side scan sonar provides the raw port and starboard intensity samples that are processed into a corrected swath.

**Intermediate Topics**

!!!!!
// TODO: SHoudl explain in more detail each topic an dhow it is structured
AT least soem of them no nee dfo rall but some definitely
geometric_correction
altitude
map_origin
!!!!!

```text
/sss_slam/data_processing/swath/pose                  [geometry_msgs/PoseStamped]
/sss_slam/data_processing/swath/geometric_correction  [geometry_msgs/PolygonStamped]
/sss_slam/data_processing/swath/processed             [marine_acoustic_msgs/RawSonarImage]

/sss_slam/data_processing/map_generation/altitude     [geometry_msgs/PointStamped]
/sss_slam/data_processing/map_generation/pose         [geometry_msgs/PoseStamped]
/sss_slam/data_processing/map_generation/map_origin   [geometry_msgs/PoseStamped]
/sss_slam/data_processing/map_generation/map          [sensor_msgs/Image]
```

These are intermediate outputs produced inside the local-map-generation pipeline. The swath-processing stage publishes the pose associated with each ping timestamp, the estimated first-bottom-return geometry for port and starboard, and the processed swath after blind-zone removal and intensity normalization. The map-generation stage then publishes the pose used for the current map update, the map origin describing where the dense map starts in the map frame, and the generated local map itself as an image. At the current stage these topics are intermediate products of the broader SLAM pipeline and are expected to be used later by downstream stages such as feature extraction.

**Published Topics**

```text
N/A
```

At the current stage, this package publishes intermediate outputs for the internal SLAM processing pipeline rather than a final end-user output topic. The swath-processing outputs feed the map-generation stage, and the map-generation outputs are intended to feed future downstream stages such as feature extraction.

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

