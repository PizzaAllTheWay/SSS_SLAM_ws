# local_map_generator

## Introduction
`local_map_generator` is a ROS 2 package that handles the processing stage of the Side Scan Sonar SLAM pipeline. 

Will have to rewrite this later when adding new nodes like probabilistic map generation and feature extraction.

`swath_processing_node`, where raw side scan sonar pings are synchronized with the current vehicle state estimate and DVL altitude, then transformed into a cleaner and more geometry aware swath representation for downstream mapping and feature extraction. The node receives the estimated vehicle pose from the state estimator, receives seabed altitude from the DVL, and receives raw sonar intensity data from the side scan sonar topic. It then performs a sequence of operations on each ping: pose/altitude handling, geometric correction, blind-zone removal, and intensity normalization before publishing the processed swath and associated intermediate geometry information.

**Subscribed Topics**
```text
/sss_slam/data_processing/estimate/state    [nav_msgs/Odometry]
/hardware/dvl                               [marine_acoustic_msgs/Dvl]
/hardware/side_scan_sonar                   [marine_acoustic_msgs/RawSonarImage]
```

The state estimate provides the vehicle pose used to place and interpret each sonar ping. The DVL provides bottom-track altitude used to estimate transducer height above the seabed. The side scan sonar provides the raw port and starboard intensity samples that are processed into a corrected swath. 

**Intermediate Topics**

```text
/sss_slam/data_processing/swath/pose                  [geometry_msgs/PoseStamped]
/sss_slam/data_processing/swath/geometric_correction  [geometry_msgs/PolygonStamped]
/sss_slam/data_processing/swath/processed             [marine_acoustic_msgs/RawSonarImage]
```

These are intermediate outputs produced by the swath-processing stage. The pose topic contains the pose associated with the ping timestamp, the geometric correction topic contains the estimated first-bottom-return geometry for port and starboard, and the processed swath topic contains the corrected sonar image after blind-zone removal and intensity normalization. At the current stage these are intermediate products used inside the broader local-map-generation pipeline. More downstream mapping and feature-extraction topics can be added later.  

**Published Topics**

```text
N/A
```

At the moment this package ends at the intermediate swath-processing stage. No final local-map output topic is published yet.

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

