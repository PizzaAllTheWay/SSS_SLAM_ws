# Side Scan Sonar SLAM
## Overview
This repository contains a full ROS 2 pipeline for Graph-Based feature SLAM using Side Scan Sonar (SSS).
It supports both **offline processing (recorded data)** and **online deployment on real vehicles**.

The system was developed as part of a Masters Thesis and targets underwater platforms running ROS 2.

The pipeline works with:
* IMU
* Depth sensor
* DVL
* Side Scan Sonar

All data must be published using standard ROS 2 message formats.

**Requirements**

* Ubuntu 24.04 LTS
* ROS 2 Jazzy

Rest of the dependencies that are specific to the code pipeline can be installed using automated script:
```
./scripts/install_dependencies.sh
```
Or manually by following individual `README.md` in each ROS package inside `src/<ros-pkg>/README.md`



--------------------------------------------------

<br>
<br>
<br>

## Repository Structure
* **data/** → Contains recorded mission datasets and vehicle configuration files. These logs typically originate from underwater vehicles such as LAUV platforms and include sensor recordings used for offline SLAM experiments and reproducible testing. The folder may also contain configuration files describing the vehicle setup used during data collection.

* **libs/** → Holds external libraries and research dependencies used by the project. These are usually third-party frameworks required by parts of the SLAM pipeline (for example UKF-M). Keeping them inside the repository ensures reproducibility and avoids dependency version conflicts across systems.

* **scripts/** → Utility scripts used to simplify common workflows such as building the workspace, starting the SLAM pipeline, recording datasets, or replaying sensor logs. These scripts automate multi-step ROS operations so the full system can be started or evaluated with a single command.

* **src/** → Contains all ROS 2 packages that implement the actual system components. Each package represents an individual module of the pipeline (for example sensor replay, state estimation, or SLAM processing) and can be launched independently or as part of the full system.



--------------------------------------------------

<br>
<br>
<br>

## Two Ways to Run SSS SLAM

**1) Offline (Recorded Data – Recommended)**

The pipeline is designed to run from ROS 2 bags. The normal workflow is to start the full SLAM system first and then play back recorded data into it. Run:

```bash
./scripts/build_sss_slam.sh
./scripts/start_sss_slam.sh
./scripts/play_sss_data.sh
```

The last command replays a ROS 2 bag into the running system. You can replace it with any of your own ROS 2 bags if they are stored elsewhere. The pipeline does not depend on how the bag was created as long as the topic names and message types match.

If your vehicle did not record ROS 2 bags and instead logged data using DUNE/Neptus (as is the case for the LAUV platform from LSTS), the data will typically exist as CSV logs. In that case you must first convert the recorded CSV files into ROS 2 topics and record them into a bag. This repository provides a custom replay node for that exact purpose. Run:

```bash
./scripts/record_sss_data.sh
```

This script builds the workspace, publishes the CSV logs as proper ROS 2 hardware topics using the original timestamps, and records them into a ROS 2 bag. Once this conversion step is complete, you simply run the standard pipeline shown above and process the generated bag like any other ROS 2 dataset.

If you want to enable data logging and benchmarking while running the SLAM pipeline, start it with logging enabled:
```bash
./scripts/start_sss_slam.sh --LOG=true
```
This activates internal logging for all relevant ROS2 nodes (e.g. state estimate NIS, residuals, benchmark metrics) for post run analysis. If the flag is not provided, the system runs in normal mode without extra debug or benchmark logging.

**2) Online (Real Vehicle)**

The full pipeline runs directly on any vehicle with:

* Ubuntu 24.04 LTS
* ROS 2 Jazzy
* Sensors publishing to the same topic types

As long as:

* IMU → `sensor_msgs/Imu`
* Depth → `std_msgs/Float64`
* DVL → `marine_acoustic_msgs/Dvl`
* Sidescan → `marine_acoustic_msgs/RawSonarImage`

For NTNU's BlueROV, ROS 2 is already native on the platform, so the pipeline can be launched directly. However, onboard compute is typically insufficient for full SLAM processing, so recording data and processing offline is recommended. For deployment and data collection on BlueROV, use:

[https://github.com/PizzaAllTheWay/bluerov2_data_collection_ws](https://github.com/PizzaAllTheWay/bluerov2_data_collection_ws)



--------------------------------------------------

<br>
<br>
<br>

## ROS 2 Package Overview
All ROS 2 packages are located in `src/`. Each package contains its own README.md with detailed explanations of its internal logic, parameters, and topic interfaces. Below is only a short high-level summary of the current packages used in the pipeline.

### sss_data
**Purpose:**

Replays recorded LAUV CSV logs and publishes them as real ROS 2 hardware topics.

This node preserves the original UNIX timestamps from the logs, publishes at realistic sensor rates, and performs no artificial filtering or modification of the data. Sonar data is replayed as raw intensity values. It enables deterministic mission replay, reproducible SLAM experiments, and generation of clean ROS 2 bags from non-ROS datasets. Detailed documentation is available in the `sss_data` package README.

**Subscribes:**

```
N/A
```

**Publishes:**

```
/hardware/imu               [sensor_msgs/Imu]
/hardware/depth             [geometry_msgs/PointStamped]
/hardware/dvl               [marine_acoustic_msgs/Dvl]
/hardware/gps               [sensor_msgs/NavSatFix]
/hardware/side_scan_sonar   [marine_acoustic_msgs/RawSonarImage]

/benchmark/state_estimate [nav_msgs/Odometry]
```

### sss_data_processing/state_estimator
**Purpose:**

The `state_estimator` package provides the vehicle motion estimate used during the data processing stage of the pipeline, primarily for local map generation from Side Scan Sonar data. It performs inertial navigation using an **INS-based Unscented Kalman Filter on Manifolds (UKF-M)**, where IMU measurements propagate the vehicle state while additional sensors are used to bound drift and maintain consistency. The estimator outputs a continuous estimate of vehicle **orientation, velocity, and position**, which is required for motion compensation and for placing sonar observations correctly when constructing local maps. Detailed documentation is available in the `sss_data_processing/state_estimator` package README.

**Subscribes:**

```
/hardware/imu        [sensor_msgs/Imu]
/hardware/depth      [geometry_msgs/PointStamped]
/hardware/dvl        [marine_acoustic_msgs/Dvl]
/hardware/gps        [sensor_msgs/NavSatFix]
```
These sensors provide inertial propagation and aiding measurements used to bound drift and stabilize the navigation solution.

If logging is enabled, the estimator also optionally subscribes to:
```
/benchmark/state_estimate   [nav_msgs/Odometry]
```
This topic is used only for **Analysis and Evaluation**, allowing the estimated trajectory to be compared against an external reference solution during post-processing.

A more detailed explanation of how these measurements are used inside the estimator can be found in the `sss_data_processing/state_estimator` package README.

**Publishes:**

```
/sss_slam/data_processing/state_estimate    [nav_msgs/Odometry]
```

This topic contains the estimated vehicle state, including **position, orientation, body-frame velocity, and associated covariance**. It is used by downstream components of the SLAM pipeline, such as the local map generation and feature extraction modules, which rely on an accurate vehicle trajectory to correctly place sonar observations in space.
