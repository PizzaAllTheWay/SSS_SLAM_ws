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

---

# Requirements
* Ubuntu 24.04 LTS
* ROS 2 Jazzy

Rest of the dependencies that are specific to the code pipeline can be installed using automated script:
```
./scripts/install_dependencies.sh
```
Or manually by following individual `README.md` in each ROS package inside `src/<ros-pkg>/README.md`

---

# Two Ways to Run SSS SLAM

## 1) Offline (Recorded Data – Recommended)

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
This activates internal estimator logging (e.g. state estimate NIS, residuals, benchmark metrics) for post run analysis. If the flag is not provided, the system runs in normal mode without extra debug or benchmark logging.

## 2) Online (Real Vehicle)

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

---

# Repository Structure
* data/      → Recorded CSV logs + vehicle configuration
* scripts/   → Build, record, playback and launch scripts
* src/       → ROS 2 nodes

---

# ROS 2 Package Overview

All ROS 2 packages are located in `src/`. Each package contains its own README.md with detailed explanations of its internal logic, parameters, and topic interfaces. Below is only a short high-level summary of the current packages used in the pipeline.

## sss_data

**Purpose:**

Replays recorded LAUV CSV logs and publishes them as real ROS 2 hardware topics.

This node preserves the original UNIX timestamps from the logs, publishes at realistic sensor rates, and performs no artificial filtering or modification of the data. Sonar data is replayed as raw intensity values. It enables deterministic mission replay, reproducible SLAM experiments, and generation of clean ROS 2 bags from non-ROS datasets. Detailed documentation is available in the `sss_data` package README.

**Publishes:**

```
/hardware/imu               [sensor_msgs/Imu]
/hardware/depth             [geometry_msgs/PointStamped]
/hardware/dvl               [marine_acoustic_msgs/Dvl]
/hardware/gps               [sensor_msgs/NavSatFix]
/hardware/side_scan_sonar   [marine_acoustic_msgs/RawSonarImage]

/benchmark/state_estimate [nav_msgs/Odometry]
```
