# state_estimator
## Introduction
`state_estimator` is a ROS2 package implementing an **INS-based state estimator** used as the motion model for the Side Scan Sonar SLAM pipeline.

The estimator uses **UKF-M (Unscented Kalman Filter on Manifolds)** with a **kinematic inertial navigation model**.
IMU measurements are used for continuous state propagation while additional sensors correct drift through measurement updates.

The estimator maintains a **9-state system**
```
[roll, pitch, yaw,
 vx, vy, vz,
 px, py, pz]
```
where
```
orientation : SO(3)
velocity    : body frame
position    : ENU navigation frame
```
The node runs continuously and produces a real-time vehicle state estimate.

**Subscribed Topics**

```
/hardware/imu        [sensor_msgs/Imu]
/hardware/depth      [geometry_msgs/PointStamped]
/hardware/dvl        [marine_acoustic_msgs/Dvl]
/hardware/gps        [sensor_msgs/NavSatFix]
```
These topics represent the physical sensors used for navigation. The IMU provides high-rate inertial measurements used for state propagation, while the remaining sensors act as aiding measurements that correct accumulated drift in velocity and position.

If logging is enabled, the estimator also optionally subscribes to:
```
/benchmark/state_estimate   [nav_msgs/Odometry]
```
This topic is used only for **Analysis and Evaluation**, allowing the estimated trajectory to be compared against an external reference solution during post processing. (See **Analysis and Evaluation** chapter down bellow)

**Published Topics**

```
/sss_slam/data_processing/state_estimate    [nav_msgs/Odometry]
```
This topic contains the estimated vehicle state including
- position
- Orientation
- Velocity in body frame
- Uncertainty

It is intended to be consumed by higher level modules such as the ROS2 package on Local Map Generation from Side Scan Sonar data.



--------------------------------------------------

<br>
<br>
<br>

## Dependencies

This package relies on **standard ROS2 Jazzy message types** and common **Python scientific libraries** (numpy, scipy, etc.). In most ROS2 environments these are already available.

All remaining dependencies for the SLAM workspace can be installed using:

```bash
./scripts/install_dependencies.sh
```

**UKF-M Library**

The estimator uses the **UKF-M (Unscented Kalman Filter on Manifolds)** library developed by CAOR – MINES ParisTech.

Repository:
[https://github.com/CAOR-MINES-ParisTech/ukfm](https://github.com/CAOR-MINES-ParisTech/ukfm)

Inside this project the library is located in `'SSS_SLAM_ws/libs/ukfm/'`.

Install it in editable mode:
```bash
cd SSS_SLAM_ws/libs/ukfm/python
python3 -m pip install -e . --break-system-packages
```
UKF-M allows filtering directly on **SO(3)**, meaning orientation is handled properly without Euler-angle linearization.

This estimator uses the **`INERTIAL_NAVIGATION`** model provided by the library.

**Small Modification**

The UKF implementation was slightly modified to expose innovation statistics used for debugging and NIS analysis.

Inside `'libs/ukfm/python/ukfm/ukf/ukf.py'` the filter stores:
* innovation residual
* innovation covariance

These values are only used for **logging, NIS analysis, and benchmarking**.
They are **not required for running the estimator**.

**Frame Convention**

The estimator operates in a **local ENU frame**:
* x → East
* y → North
* z → Up

Sensor mounting transforms (**body → sensor**) are configured in the YAML files inside the `config/` folder.

If sensors are mounted differently on another platform, only the configuration files need to be adjusted.



--------------------------------------------------

<br>
<br>
<br>

## Running
The state estimator can be launched as a standalone ROS2 node.
```bash
ros2 launch state_estimator state_estimator.launch.py
```
This starts the estimator and begins subscribing to the required sensor topics while publishing the estimated vehicle state.

If you want to enable internal logging for debugging and analysis, launch with:
```bash
ros2 launch state_estimator state_estimator.launch.py log:=true
```
When logging is enabled the estimator records internal filter data such as state estimates and innovation statistics. These logs are stored in the `logs/data/` directory and can later be analyzed using the plotting scripts included in the package.



--------------------------------------------------

<br>
<br>
<br>

## Analysis and Evaluation
Logging can be enabled when launching the node:
```bash
ros2 launch state_estimator state_estimator.launch.py log:=true
```
When enabled, the estimator records internal filter data into `logs/data/`.

This includes:
* state estimate history
* NIS statistics for all aiding sensors (AHRS, Depth, DVL, GPS)
* optional benchmark state if available

The recorded logs can be analyzed using the scripts inside the `logs/` folder.

**NIS Analysis**
```bash
python3 logs/plot_nis.py
```
This script evaluates **Normalized Innovation Squared (NIS)** consistency for each aiding sensor. It visualizes innovation statistics and helps determine whether measurement noise and process noise are tuned correctly.

**Benchmark Comparison**
```bash
python3 logs/plot_benchmark.py
```
This compares the estimator output with a reference state estimate if one is available.

For the thesis dataset the benchmark comes from the **onboard LAUV navigation system**, which provides a reasonable reference solution for evaluating estimator performance.

If no benchmark topic is published, the estimator still runs normally and only the NIS analysis will be available.



--------------------------------------------------

<br>
<br>
<br>

## Package Structure
```bash
state_estimator/
├── config/        sensor and filter configuration
├── launch/        ROS2 launch files
├── logs/          logging and analysis tools
├── state_estimator/
│   ├── state_estimator_node.py   main ROS2 estimator node
│   ├── measurement_models.py     sensor measurement models
│   └── utils.py                  helper utilities
```
