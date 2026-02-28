# sss_data

## Description
`sss_data` replays recorded underwater vehicle CSV logs and publishes them as real ROS 2 hardware topics. It simulates onboard IMU, pressure sensor (depth), DVL (bottom-track), and side-scan sonar using original recorded timestamps.

All messages use the recorded UNIX timestamp in `header.stamp`.

---

# Dependencies
All required dependencies are installed automatically via the setup script as described in main `README.md`:
```bash
./scripts/install_dependencies.sh
```
If that fails for any reason, you can install them manually as described below.

* marine_acoustic_msgs:
```bash
sudo apt install ros-jazzy-marine-acoustic-msgs
```
Uses:
[https://docs.ros.org/en/jazzy/p/marine_acoustic_msgs/](https://docs.ros.org/en/jazzy/p/marine_acoustic_msgs/)

Other dependencies are standard ROS 2 messages (preinstalled).
* Standard ROS 2 packages (sensor_msgs, std_msgs, geometry_msgs, builtin_interfaces)
* Normal Python 3 packages (already included in ROS environment)

---

# Published Topics
```
/hardware/imu               [sensor_msgs/Imu]
/hardware/depth             [std_msgs/Float64]
/hardware/dvl               [marine_acoustic_msgs/Dvl]
/hardware/side_scan_sonar   [marine_acoustic_msgs/RawSonarImage]

/benchmark/state_estimate [nav_msgs/Odometry]
```

---

## Topic Content Summary
**/hardware/imu** — `sensor_msgs/Imu`
IMU data from Microstrain 3DM-GX4 AHRS.

* Orientation quaternion computed from roll/pitch/yaw (AHRS output)
* Angular velocity from gyroscope (rad/s)
* Linear acceleration from accelerometer (m/s²)
* Timestamp is original logged UNIX time
* No covariance fields filled (can be added from Navigation.AUV.Navigation config if needed)

**/hardware/depth** — `std_msgs/Float64`
Depth in meters derived from Keller-33x pressure sensor.

* Raw depth only
* No variance included (Navigation config suggests ~0.05 m in simulation)

**/hardware/dvl** — `marine_acoustic_msgs/Dvl`
Nortek DVL 1 MHz bottom-track mode.

* 3D bottom velocity (m/s)
* Altitude from bottom lock (m)
* Computed ground speed and course
* Sound speed (from log)
* No beam-level velocities exposed
* Covariance not set (Navigation config shows DVL process noise on order 5e-5 to 5e-3)

**/hardware/side_scan_sonar** — `marine_acoustic_msgs/RawSonarImage`
DeepVision OSM2 Sidescan (SIDESCAN mode only).

* Center frequency: ~640 kHz
* Range used in mission: 60 m
* 2000 pixels per ping (1000 per side)
* Slant resolution ≈ 0.06 m per bin
* uint8 intensity values
* No beam geometry or TVG applied in node (raw intensity replay)

**/benchmark/state_estimate** — `nav_msgs/Odometry`
Onboard navigation (INS) solution replayed for benchmarking and validation purposes only.

* Pose in `map` frame (x, y, z + quaternion orientation from roll/pitch/yaw)
* Linear velocity in `base_link` (surge, sway, heave)
* Pose covariance from `NavigationUncertainty.csv`
* Twist covariance includes linear velocity variances
* Original logged UNIX timestamps

This topic can be used as system state estimate, however for this pipeline it is **NOT** used as the system state estimate.
For this SLAM pipeline it exists purely as a reference solution to benchmark, compare, and validate our own SLAM/INS implementations against the onboard navigation performance.

---

# Hardware
This dataset originates from a LAUV platform. Many sensors exist onboard, but the critical SLAM sensors used here are:

## Sensor Specifications
Reference frame: body frame (x forward, y starboard, z down).
All this configuraon is taken form  LAUV – fridtjof config
You can find it under data/Config.ini and specific senstions are noded down bellow

### IMU
* Section: `Sensors.MicrostrainMIP`
* Model: Microstrain 3DM-GX4
* Frame: Body frame reference (x forward, y starboard, z down)
* Orientation (roll, pitch, yaw): **0°, 0°, 0°** (Identity matrix in config)
* Position: **0.0, 0.0, 0.0 [m]** (no lever arm defined in Navigation)
* Sampling: 50 Hz
* Provides: orientation (AHRS), angular velocity, linear acceleration
* Internal AHRS filtering enabled
* Used as primary heading + inertial reference
* Simulation noise (DUNE):
  * Angular velocity std: ~0.05 rad/s
  * Euler angle std: ~0.05 rad

Docs: [https://files.microstrain.com/3DM-GX4-25_User_Manual_(8500-0047).pdf](https://files.microstrain.com/3DM-GX4-25_User_Manual_(8500-0047).pdf)

### Depth Sensor
* Section: `Sensors.Keller`
* Model: Keller-33x
* Frame: Body frame reference (x forward, y starboard, z down)
* Orientation (roll, pitch, yaw): **0°, 0°, 0°** (no rotation specified in config)
* Position (x, y, z): **0.658, 0.000, -0.059 [m]** (Navigation lever arm)
* Measurement: Pressure-based depth
* Execution frequency: 10 Hz (config)
* Used for vertical constraint in navigation
* Simulation noise (DUNE):
  * Depth std: ~0.05 m
Docs: [https://www.omniinstruments.co.uk/p/keller-series-33x-pressure-transmitters/](https://www.omniinstruments.co.uk/p/keller-series-33x-pressure-transmitters/)

### DVL
* Section: `Sensors.NortekDVL`
* Model: Nortek DVL 1 MHz (Ethernet)
* Frame: Body frame reference (x forward, y starboard, z down)
* Orientation (roll, pitch, yaw): **0°, -90°, 45°**
* Position (x, y, z): **0.50, 0.00, 0.20 [m]**
* Sampling rate: 5 Hz
* Provides: 3D bottom-track velocity + altitude
* Used as primary velocity source for dead-reckoning
* Simulation noise (DUNE):
  * Ground velocity std: ~0.002 m/s
Docs: [https://www.nortekgroup.com/products/dvl1000-6000m](https://www.nortekgroup.com/products/dvl1000-6000m)

### Side Scan Sonar
* Section: `Supervisors.Delegator/Sidescan` → `Sensors.DeepVisionOSM2`
* Model: DeepVision OSM2
* Frame: Body frame reference (x forward, y starboard, z down)
* Orientation (roll, pitch, yaw): **Not explicitly defined in config**
* Position: **Not explicitly defined in config**
* Mode: SIDESCAN
* Range: 60 m
* Resolution: 1000 bins per side (~6 cm slant resolution)
* Used for seabed imaging and acoustic SLAM
Docs: [https://deepvisionsonar.com/](https://deepvisionsonar.com/)

---


