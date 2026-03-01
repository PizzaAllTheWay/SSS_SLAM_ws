# state_estimator
State estimator for Side Scan Sonar SLAM

Uses UKF-M (Uncented Kalman Filter on Manifolds) 
For Process model it has been chosen to use kinematic model, more specifically INS ie using IMU measuremnts like aceleraometer and gyroscope


# Dependencies
All required dependencies are installed automatically via the setup script described in the main `README.md`:
```bash
./scripts/install_dependencies.sh
```
If that fails for any reason, you can install them manually as described below.

* Python UKF-M Library:

The workspace should contain `SSS_SLAM_WS/libs/ukfm/`. Install it in editable mode:
```bash
cd SSS_SLAM_WS/libs/ukfm/python
python3 -m pip install -e . --break-system-packages
```
If the folder is missing or broken, clone the official repository and place it inside `libs/`:
```bash
git clone https://github.com/CAOR-MINES-ParisTech/ukfm.git
```
**Note:** The UKF-M model operates in an NED (Z-down) world frame using the standard positive gravity magnitude on the Z-axis. Our drone IMU is mounted upright and follows an ENU (Z-up) convention, where stationary Z acceleration is negative. Therefore, IMU measurements must be rotated from ENU to NED before entering the filter. Gravity is defined using the precise standard constant for consistency. If your IMU mounting differs, adjust the rotation in `config/IMU_params.yaml` accordingly. (See `SSS_SLAM_ws/libs/ukfm/python/ukfm/model/inertial_navigation.py` for absolute `g` definition)

**NOTE:** The UKF implementation was slightly modified to expose innovation statistics for benchmarking. Specifically, inside `SSS_SLAM_ws/libs/ukfm/python/ukfm/ukf/ukf.py` → `UKF.update()` the innovation covariance `S` and innovation residual `(y - ŷ)` are stored as class members (`self.S`, `self.innovation`, `self.y_bar`). These modifications are **not required** for running the state estimator itself. They are only needed for logging, debugging, NIS computation, and benchmarking.

* marine_acoustic_msgs:
```bash
sudo apt install ros-jazzy-marine-acoustic-msgs
```
Uses:
[https://docs.ros.org/en/jazzy/p/marine_acoustic_msgs/](https://docs.ros.org/en/jazzy/p/marine_acoustic_msgs/)

Other dependencies are standard ROS 2 messages (preinstalled).
* Standard ROS 2 packages (sensor_msgs, std_msgs, geometry_msgs, builtin_interfaces)
* Normal Python 3 packages (already included in ROS environment)

# Running

YOu can launch normaly with 

ros2 launch state_estimator state_estimator.launch.py

or 
ros2 launch state_estimator state_estimator.launch.py log:=True

if you want to log data for debugging and future stuff

then you will find all teh loged data in data folder

in adition there you will find the analyszing scripts that plot handy stuff like 
NIS
And also if you are using prerecorded data form LAUV you can also see how well this state estimate preforms vs the benchmark state estimator iside the LAUV
3D pose nechmark vs state estimator
3 velocity graphs benchmark vs state estimator
3 orientation graphs benchmark vs state estimator


Then if logs enabled inside the logs/ folder you will see logs/data/ folder is no longer empty