# analysis

## Introduction
This folder contains scripts used to evaluate the **runtime performance of the full SLAM pipeline**.

Unlike the analysis tools inside individual ROS2 packages in `src/`, which focus on algorithm accuracy and internal metrics, this folder focuses on **system-level performance** such as:

- node runtime and callback execution time  
- CPU usage  
- RAM usage  

The goal is to evaluate how the **entire ROS2 pipeline behaves during experiments** and identify potential performance bottlenecks.



--------------------------------------------------

<br>
<br>
<br>

## Dependencies

Dependencies can be installed automatically using:

```bash
./scripts/install_dependencies.sh
```



--------------------------------------------------

<br>
<br>
<br>

## Usage

Runtime logging is automatically enabled when starting the pipeline with:

```bash
./scripts/start_sss_slam.sh --LOG=true
```

This records ROS2 tracing data together with CPU and RAM usage during execution.

After the experiment finishes, the recorded logs can be analyzed using the Python scripts located in this folder

Each script processes the recorded logs and generates plots or statistics describing the runtime performance of the SLAM pipeline.

