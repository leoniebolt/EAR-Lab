# LVI-SAM-Easyused

## Overview

This repository provides an easy-to-use implementation of **LVI-SAM**, a tightly-coupled **LiDAR–Visual–Inertial SLAM** system.
It is adapted for practical execution and reproducible experiments within a SLAM comparison study.

The implementation is used to evaluate the performance of LVI-SAM on the **M2DGR dataset** and to compare it with other modern SLAM approaches.
This repository is part of the **EAR-Lab** project.

Project repository:
https://github.com/leoniebolt/EAR-Lab.git

---

## Algorithm

**LVI-SAM (LiDAR–Visual–Inertial Smoothing and Mapping)** is a tightly-coupled SLAM framework that fuses:

- LiDAR point clouds  
- Camera images  
- IMU measurements  

The system uses factor graph optimization with IMU preintegration and jointly optimizes visual, inertial, and LiDAR constraints.
This allows accurate and robust state estimation, especially in challenging environments.

Original paper:  
T. Shan et al., *LVI-SAM: Tightly-coupled Lidar-Visual-Inertial Odometry via Smoothing and Mapping*, ICRA 2021

---

## Dataset (M2DGR)

All experiments were conducted using the **M2DGR dataset**, which provides synchronized multi-sensor data including LiDAR, camera, IMU, and ground truth.

Dataset repository:  
https://github.com/SJTU-ViSYS/M2DGR.git

### Prepare Dataset

1. Download the dataset from the official repository:
```bash
git clone https://github.com/SJTU-ViSYS/M2DGR.git
```bash
2. Select a dataset sequence (e.g. `hall_03`) containing the ROS1 rosbag files.

> Note:  
> LVI-SAM is executed using **ROS Noetic (ROS1)**.  
> Therefore, the M2DGR dataset can be used **directly without rosbag conversion**.

3. Ensure that the rosbag path matches the configuration defined in the launch file.

---

## Container Environment

All SLAM algorithms within the EAR-Lab project are executed using a **shared Docker container** to ensure:

- Reproducibility  
- Identical system dependencies  
- Fair comparison between algorithms  

The container provides:
- Ubuntu 20.04  
- ROS Noetic  
- Required SLAM dependencies  

This repository is intended to be built and executed **inside the common EAR-Lab container**.

---

## Repository Structure

LVI-SAM-Easyused/
├── config/ # Configuration files
├── launch/ # ROS launch files
├── src/ # Source code
├── scripts/ # Helper scripts
└── README.md



---

## Requirements

- Ubuntu 20.04  
- ROS Noetic  
- C++14  
- Python 3  

Required libraries and ROS packages:
- Eigen  
- OpenCV  
- PCL  
- GTSAM  
- sensor_msgs  
- nav_msgs  
- tf  
- cv_bridge  

---

## Installation

### 1. Create a Catkin Workspace

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src


### 2. Clone the Repository

git clone https://github.com/Cc19245/LVI-SAM-Easyused.git

### 3. Install Dependencies

cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y



### 4. Build the Workspace

catkin_make
source devel/setup.bash


### Running LVI-SAM with M2DGR

### Launch LVI-SAM

roslaunch lvi_sam M2DGR.launch


### Play the Dataset


rosbag play <M2DGR_SEQUENCE>.bag


Output

LVI-SAM publishes:

Odometry estimates

Trajectory path

TF transformations

Dense LiDAR map

The results can be visualized using RViz.

Notes

Configuration files are adapted for the M2DGR dataset.

This repository focuses on usability and reproducibility.

Parameter tuning may be required depending on the dataset sequence.

Comparison Context

This repository is part of a comparative evaluation of multiple SLAM algorithms conducted within the EAR-Lab project.
Each algorithm is implemented in a separate branch to ensure fair and reproducible comparison.

Acknowledgements

This project is based on the original LVI-SAM implementation.
All credit for the core algorithm goes to the original authors.

License

This project follows the license of the original LVI-SAM repository.




