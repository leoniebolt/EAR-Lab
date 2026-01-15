# LVI-SAM-Easyused

An easy-to-use and reproducible implementation of **LVI-SAM**, a tightly-coupled  
**LiDAR–Visual–Inertial SLAM** system, adapted for experimental evaluation and fair comparison.

---

##  Overview

This repository provides an **easy-to-use implementation of LVI-SAM**, adapted for
practical execution and **reproducible experiments** within a SLAM comparison study.

The implementation is used to evaluate the performance of LVI-SAM on the  
**M2DGR dataset** and to compare it with other modern SLAM approaches.

This repository is part of the **EAR-Lab** project.

 **Main project repository:**  
https://github.com/leoniebolt/EAR-Lab.git

---

##  Algorithm

**LVI-SAM (LiDAR–Visual–Inertial Smoothing and Mapping)** is a tightly-coupled SLAM framework
that jointly fuses:

- LiDAR point clouds  
- Camera images  
- IMU measurements  

The system is based on **factor graph optimization** with **IMU preintegration** and
jointly optimizes visual, inertial, and LiDAR constraints.

This tight coupling enables **accurate and robust state estimation**, especially in
challenging and dynamic environments.

 **Original paper:**  
T. Shan et al.,  
*LVI-SAM: Tightly-coupled Lidar-Visual-Inertial Odometry via Smoothing and Mapping*,  
ICRA 2021

---

##  Dataset: M2DGR

All experiments are conducted using the **M2DGR dataset**, which provides synchronized
multi-sensor data, including:

- LiDAR  
- Camera  
- IMU  
- Ground truth trajectories  

 **Dataset repository:**  
https://github.com/SJTU-ViSYS/M2DGR.git

---

##  Dataset Preparation

### 1. Download the Dataset
```bash
git clone https://github.com/SJTU-ViSYS/M2DGR.git
```

### 2. Select a Sequence
Select a dataset sequence (e.g. `hall_03`) that contains **ROS1 rosbag files**.

### 3. Notes
- LVI-SAM is executed using **ROS Noetic (ROS1)**
- The M2DGR dataset can be used **directly without rosbag conversion**
- Ensure that the **rosbag path matches** the configuration defined in the launch file

---

##  Container Environment

All SLAM algorithms within the **EAR-Lab** project are executed using a **shared Docker container**
to ensure:

- Reproducibility  
- Identical system dependencies  
- Fair comparison between algorithms  

### Container Specifications
- Ubuntu 20.04  
- ROS Noetic  
- All required SLAM dependencies  

 This repository is intended to be **built and executed inside the common EAR-Lab container**.

---

##  Repository Structure

```text
LVI-SAM-Easyused/
├── config/        # Configuration files
├── launch/        # ROS launch files
├── src/           # Source code
├── scripts/       # Helper scripts
└── README.md
```

---

##  Requirements

### System
- Ubuntu 20.04  
- ROS Noetic  
- C++14  
- Python 3  

### Libraries & ROS Packages
- Eigen  
- OpenCV  
- PCL  
- GTSAM  
- sensor_msgs  
- nav_msgs  
- tf  
- cv_bridge  

---

##  Installation

### 1. Create a Catkin Workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

### 2. Clone the Repository
```bash
git clone https://github.com/Cc19245/LVI-SAM-Easyused.git
```

### 3. Install Dependencies
```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build the Workspace
```bash
catkin_make
source devel/setup.bash
```

---

##  Running LVI-SAM with M2DGR

### 1. Launch LVI-SAM
```bash
roslaunch lvi_sam M2DGR.launch
```

### 2. Play the Dataset
```bash
rosbag play <M2DGR_SEQUENCE>.bag
```

---

##  Output

LVI-SAM publishes:

- Odometry estimates  
- Trajectory path  
- TF transformations  
- Dense LiDAR map  

All results can be visualized using **RViz**.

---

##  Notes

- Configuration files are adapted specifically for the **M2DGR dataset**
- This repository focuses on **usability and reproducibility**
- Parameter tuning may be required depending on the selected dataset sequence

---

##  Comparison Context

This repository is part of a **comparative evaluation of multiple SLAM algorithms**
conducted within the **EAR-Lab** project.

Each algorithm is implemented in a **separate branch or repository** to ensure a
fair and reproducible comparison.

---

##  Acknowledgements

This project is based on the **original LVI-SAM implementation**.

All credit for the core algorithm goes to the **original authors**.

---

##  License

This project follows the **license of the original LVI-SAM repository**.
