# LiLoc SLAM Setup and Usage

This guide explains how to set up and run the LiLoc SLAM system using Docker.

---

## Prerequisites

- Docker installed and configured
- Git
- A working ROS environment within Docker

---

## Setup

### 1. Clone the Repository

```bash
git clone https://github.com/leoniebolt/EAR-Lab.git
cd EAR-Lab
git fetch
git switch LiLoc
```

### 2. Build and Run the Docker Container

```bash
docker build -t liloc_image .
docker run -it -v <your_work_folder>:/app liloc_image
```

> **Note:** Replace `<your_work_folder>` with the full path to your local work folder.

### 3. Setup Workspace Inside Docker

```bash
mkdir -p liloc_ws/src
cd liloc_ws/src
```

### 4. Clone Required Repositories

```bash
git clone https://github.com/koide3/ndt_omp
git clone https://github.com/Livox-SDK/livox_ros_driver
git clone https://github.com/Yixin-F/LiLoc
```

### 5. Build the Workspace

```bash
cd ..
catkin_make
source devel/setup.bash
```

---

## Run LiLoc SLAM and Get TUM Poses

### Step 1: Enable GUI Display (on host machine)

```bash
xhost +local:docker
```

This allows the Docker container to display visualizations on your host.

### Step 2: Launch SLAM for the M2DGR Dataset

```bash
roslaunch liloc run_m2dgr.launch
```

### Step 3: Extract Robot Poses (new terminal, in Docker, sourced)

```bash
python3 extract_tum.py
```

This script extracts robot poses during the SLAM run.

### Step 4: Play the Dataset (new terminal, in Docker, sourced)

```bash
cd m2dgr_dataset
rosbag play <file_name>.bag
```

> **Note:** For replicating the results in the paper, use `hall_03.bag`

### Step 5: Finalize

Once the `.bag` file finishes playing, press `Ctrl+C` in the terminal where the Python script is running. A `.txt` file with all poses for evaluation will be created automatically.

---

## Important Notes

- ✓ Ensure all required repositories are cloned inside `liloc_ws/src`
- ✓ Always source the workspace (`source devel/setup.bash`) before running ROS nodes or Python scripts
- ✓ Verify your Docker container has access to the host display using `xhost`
