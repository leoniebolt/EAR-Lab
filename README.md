# EAR-Lab – ROS 2 SLAM Evaluation Environment

This repository provides a Docker-based ROS 2 environment for running and evaluating SLAM algorithms (e.g. **LIO-SAM**) using datasets such as **M2DGR**, with tools for map extraction and quantitative evaluation.

---

## 1. Clone the Repository

Clone the repository **including submodules**:

```bash
git clone --recurse-submodules https://github.com/leoniebolt/EAR-Lab.git
cd EAR-Lab
```

---

## 2. Build and Start the Docker Environment

Navigate to the Docker directory:

```bash
cd docker
```

Build the Docker images:

```bash
docker compose build
```

Start the containers:

```bash
docker compose up
```

---

## 3. Enter the ROS 2 Container

Open a shell inside the running container:

```bash
docker exec -it ros2-slam-container bash
```

---

## 4. Build the ROS 2 Workspace

Inside the container:

```bash
colcon build
source install/setup.bash
```

---

## 5. Download the M2DGR Dataset

Download a ROS 1 bag from the **M2DGR dataset**:

```
https://github.com/SJTU-ViSYS/M2DGR
```

---

## 6. Convert ROS 1 Bag to ROS 2 (db3)

Use `rosbags` to convert the bag file and place it in the correct directory:

```bash
python3 -m rosbags.convert \
  --src <path-to-bag> \
  --dst assets/M2DGR/<bag-name>_db3/
```

This converts the bag to **ROS 2 db3 format** and stores it in:

```text
assets/M2DGR/<bag-name>_db3/
```

---

## 7. Launch LIO-SAM

Launch LIO-SAM with the corresponding M2DGR parameter file:

```bash
ros2 launch lio-sam run.launch.py \
  use_sim_time:=true \
  params_file:=/assets/params/M2DGR/params_M2DGR.yaml
```

---

## 8. Play the Converted Bag

In a separate terminal (or another container shell):

```bash
ros2 bag play assets/M2DGR/<bag-name>_db3/<bag-name>.db3 --clock
```

---

## 9. Evaluation

Evaluation scripts are located in:

```text
ros2_ws/eval/
```

### Available Scripts

- **save_map.py**  
  Listens to the `/map` topic and stores the generated map as a `.pcd` file.

- **save_global_path.py**  
  Records and stores the global trajectory/path.

- **evaluate_planarity.py**  
  Detects wall surfaces in the map and computes:
  - RMSE
  - PSNR

- **plot_psnr.py**  
  Compares PSNR results across multiple SLAM runs using the stored evaluation `.txt` files.

The evaluation folder also contains:
- All SLAM-generated maps
- Their respective evaluation outputs

---

## Notes

- Ensure `use_sim_time` is enabled when playing bags.
- Dataset-specific parameters are located in:

```text
assets/params/M2DGR/
```

- The evaluation pipeline assumes consistent map scale and frame alignment.
