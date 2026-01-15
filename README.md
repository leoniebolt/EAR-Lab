# EAR-Lab – ROS 2 SLAM Evaluation Environment

This repository provides a Docker-based ROS 2 environment for running and evaluating SLAM algorithms (e.g. LIO-SAM) using datasets such as M2DGR, with tools for map extraction and quantitative evaluation.

--------------------------------------------------
1. Clone the Repository
--------------------------------------------------

Clone the repository including submodules:

git clone --recurse-submodules https://github.com/leoniebolt/EAR-Lab.git
cd EAR-Lab

--------------------------------------------------
2. Build and Start the Docker Environment
--------------------------------------------------

cd docker

docker compose build
docker compose up

--------------------------------------------------
3. Enter the ROS 2 Container
--------------------------------------------------

docker exec -it ros2-slam-container bash

--------------------------------------------------
4. Build the ROS 2 Workspace
--------------------------------------------------

colcon build
source install/setup.bash

--------------------------------------------------
5. Download the M2DGR Dataset
--------------------------------------------------

Download a ROS 1 bag from the M2DGR dataset:
https://github.com/SJTU-ViSYS/M2DGR

--------------------------------------------------
6. Convert ROS 1 Bag to ROS 2 (db3)
--------------------------------------------------

Convert the bag file and place it in the correct directory:

python3 -m rosbags.convert \
  --src <path-to-bag> \
  --dst assets/M2DGR/<bag-name>_db3/

This converts the bag to ROS 2 db3 format and stores it in:
assets/M2DGR/<bag-name>_db3/

--------------------------------------------------
7. Launch LIO-SAM
--------------------------------------------------

Launch LIO-SAM with the corresponding M2DGR parameter file:

ros2 launch lio-sam run.launch.py \
  use_sim_time:=true \
  params_file:=/assets/params/M2DGR/params_M2DGR.yaml

--------------------------------------------------
8. Play the Converted Bag
--------------------------------------------------

ros2 bag play assets/M2DGR/<bag-name>_db3/<bag-name>.db3 --clock

--------------------------------------------------
9. Evaluation
--------------------------------------------------

Evaluation scripts are located in:
ros2_ws/eval/

Available scripts:

- save_map.py
  Listens to the map topic and stores the generated map as a .pcd file.

- save_global_path.py
  Stores the global trajectory/path.

- evaluate_planarity.py
  Detects wall surfaces and computes RMSE and PSNR.

- plot_psnr.py
  Compares PSNR results across all evaluated planarity .txt files.

The evaluation folder also contains:
- All SLAM-generated maps
- Their respective evaluation results

--------------------------------------------------
Notes
--------------------------------------------------

- Make sure use_sim_time is enabled when playing bags.
- Dataset-specific parameters are located in assets/params/M2DGR/.
- Evaluation assumes consistent map scale and frame alignment.