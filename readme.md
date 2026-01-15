# EVO SLAM Evaluation Setup and Usage

This guide explains how to set up and use EVO for trajectory evaluation and comparison in Docker.

---

## Prerequisites

- Docker installed and configured
- X11 display server (for visualization)
- Ground truth poses in TUM format
- SLAM trajectory estimates in TUM format

---

## Setup

### 1. Clone the EVO Repository

```bash
git clone https://github.com/MichaelGrupp/evo
```

### 2. Build the Docker Image

```bash
docker build -t evo-image -f Dockerfile.ros-kilted .
```

> **Note:** Run this command inside the cloned git repository.

### 3. Run the Docker Container

```bash
docker run -it \
    --net=host \
    --env="DISPLAY=$DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v /home/felix/evo:/work \
    evo-image \
    /bin/bash
```

> **Note:** Modify `/home/felix/evo` to mount your local work folder.

### 4. Enable GUI Display (on host machine)

```bash
xhost +local:docker
```

This allows the EVO container to use the host display for visualization.

---

## Configuration

Inside the container, configure EVO visualization settings:

```bash
evo_config set plot_trajectory_cmap viridis          # Change color to viridis colormap
evo_config set plot_seaborn_style whitegrid          # Remove blue/grey background tint
evo_config set plot_reference_linestyle --           # Change solid line to dashed for clarity
```

---

## Evaluation Commands

> **Important:** All commands must be run in the folder containing the `.txt` trajectory files.

### Trajectory Comparison and Visualization

Plot trajectories of multiple SLAM algorithms:

```bash
evo_traj tum liloc_estimate.txt FAST_LIO.txt LIO-SAM.txt LVI-SAM.txt \
    --ref ground_truth.txt \
    --plot \
    --plot_mode xy \
    --align \
    --correct_scale \
    --t_max_diff 0.5 \
    --save_plot map.png
```

### Absolute Pose Error (APE) Evaluation

Evaluate APE for each algorithm:

```bash
evo_ape tum ground_truth.txt liloc_estimate.txt -a -s --t_max_diff 0.5 --save_results liloc.zip
evo_ape tum ground_truth.txt FAST_LIO.txt -a -s --t_max_diff 0.5 --save_results fast_lio.zip
evo_ape tum ground_truth.txt LIO-SAM.txt -a -s --t_max_diff 0.5 --save_results LIO-SAM.zip
evo_ape tum ground_truth.txt LVI-SAM.txt -a -s --t_max_diff 0.5 --save_results LVI-SAM.zip
```

Compare APE results:

```bash
evo_res liloc.zip fast_lio.zip LIO-SAM.zip LVI-SAM.zip --plot
evo_res liloc.zip fast_lio.zip LIO-SAM.zip LVI-SAM.zip --plot --save_plot rmse.png
```

### Relative Pose Error (RPE) Evaluation

Evaluate RPE for each algorithm:

```bash
evo_rpe tum ground_truth.txt liloc_estimate.txt -va --save_results rpe_liloc.zip
evo_rpe tum ground_truth.txt FAST_LIO.txt -va --save_results rpe_fast_lio.zip
evo_rpe tum ground_truth.txt LIO-SAM.txt -va --save_results rpe_lio_sam.zip
evo_rpe tum ground_truth.txt LVI-SAM.txt -va --save_results rpe_lvi_sam.zip
```

Compare RPE results:

```bash
evo_res rpe_liloc.zip rpe_fast_lio.zip rpe_lio_sam.zip rpe_lvi_sam.zip -p
```

---

## Important Notes

- ✓ Always enable X11 forwarding on host before running container
- ✓ Ensure all `.txt` trajectory files are in TUM format
- ✓ Adjust `--t_max_diff` parameter based on your dataset's time synchronization accuracy
- ✓ Use `--align` flag to align trajectories before evaluation
- ✓ Use `--correct_scale` for monocular SLAM systems that may have scale drift
