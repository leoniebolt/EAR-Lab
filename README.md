## FAST LIO ROS2

In case you later want to evaluate the trajectory of the SLAM and the map, clone this repository and switch to the branch FAST-LIO-ROS2 to get the python scripts to create a custom tum file from this SLAM.
```bash
git clone https://github.com/leoniebolt/EAR-Lab.git
cd EAR-Lab
git checkout FAST-LIO-ROS2
```

# Installation
Download FAST LIO ROS2 repository:
```bash
git clone --recursive https://github.com/Taeyoung96/FAST_LIO_ROS2.git
cd FAST_LIO_ROS2
```


# Prepare Dataset
Download dataset from M2DGR https://github.com/SJTU-ViSYS/M2DGR.git and move it into the folder FAST_LIO_ROS2.

After downloading the dataset with the rosbag and the ground truth, the rosbag needs to be converted to a ROS2 compatible bag.
For that we used rosbags.
First, install rosbags:
```bash
pip3 install rosbags
pip3 install python3-rosbag
```


Then convert the rosbag (in this case, the dataset hall_03 was used):
```bash
rosbags-convert \
  --src hall_03.bag \
  --dst hall_03
```

There should be a folder called hall_03, in there should be a hall_03.db3 and a metadata.yaml.

# Start the SLAM
## Build the container

```bash
cd docker

docker build -t fast-lio-ros2:latest .

xhost +local:docker

sudo chmod -R 777 container_run.sh

./container_run.sh fast-lio-ros2 fast-lio-ros2:latest
```

```bash
cd src/livox_ros_driver2/

./build.sh humble
 ```

## Start container

```bash
cd docker

xhost +local:docker

sudo chmod -R 777 container_run.sh

docker exec -it fast-lio-ros2 /bin/bash
```

and then inside the container

```bash
source /opt/ros/humble/setup.bash
 ```


 Open another terminal (in case you want to make a tum file and a pcd file, open three more, so you have four in total:
 ```bash
docker exec -it fast-lio-ros2 /bin/bash
```

and then inside the attached container(s):

```bash

source /opt/ros/humble/setup.bash

source install/setup.bash
```



**1st container**:
Start the SLAM process:
```bash
colcon build

source install/setup.bash

ros2 launch fast_lio mapping_m2dgr.launch.py
```

In case you want a tum file start this in your second container, since the script should be started before the rosbag gets played

**2nd container**:
Start the python script to create the tum file:
```bash
cd src

python3 save_global_path.py
```

In case you want a pcd file start this in your third container, since the script should be started before the rosbag gets played.

**3rd container**:
```bash
cd src

python3 save_map_ROS2.py
```

**4th container**:
Start the rosbag:
```bash
cd src

ros2 bag play hall_03
```

After the rosbag is done playing, the scripts can be stopped using Ctrl + C.
This saves two files, the tum and pcd file.
After recieving those files, the evaluation using evo can be done.

---

##  Acknowledgements

This project is based on the **original FAST-LIO-ROS2 implementation**.

All credit for the core algorithm goes to the **original authors**.

---

##  License

This project follows the **license of the original FAST-LIO-ROS2 repository**.
