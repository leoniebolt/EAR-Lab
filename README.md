## FAST LIO ROS2

In case you later want to evaluate the SLAM, clone this repository to get the python script to create a custom tum file from this SLAM.
```bash
git clone https://github.com/leoniebolt/EAR-Lab.git
cd EAR-Lab
```

# Installation
Follow instructions of FAST LIO ROS2 github: https://github.com/Lee-JaeWon/FAST_LIO_ROS2.git:
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
```


Then convert the rosbag (in this case, the dataset hall_03 was used):
```bash
rosbags-convert \
  --src hall_03.bag \
  --dst hall_03
```

There should be a folder called hall_03, in there should be a hall_03.db3 and a metadata.yaml.

# Start the SLAM

```bash
cd docker

docker build -t fast-lio-ros2:latest .

xhost +local:docker

sudo chmod -R 777 container_run.sh

 ./container_run.sh fast-lio-ros2 fast-lio-ros2:latest
 ```



 Open another terminal (in case you want to make a tum file, open two more, so you have 3 in total:
 ```bash
 docker exec -it fast-lio-ros2 /bin/bash

 source /opt/ros/humble/setup.bash
```


All the following should stay in
```bash
/ros2_ws
```



**1st container**:
Start the SLAM process:
```bash
cd src/livox_ros_driver2/

source install/setup.bash

ros2 launch fast_lio mapping_m2dgr.launch.py
```

In case you want a tum file start this in your second container, since the script should be started before the rosbag gets played.
**2nd container**:
Start the python script to create the tum file:
```bash
python3 save_global_path.py
```


**3rd container**:
Start the rosbag:
```bash
ros2 bag play hall_03
```
