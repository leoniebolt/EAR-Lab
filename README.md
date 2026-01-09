### FAST LIO ROS2

## Installation
Follow instructions of FAST LIO ROS2 github:
'''bash
https://github.com/Lee-JaeWon/FAST_LIO_ROS2.git
'''

```bash
git clone --recursive https://github.com/Taeyoung96/FAST_LIO_ROS2.git

cd docker

docker build -t fast-lio-ros2:latest .

xhost +local:docker

sudo chmod -R 777 container_run.sh

 ./container_run.sh fast-lio-ros2 fast-lio-ros2:latest
 ```


 Open a two more terminals (so you have in total 3):
 '''bash
 docker exec -it fast-lio-ros2 /bin/bash

 source /opt/ros/humble/setup.bash
'''

All should stay in
'''bash
~/ros2_ws
'''

**1st container**:
'''bash
cd src/livox_ros_driver2/

source install/setup.bash

ros2 launch fast_lio mapping_m2dgr.launch.py
'''


**2nd container**:
'''bash
ros2 bag play hall_03
'''


