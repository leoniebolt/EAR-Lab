SET-UP:
1.https://github.com/leoniebolt/EAR-Lab.git
2.git fetch
3.cd EAR-Lab
4.git switch LiLoc
5.docker build -t liloc_image .
6.docker run -it -v <your_work_folder>:/app liloc_image //to run and mount
7. mkdir liloc_ws/src
   cd liloc_ws/src
8. git clone https://github.com/koide3/ndt_omp			//clones the GIT from the original liloc repo
   git clone https://github.com/Livox-SDK/livox_ros_driver
   git clone https://github.com/Yixin-F/LiLoc

   cd ..
   catkin_make							
   source devel/setup.bash

RUN LiLoc SLAM AND GET TUM POSES:

9. xhost +local:docker						//run on host to let the evo-container use the display
10.roslaunch  liloc run_m2dgr.launch 				//launch the slam for the m2dgr dataset

NEW TERMINAL (in the Docker and sourced)
11.python3 extract_tum.py					//starts the script for the robot poses

NEW TERMINAL (in the Docker and sourced)
12.cd m2dgr_dataset						//but the m2dgr dataset part into this folder
   rosbag play <file_name>.bag					//for replicating our results use the hall_03.bag dataset part

13. after the .bag file is finshed press Crtl+C in the terminal where the python script lays
    a .txt file with all poses for evaluation is created
