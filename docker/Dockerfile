# Base image with full desktop and Gazebo support
FROM osrf/ros:humble-desktop-full

# Avoid interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# Set up CUDA & GPU environment variables (safe for CPU-only too)
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility
ENV NVIDIA_REQUIRE_CUDA="cuda>=11.3"
ENV PATH=/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}

# Set TurtleBot3 model
ENV TURTLEBOT3_MODEL=burger

# Create a non-root user
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME && \
    useradd -m -s /bin/bash --uid $USER_UID --gid $USER_GID $USERNAME && \
    apt update && apt install -y sudo && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Switch to non-root user
USER $USERNAME
WORKDIR /home/$USERNAME

# ============================================================
# === BEGIN: Bereich für zusätzliche Ubuntu/ROS-Packages   ===
# Hier kannst du weitere ROS oder Ubuntu Pakete hinzufügen.
# Beispiel:
# ros-humble-mein-extra-paket \
# sudo apt install -y <dein-paket>
# === ENDE: Bereich für zusätzliche Ubuntu/ROS-Packages    ===
# ============================================================

# Install ROS dev tools, TurtleBot3, Gazebo, MoveIt, and visualization tools
#     ros-humble-robot-state-publisher \ ros-humble-joint-state-publisher \ added but still not build with it
RUN sudo apt update && sudo apt install -y software-properties-common && \
    sudo add-apt-repository ppa:borglab/gtsam-release-4.1 -y && \
    sudo apt update && sudo apt install -y \
    git curl gdb gdbserver python3-pip \
    python3-colcon-common-extensions \
    # --- Dependencies from your requirements ---
    ros-humble-perception-pcl \
    ros-humble-pcl-msgs \
    ros-humble-vision-opencv \
    ros-humble-xacro \
    libgtsam-dev \
    libgtsam-unstable-dev \
    ros-humble-ouster-ros \
    ros-humble-velodyne \
    # ------------------------------------------
    ros-humble-rviz2 \
    ros-humble-robot-state-publisher \ 
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-turtlebot3 \
    ros-humble-slam-toolbox \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-bringup \
    ros-humble-turtlebot3-teleop \
    ros-humble-turtlebot3-navigation2 \
    ros-humble-moveit \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-diff-drive-controller \
    ros-humble-joint-trajectory-controller \
    # Note: I removed ros-humble-gtsam to avoid conflicts with the PPA version
    libtbb-dev \
    libpcl-dev \
    && sudo apt autoremove -y && sudo apt clean

# ============================================================
# === BEGIN: Bereich für zusätzliche Python-Pakete         ===
# Hier kannst du weitere Python Pakete per pip installieren.
# Beispiel:
# pip install <dein-python-paket>
# === ENDE: Bereich für zusätzliche Python-Pakete          ===
# ============================================================

# Python and ROS tools
RUN pip install --upgrade pip && \
    pip install colcon-clean numpy rosbags 
    # === BEGIN: Eigene Python-Pakete ===
    # Hier weitere pip Pakete einfügen, z.B.:
    # numpy \
    # matplotlib 
    # === END: Eigene Python-Pakete ===
    
# Setup environment sourcing
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Create ROS 2 workspace and setup sourcing
RUN mkdir -p /home/$USERNAME/ros2_ws/src && \
    echo 'if [ -f /home/'$USERNAME'/ros2_ws/install/setup.bash ]; then source /home/'$USERNAME'/ros2_ws/install/setup.bash; fi' >> /home/$USERNAME/.bashrc




# Set workspace as default dir
WORKDIR /home/$USERNAME/ros2_ws

# Default shell
CMD ["/bin/bash"]
