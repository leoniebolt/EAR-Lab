# Start with ROS Noetic (Ubuntu 20.04)
FROM ros:noetic-robot

# Prevent interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-pip build-essential cmake git wget \
    libpcl-dev libeigen3-dev libopencv-dev libusb-1.0-0-dev \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-tf2-sensor-msgs \
    ros-noetic-gtsam \
    ros-noetic-ros-environment \
    && rm -rf /var/lib/apt/lists/*

# 2. Install GTSAM (Required to be built from source for specific versions)
RUN git clone https://github.com/borglab/gtsam.git /tmp/gtsam \
    && cd /tmp/gtsam \
    && git checkout 4.2a1 \
    && mkdir build && cd build \
    && cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF .. \
    && make install -j$(nproc) \
    && rm -rf /tmp/gtsam

# 3. Handle Python requirements
WORKDIR /app
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

# Set up workspace
COPY . .

# Source ROS automatically on entry
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

CMD ["bash"]
