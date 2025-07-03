FROM ros:noetic-ros-core

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    DISABLE_ROS1_EOL_WARNINGS=true \
    SDK_WS=/root/livox_sdk_ws \
    LIVOX_WS=/root/livox_ros_ws \
    CATKIN_WS=/root/catkin_ws

# Set working directory
WORKDIR $CATKIN_WS/src

# Install system dependencies
RUN apt-get update && apt-get install -y \
      git \
      ros-noetic-pcl-ros \
      ros-noetic-eigen-conversions \
      ros-noetic-cv-bridge \
      ros-noetic-rviz \
    && rm -rf /var/lib/apt/lists/*

# Clone and build Livox SDK
RUN git clone https://github.com/Livox-SDK/Livox-SDK.git $SDK_WS && \
    cmake -S $SDK_WS -B $SDK_WS/build && \
    cmake --build $SDK_WS/build -- -j$(nproc) && \
    cmake --install $SDK_WS/build

# Clone and build livox_ros_driver
RUN mkdir -p $LIVOX_WS/src && \
    git clone https://github.com/Livox-SDK/livox_ros_driver.git $LIVOX_WS/src/livox_ros_driver && \
    bash -c "source /opt/ros/noetic/setup.bash && cd $LIVOX_WS && catkin_make"

# Copy FAST-LIO into src/
COPY . $CATKIN_WS/src/FAST-LIO

# Clone dependencies
RUN git clone https://github.com/xuankuzcr/rpg_vikit.git $CATKIN_WS/src/rpg_vikit && \
    git clone https://github.com/mohammad-intramotev/Sophus.git $CATKIN_WS/src/Sophus

# Build Sophus
RUN mkdir -p $CATKIN_WS/src/Sophus/build && \
    cd $CATKIN_WS/src/Sophus/build && \
    cmake .. && make && make install

# Build the entire catkin workspace
WORKDIR $CATKIN_WS
RUN bash -c "source /opt/ros/noetic/setup.bash && \
             source $LIVOX_WS/devel/setup.bash && \
             catkin_make"

# Source ROS and the workspace setup on container start
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# Default command
CMD ["tail", "-f", "/dev/null"]
