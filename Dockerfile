# ==============================================================================
# Modern C++20 ROS 2 Jazzy Driver for XGO Mini 2S Robot Dog
# ==============================================================================
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    ros-jazzy-geometry-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-nav-msgs \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS 2 workspace
WORKDIR /ros2_ws

# Copy package source
COPY . /ros2_ws/src/xgo_driver_cpp

# Build workspace in Release mode
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --packages-select xgo_driver_cpp \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

# Setup entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "xgo_driver_cpp", "xgo_driver.launch.py"]
