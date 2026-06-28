# Dockerfile — Autonomous Drone ROS2 Stack
# Phase 6+ — for Jetson Orin Nano deployment
FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-cv-bridge \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /drone_ws
COPY . /drone_ws/src/autonomous_drone_ros2

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /drone_ws && colcon build --symlink-install"

CMD ["/bin/bash"]
