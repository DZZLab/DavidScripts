#STATUS: REAL-TIME TESTING AND DEV
FROM 178669/agx_dev:deps AS apt_updates

# Install ros packages through apt
RUN apt install -y \
    ros-humble-navigation2 \
    ros-humble-robot_localization


SHELL ["/bin/bash", "-c"]
RUN --mount=type=ssh \ 
    cd /root/ros2_ws/src && git pull && git submodule sync && git submodule update --init --recursive && \
    echo 'export CXXFLAGS="-Wno-pedantic"' >> /root/.bashrc && \
    echo 'export CXXFLAGS="-Wno-dev"' >> /root/.bashrc
    # Pray to fucking god these stupid protobuf warnings go away 

# Install deps
WORKDIR /root/ros2_ws
RUN /bin/bash -c "ls src; source /opt/ros/humble/setup.sh; colcon build --symlink-install"