#STATUS: REAL-TIME TESTING AND DEV
FROM 178669/agx_dev:dev AS apt_updates

SHELL ["/bin/bash", "-c"]
RUN --mount=type=ssh \ 
    cd /root/ros2_ws/src && git pull && git submodule sync && git submodule update --init --recursive
WORKDIR /root/ros2_ws
RUN /bin/bash -c "ls src; source /opt/ros/humble/setup.sh; colcon build --symlink-install"


# Addition to set colcon build default params, the cmake-args can silence some stupid warnings too 
RUN mkdir -p /root/.colcon && \
    cat <<EOF > /root/.colcon/defaults.yaml
build:
  symlink-install: true
  cmake-args:
    - -Wno-dev
EOF
