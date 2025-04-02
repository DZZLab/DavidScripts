#STATUS: REAL-TIME TESTING AND DEV
FROM 178669/agx_dev:dev AS apt_updates

# Install ros packages through apt
RUN apt install -y \
    ros-humble-navigation2 \
    ros-humble-robot-localization

SHELL ["/bin/bash", "-c"]
RUN --mount=type=ssh \ 
    cd /root/ros2_ws/src && git pull && git submodule sync && git submodule update --init --recursive && \
    echo 'export CXXFLAGS="-Wno-pedantic"' >> /root/.bashrc && \
    echo 'export CXXFLAGS="-Wno-dev"' >> /root/.bashrc
    # Pray to fucking god these stupid protobuf warnings go away 

# Install deps
WORKDIR /root/ros2_ws
RUN /bin/bash -c "ls src; source /opt/ros/humble/setup.sh; colcon build --symlink-install"


FROM apt_updates AS test_ben_refactor
#------------------------------------------------------------------------------
# build CAVeTalk protobufs 😭: 
SHELL ["/bin/bash", "-c"]
WORKDIR /root/ros2_ws/src
RUN --mount=type=ssh \ 
    cd rover_comms && \
    git fetch && \
    git checkout SD-334-Comms-refactor && \
    git pull origin SD-334-Comms-refactor && \
    git submodule update --init --recursive

WORKDIR /root/ros2_ws/src/rover_comms/external/CAVeTalk
SHELL ["/bin/bash", "-c"]
ENV PYENV_ROOT=/root/.pyenv
ENV PATH="$PYENV_ROOT/bin:$PYENV_ROOT/shims:$PATH"
RUN export PYENV_ROOT="/root/.pyenv" && \
    export PATH="$PYENV_ROOT/bin:$PYENV_ROOT/shims:$PATH" && \
    eval "$(pyenv init --path)" && \
    eval "$(pyenv init -)" && \
    eval "$(pyenv virtualenv-init -)" && \
    pyenv activate py311venv && \
    pip install gcovr && \
    chmod +x tools/nanopb/generate.sh && \
    ./tools/nanopb/generate.sh && \
    cd external/protobuf && \
    cmake -S . -B _build -DCMAKE_INSTALL_PREFIX=_build/protobuf-install -DCMAKE_CXX_STANDARD=20 -G Ninja -DCMAKE_BUILD_TYPE=Release -Dprotobuf_BUILD_TESTS=OFF -DABSL_PROPAGATE_CXX_STD=ON && \
    cmake --build _build --config Release && \
    cmake --build _build -t install