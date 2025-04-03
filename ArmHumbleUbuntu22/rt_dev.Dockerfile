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



#------------------------------------------------------------------------------
# Replace rover_comms with Ben's refactored rover_comms... build CAVeTalk protobufs 😭: 
FROM apt_updates AS test_ben_refactor
SHELL ["/bin/bash", "-c"]
WORKDIR /root/ros2_ws/src
RUN --mount=type=ssh \ 
    rm -r rover_comms && \
    git clone -b agx_dev git@github.com:CAVEMaN-SeniorDesign/rover_comms.git && \
    cd rover_comms &&  git submodule update --init --recursive && cd external/CAVeTalk && git submodule update --init --recursive


# Build protos and CAVeTalk in one step
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

WORKDIR /root/ros2_ws/src/rover_comms/external/CAVeTalk
SHELL ["/bin/bash", "-c"]
ENV PYENV_ROOT=/root/.pyenv
ENV PATH="$PYENV_ROOT/bin:$PYENV_ROOT/shims:${PATH}"
ENV PATH="/opt/cmake3_31_6/bin:${PATH}"
RUN source /root/.bashrc && \
    cd /root/ros2_ws/src/rover_comms/external/CAVeTalk && \
    cd tools/cppcheck/cppcheck && \
    cmake -G Ninja -B build && \
    cmake --build build && \
    cd ../../.. && \
    cd tools/uncrustify/uncrustify && \
    cmake -G Ninja -DCMAKE_BUILD_TYPE=Release -B build && \
    cmake --build build --config Release && \
    cd ../../.. && \
    cmake -B build -G Ninja && \
    cmake --build build -t cppcheck && \
    cmake --build build -t uncrustify && \
    cmake --build build

# Build ROS2
WORKDIR /root/ros2_ws
RUN /bin/bash -c "ls src; source /opt/ros/humble/setup.sh; colcon build --symlink-install"