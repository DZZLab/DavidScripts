#STATUS: REAL-TIME TESTING AND DEV
FROM 178669/sd_jetson:hi_freq_test_2 AS clean_ros2_ws

SHELL ["/bin/bash", "-c"]
WORKDIR /root
RUN rm -r ros2_ws

FROM clean_ros2_ws AS install_ros2_ws
WORKDIR /root
RUN mkdir -p -m 0700 /root/.ssh && ssh-keyscan github.com >> /root/.ssh/known_hosts
RUN --mount=type=ssh \
    mkdir -p ros2_ws/src && \
    git clone -b agx_dev git@github.com:CAVEMaN-SeniorDesign/Navigation_Unit_ROS2.git ros2_ws/src && \
    cd ros2_ws/src && git submodule update --init --recursive

RUN --mount=type=ssh \
mkdir -p ros2_dependencies && \
git clone -b foxy_ub18 git@github.com:CAVEMaN-SeniorDesign/ros2_dependencies.git ros2_dependencies/src

FROM install_ros2_ws AS system_installs
WORKDIR /root/Documents
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/foxy/install/setup.sh && \
    git clone https://github.com/RoverRobotics-forks/serial-ros2 && \
    cd serial-ros2 && \
    cmake -B build -S . -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build build && \
    cmake --install build
    

#------------------------------------------------------------------------------
# build CAVeTalk protobufs 😭: 
FROM system_installs AS cavetalk-proto-build
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

#------------------------------------------------------------------------------
# build CAVeTalk 😭: 
FROM cavetalk-proto-build AS cavetalk-build
WORKDIR /root/ros2_ws/src/rover_comms/external/CAVeTalk
SHELL ["/bin/bash", "-c"]
ENV PYENV_ROOT=/root/.pyenv
ENV PATH="$PYENV_ROOT/bin:$PYENV_ROOT/shims:${PATH}"
ENV PATH="/opt/cmake3_31_6/bin:${PATH}"
RUN which cmake && cmake --version
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


# Addition to set colcon build default params, the cmake-args can silence some stupid warnings too -> also fix tinyxml errors
RUN mkdir -p /root/.colcon && \
    cat <<EOF > /root/.colcon/defaults.yaml
build:
  symlink-install: true
  cmake-args:
    - -Wno-dev
    - -DCMAKE_POSITION_INDEPENDENT_CODE=ON
EOF

#------------------------------------------------------------------------------
# final build step
FROM cavetalk-build AS final_build
SHELL ["/bin/bash", "-c"]
RUN --mount=type=ssh \ 
    cd /root/ros2_ws/src && git pull && git submodule sync && git submodule update --init --recursive
WORKDIR /root/ros2_ws

RUN /bin/bash -c "cd /root/ros2_dependencies; ls src; source ~/.bashrc;source /opt/ros/foxy/install/setup.sh; source /root/ros2_dependencies/install/setup.bash; colcon build --symlink-install"
RUN /bin/bash -c "cd /root/ros2_ws; ls src;source ~/.bashrc; source /opt/ros/foxy/install/setup.sh; source /root/ros2_dependencies/install/setup.bash; colcon build --symlink-install"



RUN echo 'source /root/ros2_dependencies/install/setup.bash' >> /root/.bashrc


