FROM 178669/agx_dev:Ubuntu22 AS apt_updates
RUN apt-get update -y && \
    apt-get upgrade -y && \
    apt-get install python3-venv -y && \
    apt-get install -y \
    git \
    ninja-build \
    build-essential \
    wget \
    bzip2 \
    zlib1g-dev \
    libffi-dev \
    libssl-dev \
    libbz2-dev \
    libreadline-dev \
    libsqlite3-dev \
    liblzma-dev \
    libncurses-dev \
    tk-dev 

#------------------------------------------------------------------------------
# Upgrade cmake, a CAVeTalk dependency CMAKE >= 3.30
WORKDIR /opt/cmake_3_31_6
RUN mkdir -p /opt/cmake3_31_6 && \
    wget "https://github.com/Kitware/CMake/releases/download/v3.31.6/cmake-3.31.6-linux-aarch64.sh" && \
    chmod +x cmake-3.31.6-linux-aarch64.sh && \
    bash cmake-3.31.6-linux-aarch64.sh --skip-license --prefix=/opt/cmake3_31_6 && \
    rm cmake-3.31.6-linux-aarch64.sh
ENV PATH="/opt/cmake3_31_6/bin:${PATH}"

#------------------------------------------------------------------------------
# Setup ros2_ws, additionally installs openCV 4.6.0 from source
FROM apt_updates AS ros2_ws-create
WORKDIR /root
RUN mkdir -p -m 0700 /root/.ssh && ssh-keyscan github.com >> /root/.ssh/known_hosts
RUN --mount=type=ssh \
    mkdir -p ros2_ws/src && \
    git clone -b Ubuntu22 git@github.com:CAVEMaN-SeniorDesign/Navigation_Unit_ROS2.git ros2_ws/src && \
    cd ros2_ws/src && git submodule sync && git submodule update --init --recursive && \
    cd /root && git clone -b debugOpenCV git@github.com:DZZLab/DavidScripts.git Scripts && \
    apt install ros-humble-xacro && \
    cd /root/Scripts && \
    chmod +x installOpenCV.sh && \
    bash installOpenCV.sh

#------------------------------------------------------------------------------
# install CAVeTalk python: 
FROM ros2_ws-create AS cavetalk-python
SHELL ["/bin/bash", "-c"]
ENV PYENV_ROOT=/root/.pyenv
ENV PATH="$PYENV_ROOT/bin:$PYENV_ROOT/shims:$PATH"
RUN curl -fsSL https://pyenv.run | bash && \
    echo 'source /opt/ros/humble/setup.sh' >> /root/.bashrc && \
    echo 'export PATH="/opt/cmake3_31_6/bin:$PATH"' >> /root/.bashrc && \
    echo '# Start in ros2_ws' >> /root/.bashrc && \
    echo 'cd /root/ros2_ws' >> /root/.bashrc && \
    echo '# Source our dev folder' >> /root/.bashrc && \
    echo 'source install/setup.bash' >> /root/.bashrc && \
    echo 'bash /root/Scripts/git_auth.sh' >> /root/.bashrc && \
    echo 'export PYENV_ROOT="$HOME/.pyenv"' >> /root/.bashrc && \
    echo 'export PATH="$PYENV_ROOT/bin:$PATH"' >> /root/.bashrc && \
    echo 'eval "$(pyenv init --path)"' >> /root/.bashrc && \
    echo 'eval "$(pyenv init -)"' >> /root/.bashrc && \
    echo 'eval "$(pyenv virtualenv-init -)"' >> /root/.bashrc && \
    echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/absl/log/libabsl_log_internal_check_op.so:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/absl/log/:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/absl/:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/absl/container/:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/absl/:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/absl/hash:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/absl/status:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/absl/strings:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/absl/algorithm:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/absl/base:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/absl/cleanup:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/absl/container:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/absl/crc:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/debugging:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/flags:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/functional:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/hash:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/log:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/memory:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/meta:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/numeric:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/profiling:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/random:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/status:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/synchronization:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/time:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/types:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/utility:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/synchronization:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/absl/synchronization/:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/absl/synchronization:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/absl/flags:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/absl/time/' >> /root/.bashrc && \
    source /root/.bashrc && pyenv install 3.11.11 --verbose && pyenv virtualenv 3.11.11 py311venv
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/absl/log/libabsl_log_internal_check_op.so:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/absl/log/:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/absl/:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/absl/container/:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/absl/:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/absl/hash:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/absl/status:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/absl/strings:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/absl/algorithm:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/absl/base:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/absl/cleanup:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/absl/container:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/absl/crc:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/debugging:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/flags:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/functional:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/hash:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/log:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/memory:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/meta:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/numeric:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/profiling:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/random:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/status:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/synchronization:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/time:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/types:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/utility:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/synchronization:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/absl/synchronization/:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/absl/synchronization:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/absl/flags:/root/ros2_ws/build/rover_comms/Jetson-Comms/CAVeTalk/external/protobuf/third_party/abseil-cpp/absl/time/

#------------------------------------------------------------------------------
# build CAVeTalk protobufs 😭: 
FROM cavetalk-python AS cavetalk-proto-build
WORKDIR /root/ros2_ws/src/rover_comms/Jetson-Comms/CAVeTalk
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
WORKDIR /root/ros2_ws/src/rover_comms/Jetson-Comms/CAVeTalk
SHELL ["/bin/bash", "-c"]
ENV PYENV_ROOT=/root/.pyenv
ENV PATH="$PYENV_ROOT/bin:$PYENV_ROOT/shims:${PATH}"
ENV PATH="/opt/cmake3_31_6/bin:${PATH}"
RUN which cmake && cmake --version
RUN source /root/.bashrc && \
    cd /root/ros2_ws/src/rover_comms/Jetson-Comms/CAVeTalk && \
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

#------------------------------------------------------------------------------
# add packages installed globally for rover_comms: 
FROM cavetalk-build AS rover_deps
WORKDIR /root/Documents
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.sh && \
    git clone https://github.com/catchorg/Catch2.git && \
    cd Catch2 && \
    cmake -Bbuild -H. -DBUILD_TESTING=OFF && \
    cmake --build build --target install && \
    cd .. && \
    git clone https://github.com/RoverRobotics-forks/serial-ros2 && \
    cd serial-ros2 && \
    cmake -B build -S . -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build build && \
    cmake --install build


#------------------------------------------------------------------------------
# pray that colcon build works fully okay...
RUN  apt-get update -y && \
    apt upgrade -y 

#------------------------------------------------------------------------------
# Install robot_localization dependency (GeographicLib)
SHELL ["/bin/bash", "-c"]
RUN --mount=type=ssh \
    cd /root/Documents && \
    git clone git@github.com:geographiclib/geographiclib.git && \
    cd geographiclib && \
    mkdir build && \
    cd build && \
    cmake .. && \
    cmake --build . && \
    make install
    
#------------------------------------------------------------------------------
# Add Pavel Dependencies
WORKDIR /root
SHELL ["/bin/bash", "-c"]
RUN --mount=type=ssh \
    cd /root/Documents && \
    git clone git@github.com:CAVEMaN-SeniorDesign/RealSense_API.git && \
    cd RealSense_API && \
    chmod +x ./tools/install_dependencies_agx.sh && \
    ./tools/install_dependencies_agx.sh

#------------------------------------------------------------------------------
# Add Pavel Dependencies
SHELL ["/bin/bash", "-c"]
RUN --mount=type=ssh \
    cd /root/ros2_ws/src && git pull && git submodule sync && git submodule update --init --recursive && \
    cd /root/ros2_ws/src

#------------------------------------------------------------------------------
# Additions to ~/.bashrc
RUN echo 'export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST' >> /root/.bashrc && \
    echo 'ROS_LOCALHOST_ONLY=1' >> /root/.bashrc

