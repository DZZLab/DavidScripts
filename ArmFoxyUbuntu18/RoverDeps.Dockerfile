#STATUS: BUILT & TESTED 
#Combines Dockerfile.agx and Dockerfile.add_pavel

FROM dustynv/ros:foxy-desktop-l4t-r32.7.1 AS apt_updates
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
    tk-dev \
    ssh 

RUN apt-get update && \
    apt-get install -y software-properties-common wget curl gnupg && \
    add-apt-repository ppa:ubuntu-toolchain-r/test && \
    apt-get update && \
    apt-get install -y \
    gcc-9 \
    g++-9 \
    build-essential \
    cmake \
    git \
    && rm -rf /var/lib/apt/lists/*

# Set gcc-9 and g++-9 as the default
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 100 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 100

# Verify versions
RUN gcc --version && g++ --version

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
RUN rm -r ros2_ws || echo "No existing ros2_ws found"
RUN --mount=type=ssh \
    mkdir -p ros2_ws/src && \
    git clone -b agx_dev git@github.com:CAVEMaN-SeniorDesign/Navigation_Unit_ROS2.git ros2_ws/src && \
    cd ros2_ws/src && git submodule update --init --recursive && \
    cd /root && git clone -b debugOpenCV git@github.com:DZZLab/DavidScripts.git Scripts && \
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
    echo 'source /opt/ros/foxy/install/setup.sh' >> /root/.bashrc && \
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
    source /root/.bashrc && pyenv install 3.11.11 --verbose && pyenv virtualenv 3.11.11 py311venv

#------------------------------------------------------------------------------
# build CAVeTalk protobufs 😭: 
FROM cavetalk-python AS cavetalk-proto-build
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

#------------------------------------------------------------------------------
# add packages installed globally for rover_comms: 
FROM cavetalk-build AS rover_deps
WORKDIR /root/Documents
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/foxy/install/setup.sh && \
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
# Build Realsense SDK
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
    echo 'export ROS_LOCALHOST_ONLY=1' >> /root/.bashrc

# Build RS-SDK    
WORKDIR /root/Scripts
SHELL ["/bin/bash", "-c"]
RUN --mount=type=ssh \ 
    git pull && \
    chmod +x librealsense_agx_install.sh && \
    ./librealsense_agx_install.sh


FROM  rover_deps AS colcon_build
WORKDIR /root/ros2_ws

RUN --mount=type=ssh \
mkdir -p /root/ros2_dependencies && \
git clone -b foxy_ub18 git@github.com:CAVEMaN-SeniorDesign/ros2_dependencies.git /root/ros2_dependencies/src
SHELL ["/bin/bash", "-c"]
RUN --mount=type=ssh \ 
    cd /root/ros2_ws/src && git pull && git submodule sync && git submodule update --init --recursive

RUN /bin/bash -c "cd /root/ros2_dependencies && \
    ls src && \
    source /opt/ros/foxy/install/setup.sh && \
    colcon build --symlink-install"

RUN /bin/bash -c "cd /root/ros2_ws && \
    ls src && \
    source /opt/ros/foxy/install/setup.sh && \
    source /root/ros2_dependencies/install/setup.bash && \
    colcon build --symlink-install"

RUN echo 'source /root/ros2_dependencies/install/setup.bash' >> /root/.bashrc