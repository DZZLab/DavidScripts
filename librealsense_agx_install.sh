#!/bin/bash -xe

cd ~/
sudo rm -rf ./librealsense_build
mkdir librealsense_build && cd librealsense_build
echo Installing Librealsense-required dev packages
sudo apt-get install git cmake libssl-dev freeglut3-dev libusb-1.0-0-dev pkg-config libgtk-3-dev unzip -y
rm -f ./master.zip

wget https://github.com/IntelRealSense/librealsense/archive/master.zip
unzip ./master.zip -d .
cd ./librealsense-master

echo Install udev-rules
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/ 
sudo cp config/99-realsense-d4xx-mipi-dfu.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger 
mkdir build && cd build
cmake ../ -DFORCE_LIBUVC=true -DCMAKE_BUILD_TYPE=release
make -j2
sudo make install
echo -e "\e[92m\n\e[1mLibrealsense script completed.\n\e[0m"