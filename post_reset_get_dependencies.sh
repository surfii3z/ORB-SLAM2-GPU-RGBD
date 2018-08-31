cd ~/SLAM/Pangolin
mkdir build
cd build
cmake ..
make -j8
sudo make install

# Install eigen
cd ~/SLAM/eigen-eigen-5a0156e40feb
mkdir build
cd build
cmake ..
make -j8
sudo make install

# Install OpenCV 3.4.1 with CUDA, OpenGL
# Might need to reinstall to 
sudo apt-get install gstreamer1.0-tools gstreamer1.0-alsa gstreamer1.0-plugins-base
cd ~/SLAM/buildOpenCVTX2
# Make sure that git fast forwarding works correctly, no password entry needed
git config user.name "Connor Soohoo"
git config user.email "connorsoohoo@gmail.com"
./buildOpenCV.sh -s ~/SLAM
cd ~/SLAM/opencv
cd build
make -j8
sudo make install

# Install gstreamer
sudo apt-get install gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly
sudo apt-get install gstreamer1.0-libav libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
sudo apt get install libgstreamer-plugins-good1.0-dev libgstreamer-plugins-bad1.0-dev
gst-inspect-1.0 --version

# Install Real Sense camera drivers
export PATH=/usr/local/cuda-8.0/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-8.0/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
#cd ~/SLAM/buildLibrealsense2TX
#Assumes you've transferred the librealsense files to the tx2
cd ~/Desktop/tx2files/buildLibrealsense2TX

# Be sure to install the kernel patch first.
# Also be sure to use the included version of CMake to build
# If this fails to build, then add the followings lines to the CMakeLists.txt of librealsense
#SET(CMAKE_CUDA_COMPILER_ENV_VAR /usr/local/cuda-8.0/bin/nvcc)
#SET(CMAKE_CUDA_COMPILER /usr/local/cuda-8.0/bin/nvcc)
#enable_language(CUDA)
# Change project(librealsense2 LANGUAGES CXX C CUDA) to just project(librealsense2 LANGUAGES CXX C)
# Also be sure you can find nvcc by calling nvcc --v
cd scripts
chmod +x ./installDependencies.sh
chmod +x ./buildCMake.sh
cd ..
chmod +x ./installLibrealsense.sh
sudo ./installLibrealsense.sh

sudo ln -s /usr/local/cuda-8.0/lib64/libnvToolsExt.so /usr/lib/libnvToolsExt.so
cd ~/SLAM/ORB-SLAM2-GPU2016-final
chmod +x build.sh
sudo ./build.sh

## For max performance settings (now both cores 1 and 2 are working with erika)
cd ~/
sudo ./jetson_clocks
sudo echo 1 > /sys/devices/system/cpu/cpu1/online
sudo echo 1 > /sys/devices/system/cpu/cpu2/online

# Turn on jailhouse (if installed)
# cd ~/Desktop
# sudo ./startJailhouseApp.sh
