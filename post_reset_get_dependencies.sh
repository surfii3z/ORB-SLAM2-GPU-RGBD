cd ~/SLAM/Pangolin 
mkdir build
cd build
cmake ..
make 
sudo make install

# Install eigen 
cd ~/SLAM/eigen-eigen-5a0156e40feb
mkdir build 
cd build
cmake ..
sudo make install 

# Install OpenCV with CUDA, OpenGL 
cd ~/SLAM/buildOpenCVTX2
./buildOpenCV.sh -s ~/SLAM
cd ~/SLAM/opencv
cd build
sudo make install

# Install Real Sense camera drivers 
export PATH=/usr/local/cuda-8.0/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-8.0/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}} 
cd ~/SLAM/buildLibrealsense2TX

# Be sure to install the kernel patch first.
# Also be sure to use the included version of CMake to build 
# If this fails to build, then add the followings lines to the CMakeLists.txt of librealsense
#SET(CMAKE_CUDA_COMPILER_ENV_VAR /usr/local/cuda-8.0/bin/nvcc)
#SET(CMAKE_CUDA_COMPILER /usr/local/cuda-8.0/bin/nvcc)
#enable_language(CUDA)
# Change project(librealsense2 LANGUAGES CXX C CUDA) to just project(librealsense2 LANGUAGES CXX C)
# Also be sure you can find nvcc by calling nvcc --v
./installLibrealsense.sh


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


