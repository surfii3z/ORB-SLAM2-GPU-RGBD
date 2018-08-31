# Out of box Jetson Tx2. Installs all dependencies needed for ORB-SLAM 2
mkdir ~/SLAM
cd ~/SLAM

# Install CUDA 8.0
wget http://developer.download.nvidia.com/devzone/devcenter/mobile/jetpack_l4t/006/linux-x64/cuda-repo-l4t-8-0-local_8.0.34-1_arm64.deb
sudo dpkg -i cuda-repo-l4t-8-0-local_8.0.34-1_arm64.deb
sudo apt update
sudo apt search cuda

sudo add-apt-repository universe
sudo add-apt-repository multiverse
sudo apt-get update

sudo apt install cuda-toolkit-8.0

export PATH=/usr/local/cuda-8.0/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-8.0/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

# Install Lapack and Lablas and build
sudo apt-get install libblas-dev
sudo apt-get install liblapack-dev
sudo apt install libomp-dev

# Install Pangolin, OpenCV with CUDA, and ORB-SLAM2
sudo apt-get install libglew-dev
cd ~/SLAM
git clone https://github.com/stevenlovegrove/Pangolin.git
git clone https://github.com/jetsonhacks/buildOpenCVTX2.git
git clone https://github.com/connorsoohoo/ORB-SLAM2-GPU2016-final.git
#git clone https://github.com/jetsonhacks/buildLibrealsense2TX.git

# Need to install the latest version of Eigen (3.3.4) and build that
wget http://bitbucket.org/eigen/eigen/get/3.3.4.tar.bz2 --no-check-certificate
tar xvjf 3.3.4.tar.bz2

# Change OpenGL linking to Pangolin and RESET Tx2 to apply change
cd /usr/lib/aarch64-linux-gnu
sudo rm libGL.so
sudo ln -s /usr/lib/aarch64-linux-gnu/tegra/libGL.so libGL.so


# Install ORB SLAM 2 GPU and do NOT build


# RESET here, then run following the second one
