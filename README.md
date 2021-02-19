## Feature note
- [x] ROS publish odometry message:  [raulmur/ORB_SLAM2/PR#692](https://github.com/raulmur/ORB_SLAM2/pull/692)
- [x] ROS build with catkin build: [raulmur/ORB_SLAM2/PR#2](https://github.com/raulmur/ORB_SLAM2/pull/2)
- [x] GPU accelerated: [connorsoohoo/ORB-SLAM2-GPU-RGBD](https://github.com/connorsoohoo/ORB-SLAM2-GPU-RGBD)
- [x] Binary version of vocab:  [raulmur/ORB_SLAM2/PR#21](https://github.com/raulmur/ORB_SLAM2/pull/21)
- [x] Pause/ Resume mapping: [raulmur/ORB_SLAM2/PR#587](https://github.com/raulmur/ORB_SLAM2/pull/587)
- [x] Save/ load map: [raulmur/ORB_SLAM2/PR#381](https://github.com/raulmur/ORB_SLAM2/pull/381)


## Installation
1) Install the prerequisites from [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2) original repo.
2) Install dependencies according to `get_dependencies.sh` and `post_reset_get_dependencies.sh`
```bash
# dependencies according to get_dependencies.sh, might not need all, IDK.

# Install Lapack and Lablas and build
sudo apt install libblas-dev
sudo apt install liblapack-dev
sudo apt install libomp-dev
sudo apt-get install libglew-dev


sudo apt-get install gstreamer1.0-tools gstreamer1.0-alsa gstreamer1.0-plugins-base

# Install gstreamer
sudo apt-get install gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly
sudo apt-get install gstreamer1.0-libav libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
sudo apt get install libgstreamer-plugins-good1.0-dev libgstreamer-plugins-bad1.0-dev

```
3) Build using `build.sh`
```bash
cd /path/to/orb_slam2_gpu
sh build.sh
```
4) Build the binary version of vocaburary.
```
./tools/bin_vocabulary
```
5) Install cuda-enable OpenCV. Tested with `OpenCV 3.4.11` and `CUDA 10.1`
    - [Installing Multiple CUDA & cuDNN Versions in Ubuntu](https://towardsdatascience.com/installing-multiple-cuda-cudnn-versions-in-ubuntu-fcb6aa5194e2)
    - [Specify custom build OpenCV version with ROS project](https://answers.ros.org/question/242376/having-trouble-using-cuda-enabled-opencv-with-kinetic/)
```cmake
# In summary, in CMakeList.txt just change from  find_package(OpenCV) to the following

find_package(OpenCV REQUIRED
    NO_MODULE # should be optional, tells CMake to use config mode
    PATHS /usr/local # look here
    NO_DEFAULT_PATH) # and don't look anywhere else
```
6) Compile vision_opencv from source. This is because the default version of OpenCV that comes with Ubuntu is not CUDA-enabled. So we need to install CUDA-enabled version of OpenCV (in the previous step) and then recompile vision_opencv with the new version of the OpenCV. Otherwise, catkin workspace will craw in the default version of OpenCV causing catkin build error.
```bash
 cd /path/to/ROS/src
 git clone https://github.com/ros-perception/vision_opencv
 cd vision_opencv
 git checkout melodic
 # change CMakeList.txt as in 1)
 cd /path/to/ROS
 catkin build
```
-------
# This part is copied from [ Alkaid-Benetnash/ORB_SLAM2](https://github.com/Alkaid-Benetnash/ORB_SLAM2/blob/map_save_load_and_bin_voc/README.md)
## 9. SLAM and Localization Modes
You can change between the *SLAM* and *Localization mode* using the GUI of the map viewer.

### SLAM Mode
This is the default mode. The system runs in parallal three threads: Tracking, Local Mapping and Loop Closing. The system localizes the camera, builds new map and tries to close loops.

### Localization Mode
This mode can be used when you have a good map of your working area. In this mode the Local Mapping and Loop Closing are deactivated. The system localizes the camera in the map (which is no longer updated), using relocalization if needed. 

## 10. Binary Format ORB Vocabulary

You can load ORB vocabulary in either text or binary format. The format is determined by suffix(.txt for text format and .bin for binary format).

`build.sh` will generate a text-to-binary convertor `bin_vocabulary` in `Vocabulary/` . You can also find it as a target in `CMakeLists.txt`.

`bin_vocabulary` will convert `./ORBvoc.txt` to `./ORBvoc.bin` and you can use the new `ORBvoc.bin` as  `PATH_TO_VOCABULARY`  wherever needed.

PS: binary format is loaded faster and text format is more human-readable.

## 11. Map Save/Load

#### Enable:

Considering this feature doesn't hurt performance, and it is annonying to deal with conditional compilation flags, so this feature will be enabled unconditionally.

#### Usage:

This feature is integrated with `class System`. The path of mapfile can be set by adding `Map.mapfile: map.bin` to ORB_SLAM2's settings file. See the last few line of `Example/Monocular/TUM1.xml`.

To save a map, you need construct `ORB_SLAM2::System` with the last parameter be `true`. Then the `System` will save map to mapfile specified in setting file when `ShutDown`.

With a readable mapfile, map will be loaded automatically and `System` will run in localization mode, but you can change it to SLAM mode later.

If you set a mapfile but it doesn't exist, `System` will create new map.

mono_tum has been updated as a simple example of this functionality. An extra command line parameter(0 or 1) should be given to indicate whether you want to save map or not.

#### Implementation related:

I use boost_serialization library to serialize `Map`, `MapPoint`, `KeyFrame`,`KeyFrameDatabase`, `cv::Mat`, `DBoW2::BowVector`, `DBoW2::FeatureVector`. In brief, only the `ORBVector` isn't serialized.

This feature is tested with boost 1.64 and it works fine mostly. There is still some occasional segmentfault to dig in.

-------
# Original Readme from connorsoohoo
# ORB-SLAM2-GPU-RGBD
This is an optimized version of the ORB SLAM2 library and yunchih's monocular GPU acceleration to include RGB-D vision.
This optimization runs in real time on the Jetson TX2. At max clock rate we achieve around **18-20 fps** on average.

## Usage

We use the Intel RealSense D435 camera to supply RGB-D vision. Refer to the installation on how to install the camera drivers and library onto the TX2.

```
./build/rgbd_real_sense Vocabulary/ORBvoc.txt RealSense_GPU/rgbd_real_sense.yaml
```

There are also files to run monocular and stereo vision.

```
./build/mono_real_sense Vocabulary/ORBvoc.txt RealSense_GPU/mono_real_sense.yaml
```
```
./build/stereo_real_sense Vocabulary/ORBvoc.txt RealSense_GPU/stereo_real_sense.yaml
```


To enable verbose commands, like clocking times of bottleneck hotspots or printing velocities, check the Utils.hpp file in include for the verbose flag.

```c++
//Change from 0 to 1 to enable clock timing of various functions
#define UTIL_VERBOSE 0
```


## Installation

Run the following two scripts to install the dependencies.

```
chmod +x get_dependencies.sh
./get_dependencies.sh
chmod +x post_reset_get_dependencies.sh
./post_reset_get_dependencies.sh
```

The main issue with the install is that apt says that it is unable to lock the administration directory.
This usually happens if 'Update Manager' is running in parallel for any update check or install as the update manager process places its own lock.
The best course of action is just to wait a couple minutes and then try again, and then reboot and try again. consult one of these [solutions](https://askubuntu.com/questions/15433/unable-to-lock-the-administration-directory-var-lib-dpkg-is-another-process).

These scripts install the dependencies successfully with the exception of librealsense, whose installation depends on the environment. Consult the JetsonHacks repo for more thorough installation details of the Real Sense library and camera drivers.
Folks in the AMBER Lab only need to use the buildLibrealsense2TX library in the Dropbox, which this script defaults to and applies the necessary patches to the kernel in the previously run install script. Otherwise, switch the comments in the script when installing the librealsense library and drivers.
Note you will have to patch the kernel to be able to run the RealSense camera on the TX2.
Moreover, the Real Sense cameras prefer a USB 3.1 connection to supply steady RGB-D video, so be sure the connections use this type of connector.

-------
## ORB-SLAM2-GPU
This is a fork of Raul Mur-Artal's [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2), on which we rewrite hot paths with CUDA. Our optimization enables us to run the algorithm in **real time** on a Nvidia's Jetson TX1.

- [Project presentation website](http://yunchih.github.io/ORB-SLAM2-GPU2016-final/)
- [Demo video on youtube](https://www.youtube.com/watch?v=p77hLLRfBGQ)

**Following is from the original README of ORB-SLAM2**

## Introduction

ORB-SLAM2 is a real-time SLAM library for **Monocular**, **Stereo** and **RGB-D** cameras that computes the camera trajectory and a sparse 3D reconstruction (in the stereo and RGB-D case with true scale). It is able to detect loops and relocalize the camera in real time. We provide examples to run the SLAM system in the [KITTI dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) as stereo or monocular, and in the [TUM dataset](http://vision.in.tum.de/data/datasets/rgbd-dataset) as RGB-D or monocular. We also provide a ROS node to process live monocular or RGB-D streams. **The library can be compiled without ROS**. ORB-SLAM2 provides a GUI to change between a *SLAM Mode* and *Localization Mode*, see section 9 of this document.

#####Videos showing ORB-SLAM2:
<a href="http://www.youtube.com/watch?feature=player_embedded&v=dF7_I2Lin54
" target="_blank"><img src="http://img.youtube.com/vi/dF7_I2Lin54/0.jpg"
alt="Tsukuba Dataset" width="240" height="180" border="10" /></a>
<a href="http://www.youtube.com/watch?feature=player_embedded&v=51NQvg5n-FE
" target="_blank"><img src="http://img.youtube.com/vi/51NQvg5n-FE/0.jpg"
alt="KITTI Dataset" width="240" height="180" border="10" /></a>
<a href="http://www.youtube.com/watch?feature=player_embedded&v=LnbAI-o7YHk
" target="_blank"><img src="http://img.youtube.com/vi/LnbAI-o7YHk/0.jpg"
alt="TUM RGBD Dataset" width="240" height="180" border="10" /></a>
<a href="http://www.youtube.com/watch?feature=player_embedded&v=MUyNOEICrf8
" target="_blank"><img src="http://img.youtube.com/vi/MUyNOEICrf8/0.jpg"
alt="EuRoC Dataset (V1_02, V1_03)" width="240" height="180" border="10" /></a>
<a href="http://www.youtube.com/watch?feature=player_embedded&v=xXt90wZejwk
" target="_blank"><img src="http://img.youtube.com/vi/xXt90wZejwk/0.jpg"
alt="EuRoC Dataset (V1_02, V1_03)" width="240" height="180" border="10" /></a>

**Notice for ORB-SLAM Monocular users:**
The monocular capabilities of ORB-SLAM2 compared to [ORB-SLAM Monocular](https://github.com/raulmur/ORB_SLAM) are similar. However in ORB-SLAM2 we apply a full bundle adjustment after a loop closure, the extraction of ORB is slightly different (trying to improve the dispersion on the image) and the tracking is also slightly faster. The GUI of ORB-SLAM2 also provides you new capabilities as the *modes* mentioned above and a reset button. We recommend you to try this new software :)

###Related Publications:

[1] Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. **ORB-SLAM: A Versatile and Accurate Monocular SLAM System**. *IEEE Transactions on Robotics,* vol. 31, no. 5, pp. 1147-1163, 2015. (2015 IEEE Transactions on Robotics **Best Paper Award**). **[PDF](http://webdiis.unizar.es/~raulmur/MurMontielTardosTRO15.pdf)**.

[2] Dorian Gálvez-López and Juan D. Tardós. **Bags of Binary Words for Fast Place Recognition in Image Sequences**. *IEEE Transactions on Robotics,* vol. 28, no. 5, pp.  1188-1197, 2012. **[PDF](http://doriangalvez.com/php/dl.php?dlp=GalvezTRO12.pdf)**

#1. License

ORB-SLAM2 is released under a [GPLv3 license](https://github.com/raulmur/ORB_SLAM2/blob/master/License-gpl.txt). For a list of all code/library dependencies (and associated licenses), please see [Dependencies.md](https://github.com/raulmur/ORB_SLAM2/blob/master/Dependencies.md).

For a closed-source version of ORB-SLAM2 for commercial purposes, please contact the authors: orbslam (at) unizar (dot) es.

If you use ORB-SLAM2 in an academic work, please cite:

    @article{murTRO2015,
      title={{ORB-SLAM}: a Versatile and Accurate Monocular {SLAM} System},
      author={Mur-Artal, Ra\'ul, Montiel, J. M. M. and Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics},
      volume={31},
      number={5},
      pages={1147--1163},
      doi = {10.1109/TRO.2015.2463671},
      year={2015}
     }

#2. Prerequisites
We have tested the library in **Ubuntu 12.04** and **14.04**, but it should be easy to compile in other platforms. A powerful computer (e.g. i7) will ensure real-time performance and provide more stable and accurate results.

## C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 2.4.3. Tested with OpenCV 2.4.11**.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## BLAS and LAPACK
[BLAS](http://www.netlib.org/blas) and [LAPACK](http://www.netlib.org/lapack) libraries are requiered by g2o (see below). On ubuntu:
```
sudo apt-get install libblas-dev
sudo apt-get install liblapack-dev
```

## DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

## ROS (optional)
We provide some examples to process the live input of a monocular, stereo or RGB-D camera using [ROS](ros.org). Building these examples is optional. In case you want to use ROS, a version Hydro or newer is needed.

#3. Building ORB-SLAM2 library and TUM/KITTI examples

Clone the repository:
```
git clone https://github.com/raulmur/ORB_SLAM2.git ORB_SLAM2
```

We provide a script `build.sh` to build the *Thirdparty* libraries and *ORB-SLAM2*. Please make sure you have installed all required dependencies (see section 2). Execute:
```
cd ORB_SLAM2
chmod +x build.sh
./build.sh
```

This will create **libORB_SLAM2.so**  at *lib* folder and the executables **mono_tum**, **mono_kitti**, **rgbd_tum**, **stereo_kitti** in *Examples* folder.

#4. Monocular Examples

## TUM Dataset

1. Download a sequence from http://vision.in.tum.de/data/datasets/rgbd-dataset/download and uncompress it.

2. Execute the following command. Change `TUMX.yaml` to TUM1.yaml,TUM2.yaml or TUM3.yaml for freiburg1, freiburg2 and freiburg3 sequences respectively. Change `PATH_TO_SEQUENCE_FOLDER`to the uncompressed sequence folder.
```
./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUMX.yaml PATH_TO_SEQUENCE_FOLDER
```

## KITTI Dataset  

1. Download the dataset (grayscale images) from http://www.cvlibs.net/datasets/kitti/eval_odometry.php

2. Execute the following command. Change `KITTIX.yaml`by KITTI00-02.yaml, KITTI03.yaml or KITTI04-12.yaml for sequence 0 to 2, 3, and 4 to 12 respectively. Change `PATH_TO_DATASET_FOLDER` to the uncompressed dataset folder. Change `SEQUENCE_NUMBER` to 00, 01, 02,.., 11.
```
./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER
```

#5. Stereo Example

## KITTI Dataset

1. Download the dataset (grayscale images) from http://www.cvlibs.net/datasets/kitti/eval_odometry.php

2. Execute the following command. Change `KITTIX.yaml`to KITTI00-02.yaml, KITTI03.yaml or KITTI04-12.yaml for sequence 0 to 2, 3, and 4 to 12 respectively. Change `PATH_TO_DATASET_FOLDER` to the uncompressed dataset folder. Change `SEQUENCE_NUMBER` to 00, 01, 02,.., 11.
```
./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt Examples/Stereo/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER
```

#6. RGB-D Example

## TUM Dataset

1. Download a sequence from http://vision.in.tum.de/data/datasets/rgbd-dataset/download and uncompress it.

2. Associate RGB images and depth images using the python script [associate.py](http://vision.in.tum.de/data/datasets/rgbd-dataset/tools). We already provide associations for some of the sequences in *Examples/RGB-D/associations/*. You can generate your own associations file executing:

  ```
  python associate.py PATH_TO_SEQUENCE/rgb.txt PATH_TO_SEQUENCE/depth.txt > associations.txt
  ```

3. Execute the following command. Change `TUMX.yaml` to TUM1.yaml,TUM2.yaml or TUM3.yaml for freiburg1, freiburg2 and freiburg3 sequences respectively. Change `PATH_TO_SEQUENCE_FOLDER`to the uncompressed sequence folder. Change `ASSOCIATIONS_FILE` to the path to the corresponding associations file.

  ```
  ./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUMX.yaml PATH_TO_SEQUENCE_FOLDER ASSOCIATIONS_FILE
  ```

#7. ROS Examples

### Building the nodes for mono, stereo and RGB-D
1. Add the path including *Examples/ROS/ORB_SLAM2* to the ROS_PACKAGE_PATH environment variable. Open .bashrc file and add at the end the following line. Replace PATH by the folder where you cloned ORB_SLAM2:

  ```
  export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM2/Examples/ROS
  ```

2. Go to *Examples/ROS/ORB_SLAM2* folder and execute:

  ```
  mkdir build
  cd build
  cmake .. -DROS_BUILD_TYPE=Release
  make -j
  ```

### Running Monocular Node
For a monocular input from topic `/camera/image_raw` run node ORB_SLAM2/Mono. You will need to provide the vocabulary file and a settings file. See the monocular examples above.

  ```
  rosrun ORB_SLAM2 Mono PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```

### Running Stereo Node
For a stereo input from topic `/camera/left/image_raw` and `/camera/right/image_raw` run node ORB_SLAM2/Stereo. You will need to provide the vocabulary file and a settings file. If you **provide rectification matrices** (see Examples/Stereo/EuRoC.yaml example), the node will recitify the images online, **otherwise images must be pre-rectified**.

  ```
  rosrun ORB_SLAM2 Stereo PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE ONLINE_RECTIFICATION
  ```

**Example**: Download a rosbag (e.g. V1_01_easy.bag) from the EuRoC dataset (http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). Open 3 tabs on the terminal and run the following command at each tab:
  ```
  roscore
  ```

  ```
  rosrun ORB_SLAM2 Stereo Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml true
  ```

  ```
  rosbag play --pause V1_01_easy.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw
  ```

Once ORB-SLAM2 has loaded the vocabulary, press space in the rosbag tab. Enjoy!. Note: a powerful computer is required to run the most exigent sequences of this dataset.

### Running RGB_D Node
For an RGB-D input from topics `/camera/rgb/image_raw` and `/camera/depth_registered/image_raw`, run node ORB_SLAM2/RGBD. You will need to provide the vocabulary file and a settings file. See the RGB-D example above.

  ```
  rosrun ORB_SLAM2 RGBD PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```

#8. Processing your own sequences
You will need to create a settings file with the calibration of your camera. See the settings file provided for the TUM and KITTI datasets for monocular, stereo and RGB-D cameras. We use the calibration model of OpenCV. See the examples to learn how to create a program that makes use of the ORB-SLAM2 library and how to pass images to the SLAM system. Stereo input must be synchronized and rectified. RGB-D input must be synchronized and depth registered.

#9. SLAM and Localization Modes
You can change between the *SLAM* and *Localization mode* using the GUI of the map viewer.

### SLAM Mode
This is the default mode. The system runs in parallal three threads: Tracking, Local Mapping and Loop Closing. The system localizes the camera, builds new map and tries to close loops.

### Localization Mode
This mode can be used when you have a good map of your working area. In this mode the Local Mapping and Loop Closing are deactivated. The system localizes the camera in the map (which is no longer updated), using relocalization if needed.
