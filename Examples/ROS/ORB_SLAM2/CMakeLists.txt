cmake_minimum_required(VERSION 2.8)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set (CMAKE_CXX_STANDARD 11)

rosbuild_init()

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

include(FindOpenMP)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS} -Wall  -O3 ") #-march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -Wall  -O3") # -march=native")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# Check C++11 or C++0x support
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11")
add_definitions(-DCOMPILEDWITHC11)
message(STATUS "Using flag -std=c++11.")


LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

find_package(OpenCV 3.4.1 REQUIRED
NO_MODULE 
PATHS /usr/local
NO_DEFAULT_PATH) #make sure not doesn't link to ROS OpenCV version
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

message(STATUS ${OpenCV_INCLUDE_DIRS})
message(STATUS ${CUDA_INCLUDE_DIRS})

find_package(CUDA QUIET REQUIRED)
set(CUDA_PROPAGATE_HOST_FLAGS OFF)
SET(CUDA_HOST_COMPILER /usr/bin/g++)
LIST(APPEND CUDA_NVCC_FLAGS "--compiler-options -fno-strict-aliasing -use_fast_math")


include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../../../
${PROJECT_SOURCE_DIR}/../../../include
${Pangolin_INCLUDE_DIRS}
/usr/local/lib/librealsense2.so
nvToolsExt
)

cuda_include_directories(
  ${CUDA_TOOLKIT_ROOT_DIR}/samples/common/inc
)

cuda_add_library(${PROJECT_NAME} SHARED
../../../src/System.cc
../../../src/Tracking.cc
../../../src/LocalMapping.cc
../../../src/LoopClosing.cc
../../../src/ORBextractor.cc
../../../src/ORBmatcher.cc
../../../src/FrameDrawer.cc
../../../src/Converter.cc
../../../src/MapPoint.cc
../../../src/KeyFrame.cc
../../../src/Map.cc
../../../src/MapDrawer.cc
../../../src/Optimizer.cc
../../../src/PnPsolver.cc
../../../src/Frame.cc
../../../src/KeyFrameDatabase.cc
../../../src/Sim3Solver.cc
../../../src/Initializer.cc
../../../src/Viewer.cc
../../../src/cuda/Allocator_gpu.cu
../../../src/cuda/Fast_gpu.cu
../../../src/cuda/Orb_gpu.cu
../../../src/cuda/Cuda.cu
)

set(LIBS 

${OpenCV_LIBS} 
${CUDA_LIBRARIES}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../../../build/lib/libORB_SLAM2.so
-lboost_system
realsense2
)

# Csi camera publishing 
rosbuild_add_executable(csi
src/csi_camera.cc
)

target_link_libraries(csi
${LIBS}
)

# Node for monocular camera
rosbuild_add_executable(Mono
src/ros_mono.cc
)

target_link_libraries(Mono
${LIBS}
)

# Node for stereo camera
rosbuild_add_executable(Stereo
src/ros_stereo.cc
)

target_link_libraries(Stereo
${LIBS}
)

# Node for RGB-D camera
rosbuild_add_executable(RGBD
src/ros_rgbd.cc
)

target_link_libraries(RGBD
${LIBS}
)

# real sense rgbd camera publishing 
rosbuild_add_executable(real_sense_rgbd
src/rgbd_real_sense_ros.cc
)

target_link_libraries(real_sense_rgbd
${LIBS}
)

