cd ~/SLAM/ORB-SLAM2-GPU2016-final/src
sed -i 's/cv::UMat/cv::Mat/g' Converter.cc KeyFrame.cc MapDrawer.cc PnPsolver.cc
sed -i 's/cv::UMat/cv::Mat/g' KeyFrameDatabase.cc MapPoint.cc Sim3Solver.cc
sed -i 's/cv::UMat/cv::Mat/g' Frame.cc LocalMapping.cc Optimizer.cc System.cc
sed -i 's/cv::UMat/cv::Mat/g' FrameDrawer.cc LoopClosing.cc ORBextractor.cc Tracking.cc
sed -i 's/cv::UMat/cv::Mat/g' Initializer.cc Map.cc ORBmatcher.cc Viewer.cc

cd ~/SLAM/ORB-SLAM2-GPU2016-final/include
sed -i 's/cv::UMat/cv::Mat/g' Converter.h KeyFrameDatabase.h Map.h ORBVocabulary.h 
sed -i 's/cv::UMat/cv::Mat/g' Utils.hpp KeyFrame.h MapPoint.h PnPsolver.h Viewer.h
sed -i 's/cv::UMat/cv::Mat/g' FrameDrawer.h LocalMapping.h Optimizer.h Sim3Solver.h 
sed -i 's/cv::UMat/cv::Mat/g' Frame.h LoopClosing.h ORBextractor.h System.h 
sed -i 's/cv::UMat/cv::Mat/g' Initializer.h MapDrawer.h ORBmatcher.h Tracking.h

cd ~/SLAM/ORB-SLAM2-GPU2016-final/gpu 
sed -i 's/cv::UMat/cv::Mat/g' csi_camera.cc

cd ~/SLAM/ORB-SLAM2-GPU2016-final/Examples/Monocular
sed -i 's/cv::UMat/cv::Mat/g' mono_tum.cc mono_kitti.cc

cd ~/SLAM/ORB-SLAM2-GPU2016-final/Examples/RGB-D
sed -i 's/cv::UMat/cv::Mat/g' rgbd_tum.cc

cd ~/SLAM/ORB-SLAM2-GPU2016-final/Examples/ROS/ORB_SLAM2/src
sed -i 's/cv::UMat/cv::Mat/g' ros_mono.cc ros_rgbd.cc ros_stereo.cc

cd ~/SLAM/ORB-SLAM2-GPU2016-final/Examples/Stereo
sed -i 's/cv::UMat/cv::Mat/g' stereo_kitti.cc

