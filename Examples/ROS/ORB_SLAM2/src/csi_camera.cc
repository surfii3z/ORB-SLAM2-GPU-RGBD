/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.

Usage: 

./csi_camera path_to_vocabulary path_to_settings WIDTH HEIGHT FPS TIME 


*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <iomanip>

#include<opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<opencv2/core/opengl.hpp>
#include<opencv2/cudacodec.hpp>

#include<ros/ros.h>
#include<tf/transform_listener.h>
#include"geometry_msgs/PoseStamped.h"

#include<System.h>
#include<Converter.h>
#include<Utils.hpp>

using namespace std;

/*
#define SET_CLOCK(t0) \
        std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();


#define TIME_DIFF(t1, t0) \
        (std::chrono::duration_cast<std::chrono::duration<double>>((t1) - (t0)).count())
*/

std::string get_tegra_pipeline(int width, int height, int fps) {
    return "nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(width) + ", height=(int)" +
           std::to_string(height) + ", format=(string)I420, framerate=(fraction)" + std::to_string(fps) +
           "/1 ! nvtee ! nvvidconv flip-method=0 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

bool operator ! (const cv::Mat &m) {return m.empty();}

//appsink

int main(int argc, char **argv)
{

    ros::init(argc, argv, "csi");
    ros::start();

    if(argc < 3)
    {
        cerr << endl << "Usage: ./csi_camera path_to_vocabulary path_to_settings" << endl;
	ros::shutdown();
        return 1;
    } else if (argc > 7) {
        cerr << endl << "Usage: ./csi_camera path_to_vocabulary path_to_settings" << endl;
        return 1;
	ros::shutdown();
    }

    int WIDTH, HEIGHT, FPS;
    double TIME; 
    if (argc > 3) WIDTH = std::atoi(argv[3]); else WIDTH = 640;
    if (argc > 4) HEIGHT = std::atoi(argv[4]); else HEIGHT = 480;
    if (argc > 5) FPS = std::atoi(argv[5]); else FPS = 30;
    if (argc > 6) TIME = std::atof(argv[6]); else TIME = 120.0;

    // Define the gstream pipeline
    std::string pipeline = get_tegra_pipeline(WIDTH, HEIGHT, FPS);
    std::cout << "Using pipeline: \n\t" << pipeline << "\n";

    // Create OpenCV capture object, ensure it works.
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        std::cout << "Connection failed";
        return -1;
    }

    ros::NodeHandle nh; 
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("orb_pose", 100);

    bool bUseViz = true;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,bUseViz);


    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;


    double tsum = 0;
    double tbuf[10] = {0.0};
    int tpos = 0;
    double trackTimeSum = 0.0;
    // Main loop
    cv::Mat im;

    //cv::cuda::GpuMat im2;
    //cv::Ptr<cv::cudacodec::VideoReader> d_reader = cv::cudacodec::createVideoReader(pipeline, cv::CAP_GSTREAMER);
    // If I want to feed a GpuMat directly to the frame 

    SET_CLOCK(t0);
    int frameNumber = 0;
    while (true) {
      cap >> im;  

      ros::Time fTime = ros::Time::now();
      //d_reader >> im2;
 
      if (im.empty()) continue;
      SET_CLOCK(t1);
      double tframe = TIME_DIFF(t1, t0);
      if (tframe > TIME) {
        break;
      }

      PUSH_RANGE("Track image", 4);
      // Pass the image to the SLAM system
      cv::Mat Tcw = SLAM.TrackMonocular(im,tframe);
      POP_RANGE;
      SET_CLOCK(t2);

      double trackTime = TIME_DIFF(t2, t1);
      trackTimeSum += trackTime;
      tsum = tframe - tbuf[tpos];
      tbuf[tpos] = tframe;
      tpos = (tpos + 1) % 10;
      //cerr << "Frame " << frameNumber << " : " << tframe << " " << trackTime << " " << 10 / tsum << "\n";
      ++frameNumber;

      // Publish the pose information to a ROS node 
      if (!Tcw == false)
      {
          geometry_msgs::PoseStamped pose;
          pose.header.stamp = fTime;
          pose.header.frame_id ="map";

          cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t(); // Rotation information
          cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3); // translation information
          vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);

          tf::Transform new_transform;
          new_transform.setOrigin(tf::Vector3(twc.at<float>(0, 0), twc.at<float>(0, 1), twc.at<float>(0, 2)));

          tf::Quaternion quaternion(q[0], q[1], q[2], q[3]);
          new_transform.setRotation(quaternion);

          tf::poseTFToMsg(new_transform, pose.pose);
          pose_pub.publish(pose);
      }
    }
    // Stop all threads
    SLAM.Shutdown();

    cerr << "Mean track time: " << trackTimeSum / frameNumber << " , mean fps: " << frameNumber / TIME << "\n";


    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

