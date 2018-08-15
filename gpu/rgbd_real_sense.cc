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

// include OpenCV header file
#include<opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<opencv2/core/opengl.hpp>
#include<opencv2/cudacodec.hpp>

#include<System.h>
#include <Utils.hpp>

#include <librealsense2/rs.hpp>
#include <cv-helpers.hpp>

using namespace std;

/*
#define SET_CLOCK(t0) \
        std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();


#define TIME_DIFF(t1, t0) \
        (std::chrono::duration_cast<std::chrono::duration<double>>((t1) - (t0)).count())
*/



int main(int argc, char **argv)
{

    if(argc < 3)
    {
        cerr << endl << "Usage: ./csi_camera path_to_vocabulary path_to_settings" << endl;
        return 1;
    } else if (argc > 7) {
        cerr << endl << "Usage: ./csi_camera path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    int WIDTH, HEIGHT, FPS;
    double TIME; 
    if (argc > 3) WIDTH = std::atoi(argv[3]); else WIDTH = 640;  //1280
    if (argc > 4) HEIGHT = std::atoi(argv[4]); else HEIGHT = 480; //720 
    if (argc > 5) FPS = std::atoi(argv[5]); else FPS = 30;
    if (argc > 6) TIME = std::atof(argv[6]); else TIME = 30.0;

    //Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_INFRARED, 1280, 720, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);

    //cfg.enable_stream(RS2_STREAM_INFRARED, 1, WIDTH, HEIGHT, RS2_FORMAT_Y8, FPS);
    //cfg.enable_stream(RS2_STREAM_INFRARED, 2, WIDTH, HEIGHT, RS2_FORMAT_Y8, FPS);

    //Instruct pipeline to start streaming with the requested configuration
    pipe.start(cfg);

    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    rs2::frameset frames;
    for(int i = 0; i < 30; i++)
    {
        //Wait for all configured streams to produce a frame
        frames = pipe.wait_for_frames();
    }

    bool bUseViz = true;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,bUseViz);


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

      //Get each frame
      frames = pipe.wait_for_frames();
      rs2::frame ir_frame = frames.first(RS2_STREAM_INFRARED);
      rs2::frame depth_frame = frames.get_depth_frame();

      SET_CLOCK(t1);

      cv::Mat infared = frame_to_mat(ir_frame);
      cv::Mat depth = depth_frame_to_meters(pipe, depth_frame);
      // get left and right infrared frames from frameset
      //rs2::video_frame ir_frame_left = frames.get_infrared_frame(1);
      //rs2::video_frame ir_frame_right = frames.get_infrared_frame(2);

      //cv::Mat dMat_left = cv::Mat(cv::Size(WIDTH, HEIGHT), CV_8UC1, (void*)ir_frame_left.get_data());
      //cv::Mat dMat_right = cv::Mat(cv::Size(WIDTH, HEIGHT), CV_8UC1, (void*)ir_frame_right.get_data());
 
      //if (im.empty()) continue;
      
      double tframe = TIME_DIFF(t1, t0);
      if (tframe > TIME) {
        break;
      }

      PUSH_RANGE("Track image", 4);
      // Pass the image to the SLAM system
      //SLAM.TrackMonocular(dMat_left,tframe);
      //SLAM.TrackStereo(dMat_left, dMat_right, tframe);
      SLAM.TrackRGBD(infared, depth, tframe);
      POP_RANGE;
      SET_CLOCK(t2);

      double trackTime = TIME_DIFF(t2, t1);
      trackTimeSum += trackTime;
      tsum = tframe - tbuf[tpos];
      tbuf[tpos] = tframe;
      tpos = (tpos + 1) % 10;
      //cerr << "Frame " << frameNumber << " : " << tframe << " " << trackTime << " " << 10 / tsum << "\n";
      ++frameNumber;
    }

    cerr << "Mean track time: " << trackTimeSum / frameNumber << " , mean fps: " << frameNumber / TIME << "\n";

    // Stop all threads
    SLAM.Shutdown();




    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

