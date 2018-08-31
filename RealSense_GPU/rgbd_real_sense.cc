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

RGB-D file 
Usage: 
./build/rgbd_real_sense Vocabulary/ORBvoc.txt RealSense_GPU/rgbd_real_sense.yaml 


*/


#include<iostream>
#include <sstream>
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
#include<Converter.h>
#include<SGFilter.h>

#include <librealsense2/rs.hpp>
#include <cv-helpers.hpp>

#include <stdio.h>
#include <stdlib.h> 
#include <signal.h> 

#define POLY_ORDER 2
#define FILTER_SIZE 5

using namespace std;

float get_depth_scale(rs2::device dev);
rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);
bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);

bool operator ! (const cv::Mat &m) {return m.empty();}

volatile sig_atomic_t flag = 0;
void on_stop(int sig) {
    flag = 1;
}

int main(int argc, char **argv)
{
    signal(SIGINT, on_stop);
    if(argc < 3)
    {
        cerr << endl << "Usage: ./csi_camera path_to_vocabulary path_to_settings" << endl;
        return 1;
    } else if (argc > 8) {
        cerr << endl << "Usage: ./csi_camera path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    int WIDTH, HEIGHT, FPS;
    double TIME; 
    bool bUseViz = true;
    if (argc > 3) {
	std::stringstream ss(argv[3]);
	bool b; 
	if (ss >> std::boolalpha >> b) {
	    bUseViz = b;
	}
    }
    if (argc > 4) WIDTH = std::atoi(argv[4]); else WIDTH = 640;  //1280
    if (argc > 5) HEIGHT = std::atoi(argv[5]); else HEIGHT = 480; //720 
    if (argc > 6) FPS = std::atoi(argv[6]); else FPS = 30;
    if (argc > 7) TIME = std::atof(argv[7]); else TIME = 30000.0;

    //Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_RGB8, 30); //RS2_FORMAT_Y8
    cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, 30);

    // Start the camera pipeline 
    rs2::pipeline_profile selection = pipe.start(cfg);

    // Each depth camera might have different units for depth pixels, so we get it here
    // Using the pipeline's profile, we can retrieve the device that the pipeline uses
    float depth_scale = get_depth_scale(selection.get_device());

    //Pipeline could choose a device that does not have a color stream
    //If there is no color stream, choose to align depth to another stream
    rs2_stream align_to = find_stream_to_align(selection.get_streams());

    // Create a rs2::align object.
    // rs2::align allows us to perform alignment of depth frames to others frames
    //The "align_to" is the stream type to which we plan to align depth frames.
    rs2::align align(align_to);

    // Get camera intrinsics
    auto depth_stream = selection.get_stream(RS2_STREAM_COLOR)
                             .as<rs2::video_stream_profile>();
    auto resolution = std::make_pair(depth_stream.width(), depth_stream.height());
    auto i = depth_stream.get_intrinsics();
    auto principal_point = std::make_pair(i.ppx, i.ppy);
    auto focal_length = std::make_pair(i.fx, i.fy);

    //std::cout << "Width: " << resolution[0] << "Height: " << resolution[1] << std::endl;
    std::cout << "ppx: " << i.ppx << " ppy: " << i.ppy << std::endl;
    std::cout << "fx: " << i.fx << " fy: " << i.fy << std::endl;
    std::cout << "k1: " << i.coeffs[0] << " k2: " << i.coeffs[1] << " p1: " << i.coeffs[2] << " p2: " << i.coeffs[3] << " k3: " << i.coeffs[4] << std::endl;

    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    rs2::frameset frames;
    for(int i = 0; i < 30; i++)
    {
        //Wait for all configured streams to produce a frame
        frames = pipe.wait_for_frames();

	//If the profile was changed, update the align object, and also get the new device's depth scale
        if (profile_changed(pipe.get_active_profile().get_streams(), selection.get_streams()))
        {
            selection = pipe.get_active_profile();
            align_to = find_stream_to_align(selection.get_streams());
            align = rs2::align(align_to);
            depth_scale = get_depth_scale(selection.get_device());
        }
      

        //Get processed aligned frame
        auto processed = align.process(frames);
    }

    // Initialize Savitzky-Golay Filters
    ORB_SLAM2::SGFilter xFilter(POLY_ORDER, FILTER_SIZE);
    ORB_SLAM2::SGFilter yFilter(POLY_ORDER, FILTER_SIZE);
    ORB_SLAM2::SGFilter zFilter(POLY_ORDER, FILTER_SIZE);

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

    SET_CLOCK(t0);
    int frameNumber = 0;
    while (true) {

      //Get each frame
      frames = pipe.wait_for_frames();


      SET_CLOCK(t1);
      
      cv::Mat infared, depth;
      
      // Occassionally re-align the image and depth streams 
      if (frameNumber % 150 == 0) 
      {
      if (profile_changed(pipe.get_active_profile().get_streams(), selection.get_streams()))
      {
          //If the profile was changed, update the align object, and also get the new device's depth scale
          selection = pipe.get_active_profile();
          align_to = find_stream_to_align(selection.get_streams());
          align = rs2::align(align_to);
          depth_scale = get_depth_scale(selection.get_device());
      }
      

      //Get processed aligned frame
      auto processed = align.process(frames);

      // Trying to get both other and aligned depth frames
      rs2::video_frame other_frame = processed.first(align_to);
      rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();


      infared = frame_to_mat(other_frame);
      depth = frame_to_mat(aligned_depth_frame);

      // Continue normal pipeline without re-alignment
      } else {

      rs2::video_frame ir_frame = frames.first(RS2_STREAM_COLOR);
      rs2::depth_frame d_frame = frames.get_depth_frame();

      infared = frame_to_mat(ir_frame);
      depth = frame_to_mat(d_frame);
      }

      SET_CLOCK(imfeedEnd);
      PRINT_CLOCK("Align image feed", imfeedEnd, t1);

      double tframe = TIME_DIFF(t1, t0);
      if (tframe > TIME) {
        break;
      }

      PUSH_RANGE("Track image", 4);
      // Pass the image to the SLAM system
      cv::Mat Tcw = SLAM.TrackRGBD(infared, depth, tframe);
      //float *vel = SLAM.getCameraVelocity();
      //std::cout << "x: " << vel[0] << " y: " << vel[1] << " z: " << vel[2] << std::endl;
      /*
      cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t(); // Rotation information
      cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3); // translation information

      xFilter.update((float) tframe,twc.at<float>(0,0));
      yFilter.update((float) tframe,twc.at<float>(0,1));
      zFilter.update((float) tframe,twc.at<float>(0,2));

      static float vel[3];
      vel[0] = xFilter.data_.yDot;
      vel[1] = yFilter.data_.yDot;
      vel[2] = zFilter.data_.yDot;
      std::cout << "x: " << vel[0] << " y: " << vel[1] << " z: " << vel[2] << std::endl;
      */

      POP_RANGE;
      SET_CLOCK(t2);

      double trackTime = TIME_DIFF(t2, t1);
      trackTimeSum += trackTime;
      tsum = tframe - tbuf[tpos];
      tbuf[tpos] = tframe;
      tpos = (tpos + 1) % 10;
      //cerr << "Frame " << frameNumber << " : " << tframe << " " << trackTime << " " << 10 / tsum << "\n";
      ++frameNumber;



      //Handle Control+C on stop function 
      if (flag) {
          cerr << "Mean track time: " << trackTimeSum / frameNumber << " , mean fps: " << frameNumber / TIME_DIFF(t2,t0) << "\n";

          // Save camera trajectory
          SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

          // Stop all threads
          SLAM.Shutdown();

          return 0;
      }

    }

    cerr << "Mean track time: " << trackTimeSum / frameNumber << " , mean fps: " << frameNumber / TIME << "\n";

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

float get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
{
    //Given a vector of streams, we try to find a depth stream and another stream to align depth with.
    //We prioritize color streams to make the view look better.
    //If color is not available, we take another stream that (other than depth)
    rs2_stream align_to = RS2_STREAM_ANY;
    bool depth_stream_found = false;
    bool color_stream_found = false;
    for (rs2::stream_profile sp : streams)
    {
        rs2_stream profile_stream = sp.stream_type();
        if (profile_stream != RS2_STREAM_DEPTH)
        {
            if (!color_stream_found)         //Prefer color
                align_to = profile_stream;

            if (profile_stream == RS2_STREAM_COLOR)
            {
                color_stream_found = true;
            }
        }
        else
        {
            depth_stream_found = true;
        }
    }

    if(!depth_stream_found)
        throw std::runtime_error("No Depth stream available");

    if (align_to == RS2_STREAM_ANY)
        throw std::runtime_error("No stream found to align with Depth");

    return align_to;
}

bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
{
    for (auto&& sp : prev)
    {
        //If previous profile is in current (maybe just added another)
        auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp) { return sp.unique_id() == current_sp.unique_id(); });
        if (itr == std::end(current)) //If it previous stream wasn't found in current
        {
            return true;
        }
    }
    return false;
}

