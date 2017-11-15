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
*/
#include <depthimage_to_laserscan/DepthImageToLaserScan.h>
// Bring in gtest
#include "DepthToScan.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <mutex>

namespace ORB_SLAM2
{

DepthToScan::DepthToScan(const std::string &strSettingPath)
{
  cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);   
  // Set up library
  float fx = fSettings["Camera.fx"];
  float fy = fSettings["Camera.fy"];
  float cx = fSettings["Camera.cx"];
  float cy = fSettings["Camera.cy"];
  
  cv::Mat K = cv::Mat::eye(3,3,CV_32F);
  K.at<float>(0,0) = fx;
  K.at<float>(1,1) = fy;
  K.at<float>(0,2) = cx;
  K.at<float>(1,2) = cy;
  cv::Mat DistCoef(4,1,CV_32F);
  
  DistCoef.at<float>(0) = fSettings["Camera.k1"];
  DistCoef.at<float>(1) = fSettings["Camera.k2"];
  DistCoef.at<float>(2) = fSettings["Camera.p1"];
  DistCoef.at<float>(3) = fSettings["Camera.p2"];
  const float k3 = fSettings["Camera.k3"];
  if(k3!=0)
   {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
   
  float fps = fSettings["Camera.fps"];
  if(fps==0)
        fps=15;   
  float scan_time=1/fps;
  float mImageWidth = fSettings["Camera.width"];
  float mImageHeight = fSettings["Camera.height"];
  float range_min = 0.30;
  float range_max = 3.00;
  float cam_height=0.75;
  float maxpass_height=0.15;
  float range_depth=sqrt(range_max*range_max-(cam_height-maxpass_height)*(cam_height-maxpass_height));
  int scan_height = int(2*(cam_height-maxpass_height)/range_depth*fy);
    
  dtl_.set_scan_time(scan_time);
  dtl_.set_range_limits(range_min, range_max);
  dtl_.set_scan_height(scan_height);
  std::string output_frame = "/ORB_SLAM/current_frame";
  dtl_.set_output_frame(output_frame);
    
  //depth_msg_.reset(new sensor_msgs::Image);
  //depth_msg_->header.seq = 42;
  //depth_msg_->header.stamp.fromNSec(1234567890);
  //depth_msg_->header.frame_id = "/ORB_SLAM/current_frame";
  //depth_msg_->height = mImageHeight;
  //depth_msg_->width = mImageWidth;
  //depth_msg_->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  //depth_msg_->is_bigendian = false;
  //depth_msg_->step = depth_msg_->width*4; // 4 bytes per pixel
  //depth_msg_->data.assign(depth_msg_->height*depth_msg_->step, std::numeric_limits<float>::infinity());
    
  info_msg_.reset(new sensor_msgs::CameraInfo);
 // info_msg_->header = depth_msg_->header;
  info_msg_->height = mImageHeight;
  info_msg_->width =mImageWidth;
  info_msg_->distortion_model = "plumb_bob";
  info_msg_->D.resize(5); // All 0, no distortion
  //info_msg_->D[0]= DistCoef.at<float>(0) ;
  //info_msg_->D[1]= DistCoef.at<float>(1) ;
  info_msg_->K[0] = fx ;
  info_msg_->K[2] = cx;
  info_msg_->K[4] = fy;
  info_msg_->K[5] = cy;
  info_msg_->K[8] = 1.0;
  info_msg_->R[0] = 1.0;
  info_msg_->R[4] = 1.0;
  info_msg_->R[8] = 1.0;
  info_msg_->P[0] = fx;
  info_msg_->P[2] = cx;
  info_msg_->P[5] = fy;
  info_msg_->P[6] = cy;
  info_msg_->P[10] = 1.0;
  
}
  

sensor_msgs::LaserScanPtr DepthToScan::ConvScan(sensor_msgs::ImageConstPtr& depth_msg_)
{   
 sensor_msgs::LaserScanPtr scan_msg = dtl_.convert_msg(depth_msg_, info_msg_);
 return scan_msg;
}


} //namespace ORB_SLAM
