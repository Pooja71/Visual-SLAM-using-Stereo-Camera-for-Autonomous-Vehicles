/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include "OGBuilder.h"
#include <cstdio>
#include <iostream>


using namespace std;

namespace ORB_SLAM2
{
  OGBuilder::OGBuilder(unsigned int cells_size_x, unsigned int cells_size_y, double resolution,double origin_x, double origin_y,
  unsigned char default_value,std::string global_frame,std::string topic_name, Map* pMap, LocalMapping* pLocalMapping, float thed): size_x_(cells_size_x), size_y_(cells_size_y), resolution_(resolution), origin_x_(origin_x),
    origin_y_(origin_y), costmap_(NULL),probmap_(NULL),countmap_(NULL), default_value_(default_value), global_frame_(global_frame),mpMap(pMap),mpLocalMapping(pLocalMapping), thed_(thed)
{
//  access_ = new mutex_t();
  OGPublisher = nh.advertise<nav_msgs::OccupancyGrid>("ORB_SLAM/OCCUPANCY_GRID", 10);

  // create the costmap
  initMaps(size_x_ , size_y_);
  resetMaps();
  time(&now);

}

// prepare grid_ message for publication.
void OGBuilder::prepareGrid()
{
//  boost::unique_lock<mutex_t> lock(*access_);
  grid_.header.frame_id = global_frame_;
  grid_.header.stamp = ros::Time::now();
  grid_.info.resolution = resolution_;

  grid_.info.width = size_x_;
  grid_.info.height = size_y_;

  double wx, wy;
  mapToWorld(0, 0, wx, wy);
  grid_.info.origin.position.x = wx - resolution_ / 2;
  grid_.info.origin.position.y = wy - resolution_ / 2;
  grid_.info.origin.position.z = 0.0;
  grid_.info.origin.orientation.w = 1.0;
  saved_origin_x_ = getOriginX();
  saved_origin_y_ = getOriginY();

  grid_.data.resize(grid_.info.width * grid_.info.height);


  for (unsigned int i = 0; i < grid_.data.size(); i++)
  {
    grid_.data[i] = unsigned (costmap_[i]);
  }
}



void OGBuilder::deleteMaps()
{
  // clean up data
//  boost::unique_lock<mutex_t> lock(*access_);
  delete[] costmap_;
  costmap_ = NULL;
  delete[] probmap_;
  probmap_ = NULL;
  delete[] countmap_;
  countmap_ = NULL;
}

void OGBuilder::initMaps(unsigned int size_x, unsigned int size_y)
{
//  boost::unique_lock<mutex_t> lock(*access_);
  delete[] costmap_;
  delete[] probmap_;
  delete[] countmap_;

  costmap_ = new unsigned char[size_x * size_y];
  probmap_ = new float[size_x*size_y];
  countmap_ = new float[size_x*size_y];

}

void OGBuilder::resizeMap(unsigned int size_x, unsigned int size_y, double resolution,
                          double origin_x, double origin_y)
{
  size_x_ = size_x;
  size_y_ = size_y;
  resolution_ = resolution;
  origin_x_ = origin_x;
  origin_y_ = origin_y;

  initMaps(size_x, size_y);

  // reset our maps to have no information
  resetMaps();
}

void OGBuilder::resetMaps()
{
//  boost::unique_lock<mutex_t> lock(*access_);
  memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));
  memset(probmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));
  memset(countmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));
}

void OGBuilder::resetMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn)
{
//  boost::unique_lock<mutex_t> lock(*(access_));
  unsigned int len = xn - x0;
  for (unsigned int y = y0 * size_x_ + x0; y < yn * size_x_ + x0; y += size_x_)
    memset(costmap_ + y, default_value_, len * sizeof(unsigned char));
}

void OGBuilder::MapBuilder()
{
 //   boost::unique_lock<mutex_t> lock(*access_);
    std::unordered_map<long unsigned int,sensor_msgs::LaserScanPtr> mpkeyFrameLaserScanMap=mpLocalMapping->GetKeyFrameLaserScanMap();
    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
   // Sort all keyframes so that the first keyframe is at the origin.
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // After a loop closure the first keyframe m
    cv::Mat Xlc=cv::Mat::zeros(3,1, CV_32FC1);
    cv::Mat Xlw=cv::Mat::zeros(3,1, CV_32FC1);
    cv::Mat Rwc=cv::Mat::zeros(3,3, CV_32FC1);
    cv::Mat twc=cv::Mat::zeros(3,1, CV_32FC1);
    sensor_msgs::LaserScanPtr scan_msg;
    unsigned int mx, my;
    float r;
    KeyFrame* pKF;
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            pKF = vpKFs[i];
            Rwc = vpKFs[i]->GetRotation().t();
            twc = vpKFs[i]->GetCameraCenter();
            unordered_map<long unsigned int,sensor_msgs::LaserScanPtr>::const_iterator itr=mpkeyFrameLaserScanMap.find(pKF->mnId);


            if (itr!=mpkeyFrameLaserScanMap.end())
            {
              scan_msg=itr->second;
              unsigned int noe= unsigned ((scan_msg->angle_max-scan_msg->angle_min)/scan_msg->angle_increment);

              for(size_t j=0; j<noe; j++)
              {
                r=scan_msg->ranges[j];
                if(r<scan_msg->range_max && r>scan_msg->range_min)
                {
                   Xlc.at<float>(2)=float(r*cos(scan_msg->angle_increment*(j)+scan_msg->angle_min));
                   Xlc.at<float>(0)=-float(r*sin(scan_msg->angle_increment*(j)+scan_msg->angle_min));
                   Xlc.at<float>(1)=0;
                   Xlw=Rwc*Xlc+twc;
                   if(origin_x_<=Xlw.at<float>(2)&&Xlw.at<float>(2)<resolution_*size_x_+origin_x_&&origin_y_<=-Xlw.at<float>(0)&&-Xlw.at<float>(0)<resolution_*(size_y_+origin_y_))
                   {
                       worldToMap(Xlw.at<float>(2),-Xlw.at<float>(0), mx, my);
                       setCost(mx, my, 100 );
                    }

                  }

               }
        }
    }
}


OGBuilder& OGBuilder::operator=(const OGBuilder& map)
{
  // check for self assignement
  if (this == &map)
    return *this;

  // clean up old data
  deleteMaps();

  size_x_ = map.size_x_;
  size_y_ = map.size_y_;
  resolution_ = map.resolution_;
  origin_x_ = map.origin_x_;
  origin_y_ = map.origin_y_;

  // initialize our various maps
  initMaps(size_x_, size_y_);

  // copy the cost map
  memcpy(costmap_, map.costmap_, size_x_ * size_y_ * sizeof(unsigned char));

  return *this;
}

OGBuilder::OGBuilder(const OGBuilder& map) :
    costmap_(NULL)
{
//  access_ = new mutex_t();
  *this = map;
}

// just initialize everything to NULL by default
OGBuilder::OGBuilder() :
    size_x_(0), size_y_(0), resolution_(0.0), origin_x_(0.0), origin_y_(0.0), costmap_(NULL)
{
//  access_ = new mutex_t();
}

OGBuilder::~OGBuilder()
{
  deleteMaps();
//  delete access_;
}

unsigned int OGBuilder::cellDistance(double world_dist)
{
  double cells_dist = max(0.0, ceil(world_dist / resolution_));
  return (unsigned int)cells_dist;
}

unsigned char* OGBuilder::getCharMap() const
{
  return costmap_;
}

unsigned char OGBuilder::getCost(unsigned int mx, unsigned int my) const
{

  return costmap_[getIndex(mx, my)];
}

void OGBuilder::setCost(unsigned int mx, unsigned int my, unsigned char cost)
{
    unsigned int indx=getIndex(mx,my);
    costmap_[indx] =cost;
}

void OGBuilder::mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const
{
  wx = origin_x_ + (mx + 0.5) * resolution_;
  wy = origin_y_ + (my + 0.5) * resolution_;
}

bool OGBuilder::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const
{
  if (wx < origin_x_ || wy < origin_y_)
    return false;

  mx = (int)((wx - origin_x_) / resolution_);
  my = (int)((wy - origin_y_) / resolution_);

  if (mx < size_x_ && my < size_y_)
    return true;

  return false;
}



void OGBuilder::worldToMapEnforceBounds(double wx, double wy, int& mx, int& my) const
{
  // Here we avoid doing any math to wx,wy before comparing them to
  // the bounds, so their values can go out to the max and min values
  // of double floating point.
  if (wx < origin_x_)
  {
    mx = 0;
  }
  else if (wx > resolution_ * size_x_ + origin_x_)
  {
    mx = size_x_ - 1;
  }
  else
  {
    mx = (int)((wx - origin_x_) / resolution_);
  }

  if (wy < origin_y_)
  {
    my = 0;
  }
  else if (wy > resolution_ * size_y_ + origin_y_)
  {
    my = size_y_ - 1;
  }
  else
  {
    my = (int)((wy - origin_y_) / resolution_);
  }
}

void OGBuilder::updateOrigin(double new_origin_x, double new_origin_y)
{
  // project the new origin into the grid
  int cell_ox, cell_oy;
  cell_ox = int((new_origin_x - origin_x_) / resolution_);
  cell_oy = int((new_origin_y - origin_y_) / resolution_);

  // compute the associated world coordinates for the origin cell
  // because we want to keep things grid-aligned
  double new_grid_ox, new_grid_oy;
  new_grid_ox = origin_x_ + cell_ox * resolution_;
  new_grid_oy = origin_y_ + cell_oy * resolution_;

  // To save casting from unsigned int to int a bunch of times
  int size_x = size_x_;
  int size_y = size_y_;

  // we need to compute the overlap of the new and existing f
  int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
  lower_left_x = min(max(cell_ox, 0), size_x);
  lower_left_y = min(max(cell_oy, 0), size_y);
  upper_right_x = min(max(cell_ox + size_x, 0), size_x);
  upper_right_y = min(max(cell_oy + size_y, 0), size_y);

  unsigned int cell_size_x = upper_right_x - lower_left_x;
  unsigned int cell_size_y = upper_right_y - lower_left_y;

  // we need a map to store the obstacles in the window temporarily
  unsigned char* local_map = new unsigned char[cell_size_x * cell_size_y];

  // copy the local window in the costmap to the local map
  copyMapRegion(costmap_, lower_left_x, lower_left_y, size_x_, local_map, 0, 0, cell_size_x, cell_size_x, cell_size_y);

  // now we'll set the costmap to be completely unknown if we track unknown space
  resetMaps();

  // update the origin with the appropriate world coordinates
  origin_x_ = new_grid_ox;
  origin_y_ = new_grid_oy;

  // compute the starting cell location for copying data back in
  int start_x = lower_left_x - cell_ox;
  int start_y = lower_left_y - cell_oy;

  // now we want to copy the overlapping information back into the map, but in its new location
  copyMapRegion(local_map, 0, 0, cell_size_x, costmap_, start_x, start_y, size_x_, cell_size_x, cell_size_y);

  // make sure to clean up
  delete[] local_map;
}


unsigned int OGBuilder::getSizeInCellsX() const
{
  return size_x_;
}

unsigned int OGBuilder::getSizeInCellsY() const
{
  return size_y_;
}

double OGBuilder::getSizeInMetersX() const
{
  return (size_x_ - 1 + 0.5) * resolution_;
}

double OGBuilder::getSizeInMetersY() const
{
  return (size_y_ - 1 + 0.5) * resolution_;
}

double OGBuilder::getOriginX() const
{
  return origin_x_;
}

double OGBuilder::getOriginY() const
{
  return origin_y_;
}

double OGBuilder::getResolution() const
{
  return resolution_;
}

bool OGBuilder::saveMap(std::string file_name)
{
  FILE *fp = fopen(file_name.c_str(), "w");

  if (!fp)
  {
    return false;
  }

  fprintf(fp, "P2\n%u\n%u\n%u\n", size_x_, size_y_, 0xff);
  for (unsigned int iy = 0; iy < size_y_; iy++)
  {
    for (unsigned int ix = 0; ix < size_x_; ix++)
    {
      unsigned char cost = getCost(ix, iy);
      fprintf(fp, "%d ", cost);
    }
    fprintf(fp, "\n");
  }
  fclose(fp);
  return true;

}



//joe
void OGBuilder::mapUpdateLoop(double frequency)
{
  // the user might not want to run the loop every cycle
  if (frequency == 0.0)
    return;

 time_t now_new;
 time(&now_new);

 double t_diff = difftime(now_new,now);
//std::chrono::milliseconds current_t=std::chrono::duration_cast<milliseconds>(std::chrono::system_clock::now().time_since_epoch())
 if (nh.ok() && t_diff>1/frequency)
  {
    resetMaps();
    MapBuilder();
    prepareGrid();
    OGPublisher.publish(grid_);
   time_t test_now_new;
   time(&test_now_new);

   double t_diff_test =0.001;//difftime(test_now_new,now_new);
   // std::cout<<"og process time: "<<t_diff<<" "<<now_new<<std::endl;
    now=now_new;      //to be added in
   }
}

}  // namespace costmap_2d
