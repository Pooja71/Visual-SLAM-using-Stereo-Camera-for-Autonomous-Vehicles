#ifndef RENDERING_H
#define RENDERING_H

#include <unordered_map>
#include <iterator>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include "KeyFrame.h"

using namespace std;

class Rendering
{
public:
    Rendering();
    void run();
protected:
    void displayMap();
    std::string getPLYString();
    static void routeKeyframe(visualization_msgs::Marker msg);
    static void routePointcloud(const sensor_msgs::PointCloud2ConstPtr& pointCloud);
private:
    ros::NodeHandle nh;
    ros::Subscriber key_frame_listener;
    ros::Subscriber pc_listener;
};
#endif
