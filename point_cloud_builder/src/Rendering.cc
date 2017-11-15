#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include <unordered_map>
#include <vector>
#include <iterator>
#include <string>

#include "KeyFrame.h"
#include "Rendering.h"

static unordered_map<std::string, KeyFrame> keyFrameHash;
Rendering::Rendering()
{   


}

void Rendering::run()
{
    key_frame_listener = nh.subscribe("ORB_SLAM/KFs",10,routeKeyframe);
    pc_listener = nh.subscribe("ORB_SLAM/PCs",10,routePointcloud);
    ros::spin();
    displayMap();
}

void Rendering::displayMap()
{
    std::ofstream out("output.ply");
    out << getPLYString();
    out.close();
}

std::string Rendering::getPLYString()
{
    int numVerts = 0;
    std::string pointsStr = "";
    for (unordered_map<std::string,KeyFrame>::iterator kfIt = keyFrameHash.begin(); kfIt!=keyFrameHash.end(); kfIt++)
    {
        pointsStr = pointsStr + (*kfIt).second.getPLY();
        numVerts = numVerts + (*kfIt).second.getNumPoints();
    }
    std::ostringstream strs;
    strs << "ply\n";
    strs << "format ascii 1.0\n";
    strs << "element vertex ";
    strs << numVerts;
    strs << "\n";
    strs << "property double x\n";
    strs << "property double y\n";
    strs << "property double z\n";
    strs << "end_header\n";
    std::string str = strs.str();
    str = str + pointsStr;
    return str;
}

void Rendering::routeKeyframe(visualization_msgs::Marker msg)
{
    std::cout << "Getting Key Frame " << msg.header.frame_id << std::endl;
    std::string fId = msg.header.frame_id;
    int ct = keyFrameHash.count(fId);
    if( ct==0 )
    {
        KeyFrame newKF = KeyFrame(msg);
        keyFrameHash.insert({fId, newKF});

    } else {
        unordered_map<string,KeyFrame>::iterator kfIt = keyFrameHash.find(fId);
	(*kfIt).second.updateKeyFrame(msg);
    }
}

void Rendering::routePointcloud(const sensor_msgs::PointCloud2ConstPtr& pointCloud)
{
    std::cout << "Getting Key Frame " << (*pointCloud).header.frame_id << std::endl;
    std::string fId = (*pointCloud).header.frame_id;
    int ct = keyFrameHash.count(fId);
    if( ct==1 )
    {
        unordered_map<string,KeyFrame>::iterator kfIt = keyFrameHash.find(fId);
        pcl::PointCloud<pcl::PointXYZ> tmpPCL;
        pcl::fromROSMsg(*pointCloud,tmpPCL);
	(*kfIt).second.addPointCloud(tmpPCL);
    } 
}
