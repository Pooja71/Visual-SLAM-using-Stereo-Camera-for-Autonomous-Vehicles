#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include <opencv2/core/version.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>

#include <string>
#include "MapPoint.h"

#include "KeyFrame.h"

KeyFrame::KeyFrame(visualization_msgs::Marker msg)
{
    numPoints=0;
    id = msg.header.frame_id;
    projectionMat = (cv::Mat_<float>(3,4) << msg.points[0].x, msg.points[1].x, msg.points[2].x, msg.points[3].x, msg.points[0].y, msg.points[1].y, msg.points[2].y, msg.points[3].y, msg.points[0].z, msg.points[1].z, msg.points[2].z, msg.points[3].z);
}
void KeyFrame::updateKeyFrame(visualization_msgs::Marker msg)
{
    cv::Mat newProjectionMat = (cv::Mat_<float>(3,4, CV_32F) << msg.points[0].x, msg.points[1].x, msg.points[2].x, msg.points[3].x, msg.points[0].y, msg.points[1].y, msg.points[2].y, msg.points[3].y, msg.points[0].z, msg.points[1].z, msg.points[2].z, msg.points[3].z);
    cv::Mat transform = newProjectionMat * projectionMat.inv();
    for(std::vector<MapPoint*>::iterator it = associatedPoints.begin(); it != associatedPoints.end(); it++)
    {
	(*it)->updatePoint(transform);
    }
    projectionMat = newProjectionMat;
}
void KeyFrame::addPointCloud(pcl::PointCloud<pcl::PointXYZ> pc)
{
    for(pcl::PointCloud<pcl::PointXYZ>::iterator it = pc.begin(); it != pc.end(); it++)
    {
	    addPoint(new MapPoint(it->x, it->y, it->z));
    }
}
void KeyFrame::addPoint(MapPoint* pt)
{
    numPoints++;
    associatedPoints.push_back(pt);
}

std::string KeyFrame::getPLY()
{
    std::string str = "";
    for(std::vector<MapPoint*>::iterator it = associatedPoints.begin(); it != associatedPoints.end(); ++it)
    {
        str = str + (*it)->getPlyString();
    }
    return str;
}

int KeyFrame::getNumPoints()
{
    return numPoints;
}
