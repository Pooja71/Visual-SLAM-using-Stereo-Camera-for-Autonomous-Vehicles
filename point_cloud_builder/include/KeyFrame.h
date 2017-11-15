#ifndef KEYFRAME_H
#define KEYFRAME_H

#include <opencv2/core/version.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include "MapPoint.h"

class KeyFrame
{
public:
    KeyFrame(visualization_msgs::Marker msg);
    void updateKeyFrame(visualization_msgs::Marker msg);
    void addPointCloud(pcl::PointCloud<pcl::PointXYZ> pc);
    std::vector<MapPoint*> associatedPoints;
    std::string getPLY();
    int getNumPoints();
private:
    std::string id;
    int numPoints;
    cv::Mat projectionMat;
    void addPoint(MapPoint* pt);
};
#endif
