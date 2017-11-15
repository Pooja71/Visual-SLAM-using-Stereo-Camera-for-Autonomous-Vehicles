#include "MapPoint.h"

#include <opencv2/core/version.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>

MapPoint::MapPoint(float x, float y, float z)
{
    std::cout << "New Point (X,Y,Z): " << x << " " << y << " " << z << std::endl;
    pos = (cv::Mat_<float>(4,1) << x, y, z);
	
}
void MapPoint::updatePoint(cv::Mat transform)
{
    pos = transform * pos;
}

std::string MapPoint::getPlyString()
{
    std::ostringstream strs;
    strs << pos.at<float>(0);
    strs << " ";
    strs << pos.at<float>(1);
    strs << " ";
    strs << pos.at<float>(2);
    strs << "\n";
    std::string str = strs.str();
    return str;
}
