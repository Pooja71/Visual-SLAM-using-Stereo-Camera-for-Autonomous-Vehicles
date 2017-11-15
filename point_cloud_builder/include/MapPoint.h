#ifndef MAPPOINT_H
#define MAPPOINT_H

#include <opencv2/core/version.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>

#include <sstream>

class MapPoint
{
public:
    MapPoint(float x, float y, float z);
    void updatePoint(cv::Mat transform);
    std::string getPlyString();
private:
    int keyframeId;
    cv::Mat pos;
};
#endif
