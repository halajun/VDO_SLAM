#pragma once

#include "utils/macros.h"

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <map>
#include <memory>

namespace VDO_SLAM {

typedef std::unique_ptr<cv::viz::Widget3D> WidgetPtr;
typedef std::map<std::string, WidgetPtr> WidgetsMap;
typedef std::vector<std::string> WidgetIds;

struct ImageToDisplay {
    VDO_SLAM_POINTER_TYPEDEFS(ImageToDisplay);
    typedef std::vector<ImageToDisplay> Vector;

    ImageToDisplay(const cv::Mat& image_,const std::string& title_)
        :   image(image_), title(title_) {}

    const cv::Mat image;
    const std::string title;

};


}