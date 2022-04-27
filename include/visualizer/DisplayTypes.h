#pragma once

#include "utils/macros.h"

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

namespace VDO_SLAM {

struct ImageToDisplay {
    VDO_SLAM_POINTER_TYPEDEFS(ImageToDisplay);
    typedef std::vector<ImageToDisplay> Vector;

    ImageToDisplay(const cv::Mat& image_,const std::string& title_)
        :   image(image_), title(title_) {}

    const cv::Mat image;
    const std::string title;

};


}