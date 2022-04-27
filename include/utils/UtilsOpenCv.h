#pragma once

#include <opencv2/opencv.hpp>
#include <glog/logging.h>

namespace VDO_SLAM {
namespace utils {


cv::Mat concatenateImagesHorizontally(
    const cv::Mat& left_img,
    const cv::Mat& right_img);

cv::Mat concatenateImagesVertically(
    const cv::Mat& top_img,
    const cv::Mat& bottom_img);

}
}