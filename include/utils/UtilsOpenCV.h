#pragma once

#include <opencv2/opencv.hpp>

namespace vdo {
namespace utils {


void DrawCircleInPlace(cv::Mat& img, const cv::Point2d& point, const cv::Scalar& colour,  const double msize= 0.4);

// // add circles in the image at desired position/size/color
// void DrawCirclesInPlace(cv::Mat& img,
//                                      const KeypointsCV& image_points,
//                                      const cv::Scalar& color,
//                                      const double& msize,
//                                      const std::vector<int>& point_ids,
//                                      const int& rem_id);
// }
// /* -------------------------------------------------------------------------- */
// // add squares in the image at desired position/size/color
// void DrawSquaresInPlace(
//     cv::Mat& img,
//     const std::vector<cv::Point2f>& imagePoints,
//     const cv::Scalar& color,
//     const double msize,
//     const std::vector<int>& pointIds,
//     const int remId);
// }
// /* -------------------------------------------------------------------------- */
// // add x in the image at desired position/size/color
// void DrawCrossesInPlace(
//     cv::Mat& img,
//     const std::vector<cv::Point2f>& imagePoints,
//     const cv::Scalar& color,
//     const double msize,
//     const std::vector<int>& pointIds,
//     const int remId);


}
}