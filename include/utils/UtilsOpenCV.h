#pragma once

#include <opencv2/opencv.hpp>
#include <glog/logging.h>
#include <gtsam/geometry/Pose3.h>
#include <string>

namespace vdo
{
namespace utils
{
void DrawCircleInPlace(cv::Mat& img, const cv::Point2d& point, const cv::Scalar& colour, const double msize = 0.4);

#define CHECK_MAT_TYPES(mat1, mat2)                                                                                    \
  using namespace VDO_SLAM::utils;                                                                                     \
  CHECK_EQ(mat1.type(), mat2.type()) << "Matricies should be of the same type ( " << cvTypeToString(mat1.type())       \
                                     << " vs. " << cvTypeToString(mat2.type()) << ")."

std::string cvTypeToString(int type);
std::string cvTypeToString(const cv::Mat& mat);

cv::Mat concatenateImagesHorizontally(const cv::Mat& left_img, const cv::Mat& right_img);

cv::Mat concatenateImagesVertically(const cv::Mat& top_img, const cv::Mat& bottom_img);

cv::Affine3d matPoseToCvAffine3d(const cv::Mat& pose);

// should be in utils GTSAM but meh
cv::Affine3d gtsamPose3ToCvAffine3d(const gtsam::Pose3& pose);

// // applies a rotation to a pose in the camera frame (z forward)
// // to get it to align with the standard axis
cv::Mat alignCameraPoseToWorld(const cv::Mat& pose);

void flowToRgb(const cv::Mat& flow, cv::Mat& rgb);

}  // namespace utils
}  // namespace vdo