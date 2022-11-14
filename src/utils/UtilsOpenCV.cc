#include "utils/UtilsOpenCV.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace vdo
{
namespace utils
{
void DrawCircleInPlace(cv::Mat& img, const cv::Point2d& point, const cv::Scalar& colour, const double msize)
{
  cv::circle(img, point, msize, colour, 2);
}

std::string cvTypeToString(int type)
{
  std::string r;
  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);
  switch (depth)
  {
    case CV_8U:
      r = "8U";
      break;
    case CV_8S:
      r = "8S";
      break;
    case CV_16U:
      r = "16U";
      break;
    case CV_16S:
      r = "16S";
      break;
    case CV_32S:
      r = "32S";
      break;
    case CV_32F:
      r = "32F";
      break;
    case CV_64F:
      r = "64F";
      break;
    default:
      r = "User";
      break;
  }
  r += "C";
  r += (chans + '0');
  return r;
}
std::string cvTypeToString(const cv::Mat& mat)
{
  return cvTypeToString(mat.type());
}

cv::Mat concatenateImagesHorizontally(const cv::Mat& left_img, const cv::Mat& right_img)
{
  cv::Mat left_img_tmp = left_img.clone();
  if (left_img_tmp.channels() == 1)
  {
    cv::cvtColor(left_img_tmp, left_img_tmp, cv::COLOR_GRAY2BGR);
  }
  cv::Mat right_img_tmp = right_img.clone();
  if (right_img_tmp.channels() == 1)
  {
    cv::cvtColor(right_img_tmp, right_img_tmp, cv::COLOR_GRAY2BGR);
  }

  cv::Size left_img_size = left_img_tmp.size();
  cv::Size right_img_size = right_img_tmp.size();

  CHECK_EQ(left_img_size.height, left_img_size.height) << "Cannot concat horizontally if images are not the same "
                                                          "height";

  cv::Mat dual_img(left_img_size.height, left_img_size.width + right_img_size.width, CV_8UC3);

  cv::Mat left(dual_img, cv::Rect(0, 0, left_img_size.width, left_img_size.height));
  left_img_tmp.copyTo(left);

  cv::Mat right(dual_img, cv::Rect(left_img_size.width, 0, right_img_size.width, right_img_size.height));

  right_img_tmp.copyTo(right);
  return dual_img;
}

cv::Mat concatenateImagesVertically(const cv::Mat& top_img, const cv::Mat& bottom_img)
{
  cv::Mat top_img_tmp = top_img.clone();
  if (top_img_tmp.channels() == 1)
  {
    cv::cvtColor(top_img_tmp, top_img_tmp, cv::COLOR_GRAY2BGR);
  }
  cv::Mat bottom_img_tmp = bottom_img.clone();
  if (bottom_img_tmp.channels() == 1)
  {
    cv::cvtColor(bottom_img_tmp, bottom_img_tmp, cv::COLOR_GRAY2BGR);
  }

  cv::Size top_img_size = bottom_img_tmp.size();
  cv::Size bottom_img_size = bottom_img_tmp.size();

  CHECK_EQ(top_img_size.width, bottom_img_size.width) << "Cannot concat vertically if images are not the same width";

  cv::Mat dual_img(top_img_size.height + bottom_img_size.height, top_img_size.width, CV_8UC3);

  cv::Mat top(dual_img, cv::Rect(0, 0, top_img_size.width, top_img_size.height));
  top_img_tmp.copyTo(top);

  cv::Mat bottom(dual_img, cv::Rect(0, top_img_size.height, bottom_img_size.width, bottom_img_size.height));

  bottom_img_tmp.copyTo(bottom);
  return dual_img;
}

void drawCircleInPlace(cv::Mat& img, const cv::KeyPoint& kp, const cv::Scalar& color)
{
  CHECK(!img.empty());
  cv::circle(img, cv::Point(kp.pt.x, kp.pt.y), 3, color, 1u);
}

cv::Affine3d matPoseToCvAffine3d(const cv::Mat& pose)
{
  CHECK(pose.rows == 4 && pose.cols == 4);
  // convert to double
  cv::Mat posed;
  pose.convertTo(posed, CV_64F);
  return cv::Affine3d(posed);
}

cv::Affine3d gtsamPose3ToCvAffine3d(const gtsam::Pose3& pose)
{
  cv::Mat RT(4, 4, CV_64F);
  cv::eigen2cv(pose.matrix(), RT);
  return cv::Affine3d(RT);
}

cv::Mat transformCameraPoseToWorld(const cv::Mat& pose)
{
  // note the rotation we care about is the 3,3 head part
  static cv::Mat transform = (cv::Mat_<float>(4, 4) << 0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1);
  return transform * pose;
}

// // add circles in the image at desired position/size/color
// void DrawCirclesInPlace(cv::Mat& img,
//                                      const KeypointsCV& image_points,
//                                      const cv::Scalar& color,
//                                      const double& msize,
//                                      const std::vector<int>& point_ids,
//                                      const int& rem_id) {
//   // text offset
//   cv::Point2f text_offset(-10, -5);
//   if (img.channels() < 3) cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
//   for (size_t i = 0u; i < image_points.size(); i++) {
//     cv::circle(img, image_points[i], msize, color, 2);
//     if (point_ids.size() == image_points.size()) {
//       // We also have text
//       cv::putText(img,
//                   std::to_string(point_ids[i] % rem_id),
//                   image_points[i] + text_offset,
//                   CV_FONT_HERSHEY_COMPLEX,
//                   0.5,
//                   color);
//     }
//   }
// }
// /* -------------------------------------------------------------------------- */
// // add squares in the image at desired position/size/color
// void DrawSquaresInPlace(
//     cv::Mat& img,
//     const std::vector<cv::Point2f>& imagePoints,
//     const cv::Scalar& color,
//     const double msize,
//     const std::vector<int>& pointIds,
//     const int remId) {
//   cv::Point2f textOffset = cv::Point2f(-10, -5);  // text offset
//   if (img.channels() < 3) cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
//   for (size_t i = 0; i < imagePoints.size(); i++) {
//     cv::Rect square = cv::Rect(imagePoints[i].x - msize / 2,
//                                imagePoints[i].y - msize / 2,
//                                msize,
//                                msize);
//     rectangle(img, square, color, 2);
//     if (pointIds.size() == imagePoints.size())  // we also have text
//       cv::putText(img,
//                   std::to_string(pointIds[i] % remId),
//                   imagePoints[i] + textOffset,
//                   CV_FONT_HERSHEY_COMPLEX,
//                   0.5,
//                   color);
//   }
// }
// /* -------------------------------------------------------------------------- */
// // add x in the image at desired position/size/color
// void DrawCrossesInPlace(
//     cv::Mat& img,
//     const std::vector<cv::Point2f>& imagePoints,
//     const cv::Scalar& color,
//     const double msize,
//     const std::vector<int>& pointIds,
//     const int remId) {
//   cv::Point2f textOffset = cv::Point2f(-10, -5);         // text offset
//   cv::Point2f textOffsetToCenter = cv::Point2f(-3, +3);  // text offset
//   if (img.channels() < 3) cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
//   for (size_t i = 0; i < imagePoints.size(); i++) {
//     cv::putText(img,
//                 "X",
//                 imagePoints[i] + textOffsetToCenter,
//                 CV_FONT_HERSHEY_COMPLEX,
//                 msize,
//                 color,
//                 2);
//     if (pointIds.size() == imagePoints.size())  // we also have text
//       cv::putText(img,
//                   std::to_string(pointIds[i] % remId),
//                   imagePoints[i] + textOffset,
//                   CV_FONT_HERSHEY_COMPLEX,
//                   0.5,
//                   color);
//   }
// }

}  // namespace utils
}  // namespace vdo