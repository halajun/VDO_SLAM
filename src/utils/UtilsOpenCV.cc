#include "utils/UtilsOpenCV.h"
#include <opencv2/opencv.hpp>

namespace vdo
{
namespace utils
{
void DrawCircleInPlace(cv::Mat& img, const cv::Point2d& point, const cv::Scalar& colour, const double msize)
{
  cv::circle(img, point, msize, colour, 2);
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