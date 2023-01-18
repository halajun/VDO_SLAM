#include "utils/UtilsG2O.h"

namespace vdo
{
namespace utils
{
g2o::SE3Quat toSE3Quat(const cv::Mat& cvT)
{
  Eigen::Matrix<double, 3, 3> R;
  R << cvT.at<double>(0, 0), cvT.at<double>(0, 1), cvT.at<double>(0, 2), cvT.at<double>(1, 0), cvT.at<double>(1, 1),
      cvT.at<double>(1, 2), cvT.at<double>(2, 0), cvT.at<double>(2, 1), cvT.at<double>(2, 2);

  Eigen::Matrix<double, 3, 1> t(cvT.at<double>(0, 3), cvT.at<double>(1, 3), cvT.at<double>(2, 3));

  return g2o::SE3Quat(R, t);
}

gtsam::Pose3 toGtsamPose3(const g2o::SE3Quat& se3_quat)
{
  Eigen::Matrix<double, 4, 4> eigMat = se3_quat.to_homogeneous_matrix();
  return gtsam::Pose3(eigMat);
}

}  // namespace utils

}  // namespace vdo