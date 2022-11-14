#include "utils/camera.h"
#include <glog/logging.h>

namespace VDO_SLAM
{
Camera::Camera(const cv::Mat& K_, const gtsam::Pose3 camera_pose_)
  : K(K_)
  , camera_pose(camera_pose_)
  , fx(K_.at<double>(0, 0))
  , cu(K_.at<double>(0, 2))
  , fy(K_.at<double>(1, 1))
  , cv(K_.at<double>(1, 2))
{
}

// 3D->2D
void Camera::project(const Landmark& lmk, Keypoint& kp)
{
  const double x = lmk(0);
  const double y = lmk(1);
  const double z = lmk(02);

  // lmks should be in camera frame P_c in R3
  double u = (x * fx) / z + cu;
  double v = (y * fy) / z + cv;

  kp(0) = u;
  kp(1) = v;
}

void Camera::backProject(const Keypoint& kp, const Depth& depth, Landmark& lmk)
{
  const double u = kp(0);
  const double v = kp(1);
  const double z = depth;

  const double x = (u - cu) * z * 1.0 / fx;
  const double y = (v - cv) * z * 1.0 / fy;

  lmk(0) = x;
  lmk(1) = y;
  lmk(2) = z;
}

void Camera::print() const
{
  LOG(INFO) << "Camera:\n"
            << "fx: " << fx << "\n"
            << "fy: " << fy << "\n"
            << "cu: " << cu << "\n"
            << "cv: " << cv << "\n"
            << "camera pose: " << camera_pose;
}

}  // namespace VDO_SLAM