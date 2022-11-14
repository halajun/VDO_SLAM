#pragma once

#include <Eigen/Core>

#include <opencv2/core.hpp>
#include "utils/macros.h"
#include "utils/types.h"
#include <glog/logging.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Point2.h>

namespace VDO_SLAM
{
class Camera
{
public:
  VDO_SLAM_POINTER_TYPEDEFS(Camera);
  using Depth = double;
  using Landmark = gtsam::Point3;
  using Keypoint = gtsam::Point2;
  using Pose = gtsam::Pose3;

  Camera(const cv::Mat& K_, const gtsam::Pose3 camera_pose_ = gtsam::Pose3::identity());

  // 3D->2D
  void project(const Landmark& lmk, Keypoint& kp);

  void backProject(const Keypoint& kp, const Depth& depth, Landmark& lmk);

  void print() const;

private:
  //! 3x3 matrix of doubles (float64 in opencv land)
  const cv::Mat& K;
  const double fx, fy, cu, cv;
  Pose camera_pose;
};

}  // namespace VDO_SLAM