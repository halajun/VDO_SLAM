#pragma once

#include <gtsam/geometry/Pose3.h>

namespace vdo
{
static inline void calculatePoseError(const gtsam::Pose3& estimated, const gtsam::Pose3& ground_truth, double& t_error,
                                      double& rot_error)
{
  const gtsam::Pose3 pose_change_error = estimated.inverse() * ground_truth;

  const gtsam::Point3& translation_error = pose_change_error.translation();
  // L2 norm - ie. magnitude
  t_error = translation_error.norm();

  const gtsam::Matrix33 rotation_error = pose_change_error.rotation().matrix();
  double trace_ate = 0;
  for (int j = 0; j < 3; ++j)
  {
    if (rotation_error(j, j) > 1.0)
    {
      trace_ate = trace_ate + 1.0 - (rotation_error(j, j) - 1.0);
    }
    else
    {
      trace_ate = trace_ate + rotation_error(j, j);
    }
  }
  rot_error = acos((trace_ate - 1.0) / 2.0) * 180.0 / 3.1415926;
}

}  // namespace vdo