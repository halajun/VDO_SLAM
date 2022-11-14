#pragma once

#include "utils/macros.h"
#include "utils/ParamParser.h"

#include <glog/logging.h>
#include <opencv2/core/core.hpp>

namespace VDO_SLAM
{
struct BackendParams
{
  VDO_SLAM_POINTER_TYPEDEFS(BackendParams);

  // sigmas^2 (variance) for covariances. Will be (NOT) be divided by 1 for the covariances
  // TODO: make sure we're consistent with diviing by 1 as some are and somare are not
  // in the original code and this is confusing
  // variance used for 3d static points
  double var_3d_static = 0.0625;  // 1/50 1/80 1/16
  // variance used for camera motion factors (odom)
  double var_camera = 1000;  // 1/0.005 1/0.001 1/0.0001
  // variance used for smooth object motion factors
  double var_obj_smooth = 0.001;  // 0.1 0.5 (ox:) 0.001
  // variance used for object motion factors
  double var_obj = 100;  // 0.5 1 10 20 50 (ox:) 100
  // variance used for 3d dynamic point factorss
  double var_3d_dyn = 80;  // 50 100 16 (ox:) 80

  double var_camera_prior = 0.000001;

  // if true will use robust Huber noise model
  bool use_robust_kernel = true;
  // used to initalise the various robust huber kernals (only if true)
  // double k_huber_cam_motion = 0.001; #dont want to apply to camera motion as these should not have the same model
  double k_huber_obj_motion = 0.001;
  double k_huber_3d_points = 0.001;

  void print() const;

  static BackendParams::Ptr loadFromParamParser(const utils::ParamParser& pp);
};

}  // namespace VDO_SLAM
