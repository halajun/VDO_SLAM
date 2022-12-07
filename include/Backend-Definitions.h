#pragma once

#include "Macros.h"
#include <glog/logging.h>

#include <gtsam/geometry/Pose3.h>

namespace vdo
{
struct BackendParams
{
  VDO_POINTER_TYPEDEFS(BackendParams);

  // sigmas^2 (variance) for covariances. Will be (NOT) be divided by 1 for the covariances
  // TODO: make sure we're consistent with diviing by 1 as some are and somare are not
  // in the original code and this is confusing
  // variance used for 3d static points
  double var_static_lmk = 0.0625;  // 1/50 1/80 1/16
  // variance used for camera motion factors (odom)
  double var_odom = 1000;  // 1/0.005 1/0.001 1/0.0001
  // variance used for smooth object motion factors
  double var_obj_motion_smooth = 0.001;  // 0.1 0.5 (ox:) 0.001
  // variance used for object motion factors
  double var_obj_motion = 100;  // 0.5 1 10 20 50 (ox:) 100
  // variance used for 3d dynamic point factorss
  double var_dynamic_lmk = 80;  // 50 100 16 (ox:) 80

  double var_camera_prior = 0.000001;

  // if true will use robust Huber noise model
  bool use_robust_kernel = true;
  // used to initalise the various robust huber kernals (only if true)
  // double k_huber_cam_motion = 0.001; #dont want to apply to camera motion as these should not have the same model
  double k_huber_obj_motion = 0.001;
  double k_huber_3d_points = 0.001;

  inline void print() const
  {
    LOG(INFO) << "\n -----------Backend Params ---------- \n"
              << " var_static_lmk: " << var_static_lmk << "\n"
              << " var_odom: " << var_odom << "\n"
              << " var_obj_motion_smooth: " << var_obj_motion_smooth << "\n"
              << " var_obj_motion: " << var_obj_motion << "\n"
              << " var_dynamic_lmk: " << var_dynamic_lmk << "\n"
              << " var_camera_prior: " << var_camera_prior << "\n"
              << " use_robust_kernel: " << use_robust_kernel
              << "\n"
              //   << " k_huber_cam_motion: " << k_huber_cam_motion << "\n"
              << " k_huber_obj_motion: " << k_huber_obj_motion << "\n"
              << " k_huber_3d_points: " << k_huber_3d_points;
  }
};

struct BackendOutput
{
  VDO_POINTER_TYPEDEFS(BackendOutput);
  gtsam::Pose3 estimated_pose_;
};

}  // namespace vdo
