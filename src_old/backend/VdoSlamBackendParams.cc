#include "backend/VdoSlamBackendParams.h"
#include "utils/ParamParser.h"

#include <glog/logging.h>
#include <opencv2/core/core.hpp>

namespace VDO_SLAM
{
void BackendParams::print() const
{
  LOG(INFO) << "\n -----------Backend Params ---------- \n"
            << " var_3d_static: " << var_3d_static << "\n"
            << " var_camera: " << var_camera << "\n"
            << " var_obj_smooth: " << var_obj_smooth << "\n"
            << " var_obj: " << var_obj << "\n"
            << " var_3d_dyn: " << var_3d_dyn << "\n"
            << " var_camera_prior: " << var_camera_prior << "\n"
            << " use_robust_kernel: " << use_robust_kernel
            << "\n"
            //   << " k_huber_cam_motion: " << k_huber_cam_motion << "\n"
            << " k_huber_obj_motion: " << k_huber_obj_motion << "\n"
            << " k_huber_3d_points: " << k_huber_3d_points;
}

BackendParams::Ptr BackendParams::loadFromParamParser(const utils::ParamParser& pp)
{
  BackendParams::Ptr params = std::make_shared<BackendParams>();
  pp.getParam("Backend.var_3d_static", &params->var_3d_static);
  pp.getParam("Backend.var_camera", &params->var_camera);
  pp.getParam("Backend.var_obj_smooth", &params->var_obj_smooth);
  pp.getParam("Backend.var_obj", &params->var_obj);
  pp.getParam("Backend.var_3d_dyn", &params->var_3d_dyn);
  pp.getParam("Backend.var_camera_prior", &params->var_camera_prior);

  pp.getParam("Backend.use_robust_kernel", &params->use_robust_kernel);
  // pp.getParam("Backend.k_huber_cam_motion", &params->k_huber_cam_motion);
  pp.getParam("Backend.k_huber_obj_motion", &params->k_huber_obj_motion);
  pp.getParam("Backend.k_huber_3d_points", &params->k_huber_3d_points);
  return params;
}

}  // namespace VDO_SLAM