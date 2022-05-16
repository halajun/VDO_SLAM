#include "backend/VdoSlamBackendParams.h"
#include "utils/UtilsOpenCv.h"

#include <glog/logging.h>
#include <opencv2/core/core.hpp>



namespace VDO_SLAM {

void BackendParams::print() const {
    LOG(INFO) << " -----------Backend Params ---------- \n"
              << " var_3d_static: " << var_3d_static << "\n"
              << " var_camera: " << var_camera << "\n"
              << " var_obj_smooth: " << var_obj_smooth << "\n"
              << " var_obj: " << var_obj << "\n"
              << " var_3d_dyn: " << var_3d_dyn << "\n"
              << " use_robust_kernel: " << use_robust_kernel << "\n"
              << " k_huber_cam_motion: " << k_huber_cam_motion << "\n"
              << " k_huber_obj_motion: " << k_huber_obj_motion << "\n"
              << " k_huber_3d_points: " << k_huber_3d_points;
}

BackendParams::Ptr loadFromCvFileStorage(const cv::FileStorage& fs) {
    BackendParams::Ptr params = std::make_shared<BackendParams>();
    params->var_3d_static = utils::cvFileNodeBoolCast(fs["Backend.var_3d_static"]);
    params->var_camera = utils::cvFileNodeBoolCast(fs["Backend.var_camera"]);
    params->var_obj_smooth = utils::cvFileNodeBoolCast(fs["Backend.var_obj_smooth"]);
    params->var_obj = utils::cvFileNodeBoolCast(fs["Backend.var_obj"]);
    params->var_3d_dyn = utils::cvFileNodeBoolCast(fs["Backend.var_3d_dyn"]);
    params->use_robust_kernel = utils::cvFileNodeBoolCast(fs["Backend.use_robust_kernel"]);
    params->k_huber_cam_motion = utils::cvFileNodeBoolCast(fs["Backend.k_huber_cam_motion"]);
    params->k_huber_obj_motion = utils::cvFileNodeBoolCast(fs["Backend.k_huber_obj_motion"]);
    params->k_huber_3d_points = utils::cvFileNodeBoolCast(fs["Backend.k_huber_3d_points"]);
    return params;
}

} //VDO_SLAM