#pragma once

#include "FactorGraphManager.h"
#include "factors/Point3DFactor.h"
#include "utils/Timing.h"

#include <glog/logging.h>

namespace vdo
{
FactorGraphManager::FactorGraphManager(const BackendParams& params) : params_(params)
{
  setupNoiseModels(params_);
}

gtsam::Key FactorGraphManager::addStaticLandmark(const size_t tracklet_id, const gtsam::Key& pose_key,
                                                 const gtsam::Point3 lmk_w)
{
  // if not in graph, add to values -> else just add new factor
  gtsam::Symbol landmark_symbol(kSymbolStaticPoint3Key, tracklet_id);
  if (!isKeyInGraph(landmark_symbol))
  {
    // add value
    new_values_.insert(landmark_symbol, lmk_w);
  }
  // add the factor regarldess
  graph_.emplace_shared<Point3DFactor>(pose_key, landmark_symbol, lmk_w, point3DNoiseModel);
}

gtsam::Key FactorGraphManager::addCameraPose(const gtsam::Key frame, const gtsam::Pose3& pose)
{
  gtsam::Symbol pose_symbol(kSymbolCameraPose3Key, frame);
  new_values_.insert(pose_symbol, pose);
}

gtsam::Key FactorGraphManager::addCameraPosePrior(const gtsam::Key frame, const gtsam::Pose3& pose)
{
  gtsam::Symbol pose_symbol(kSymbolCameraPose3Key, frame);
  graph_.push_back(gtsam::PriorFactor<gtsam::Pose3>(pose_symbol, pose, cameraPosePrior));
}

gtsam::Key FactorGraphManager::addBetweenFactor(const gtsam::Key from_frame, const gtsam::Key to_frame,
                                                const gtsam::Pose3& odometry)
{
  gtsam::Symbol from_pose_symbol(kSymbolCameraPose3Key, from_frame);
  gtsam::Symbol to_pose_symbol(kSymbolCameraPose3Key, to_frame);

  graph_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(from_pose_symbol, to_pose_symbol, odometry,
                                                            odometryNoiseModel);
}

bool FactorGraphManager::isKeyInGraph(const gtsam::Key key) const
{
  if (smoother_->valueExists(key) || new_values_.exists(key))
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool FactorGraphManager::optimize()
{
  auto tic_update = utils::Timer::tic();
  bool smoother_ok = updateSmoother(graph_, new_values_);
  auto update_duration = utils::Timer::toc(tic_update);

  std::int64_t update_time = update_duration.count();

  static constexpr int kUpdateWarningTimeMs = 100;
  if (update_time > kUpdateWarningTimeMs)
  {
    LOG(WARNING) << "Smoother update " << update_duration.count() << " ms.";
  }

  if (smoother_ok)
  {
    // update state
    state_ = smoother_->calculateEstimate();
  }

  else
  {
    LOG(WARNING) << "Smoother not ok - States have not been updated for frame " << state_key_;
  }

  return smoother_ok;
}

bool FactorGraphManager::updateSmoother(const gtsam::NonlinearFactorGraph& new_factors, const gtsam::Values& new_values)
{
  try
  {
    smoother_result_ = smoother_->update(new_factors, new_values);
    return true;
  }
  catch (const gtsam::ValuesKeyDoesNotExist& e)
  {
    LOG(ERROR) << e.what();
    const gtsam::Key& var = e.key();
    gtsam::Symbol symb(var);

    LOG(ERROR) << "ERROR: Variable has type '" << symb.chr() << "' "
               << "and index " << symb.index() << std::endl;
    return false;
  }
  catch (const gtsam::IndeterminantLinearSystemException& e)
  {
    LOG(WARNING) << e.what();
    const gtsam::Key& var = e.nearbyVariable();
    gtsam::Symbol symb(var);

    LOG(ERROR) << "ERROR: Variable has type '" << symb.chr() << "' "
               << "and index " << symb.index() << std::endl;
    return false;
  }
}

void FactorGraphManager::setupNoiseModels(const BackendParams& params)
{
  gtsam::noiseModel::Base::shared_ptr huberObjectMotion;
  gtsam::noiseModel::Base::shared_ptr huberPoint3D;
  LOG(INFO) << "Setting up noise models for backend";
  cameraPosePrior = gtsam::noiseModel::Isotropic::Sigma(6u, params.var_camera_prior);

  point3DNoiseModel = gtsam::noiseModel::Isotropic::Sigma(3u, params.var_3d_static);

  dynamicPoint3DNoiseModel = gtsam::noiseModel::Isotropic::Sigma(3u, params.var_3d_dyn);

  odometryNoiseModel = gtsam::noiseModel::Isotropic::Sigma(6u, params.var_camera);

  objectMotionNoiseModel = gtsam::noiseModel::Isotropic::Sigma(3u, params.var_obj);

  objectMotionSmootherNoiseModel = gtsam::noiseModel::Isotropic::Sigma(6u, params.var_obj_smooth);

  if (params.use_robust_kernel)
  {
    LOG(INFO) << "Using robust kernal";
    // assuming that this doesnt mess with with original pointer as we're reassigning the member ptrs
    auto pose3dNoiseModelTemp = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(params.k_huber_3d_points), point3DNoiseModel);

    point3DNoiseModel = pose3dNoiseModelTemp;

    objectMotionNoiseModel = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(params.k_huber_obj_motion), objectMotionNoiseModel);
  }
}

}  // namespace vdo