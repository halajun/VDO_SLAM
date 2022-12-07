#pragma once

#include "FactorGraphManager.h"
#include "factors/Point3DFactor.h"
#include "utils/Timing.h"

#include <glog/logging.h>
#include <boost/foreach.hpp>

namespace vdo
{
FactorGraphManager::FactorGraphManager(const BackendParams& params) : params_(params)
{
  setupNoiseModels(params_);

  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.evaluateNonlinearError = false;
  parameters.relinearizeSkip = 1;

  smoother_ = vdo::make_unique<gtsam::ISAM2>(parameters);
}

gtsam::Key FactorGraphManager::addStaticLandmark(const size_t tracklet_id, const gtsam::Key& pose_key,
                                                 const gtsam::Point3 lmk_c)
{
  gtsam::Pose3 pose;
  if(new_values_.exists(pose_key)) {
    pose = new_values_.at<gtsam::Pose3>(pose_key);
  }
  else if(smoother_->valueExists(pose_key)) {
    pose = smoother_->calculateEstimate(pose_key).cast<gtsam::Pose3>();
  }
  else {
    CHECK(false) << "pose key - " << pose_key << " was not yet in new_values_ or isam graph";
  }

  gtsam::Point3 lmk_world = pose.transformFrom(lmk_c);
  
  // if not in graph, add to values -> else just add new factor
  gtsam::Symbol landmark_symbol = staticLandmarkKey(tracklet_id);
  if (!isKeyInGraph(landmark_symbol))
  {
    // add value
    new_values_.insert(landmark_symbol, lmk_world);
  }
  // add the factor regarldess
  graph_.emplace_shared<Point3DFactor>(pose_key, landmark_symbol, lmk_c, staticLmkNoise_);
  return landmark_symbol;
}

gtsam::Key FactorGraphManager::addCameraPose(const gtsam::Key frame, const gtsam::Pose3& pose)
{
  gtsam::Symbol pose_symbol = poseKey(frame);
  new_values_.insert(pose_symbol, pose);
  return pose_symbol;
}

gtsam::Key FactorGraphManager::addCameraPosePrior(const gtsam::Key frame, const gtsam::Pose3& pose)
{
  gtsam::Symbol pose_symbol = poseKey(frame);
  graph_.push_back(gtsam::PriorFactor<gtsam::Pose3>(pose_symbol, pose, cameraPosePrior_));
  return pose_symbol;
}

void FactorGraphManager::addBetweenFactor(const gtsam::Key from_frame, const gtsam::Key to_frame,
                                                const gtsam::Pose3& odometry)
{
  gtsam::Symbol from_pose_symbol = poseKey(from_frame);
  gtsam::Symbol to_pose_symbol = poseKey(to_frame);

  graph_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(from_pose_symbol, to_pose_symbol, odometry,
                                                            odomNoise_);
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

bool FactorGraphManager::optimize(const gtsam::Key state_key)
{
  auto tic_update = utils::Timer::tic();
  bool smoother_ok = updateSmoother(graph_, new_values_);
  updateSmoother();
  updateSmoother();
  auto update_duration = utils::Timer::toc(tic_update);

  std::int64_t update_time = update_duration.count();

  static constexpr int kUpdateWarningTimeMs = 100;
  if (update_time > kUpdateWarningTimeMs)
  {
    LOG(WARNING) << "Smoother update " << update_duration.count() << " ms.";
  }

  if (smoother_ok)
  {
    //book keeping on the state of the graph
    gtsam::Values state_before_opt = gtsam::Values(state_);
    BOOST_FOREACH (const gtsam::Values::ConstKeyValuePair& key_value, new_values_)
    {
      state_before_opt.insert(key_value.key, key_value.value);
    }
    const double error_before = graph_.error(state_before_opt);
    
    // update state
    state_ = smoother_->calculateBestEstimate();

    const double error_after = graph_.error(state_);
    LOG(INFO) << "Optimization Errors:\n"
                  << "Error before: " << error_before << "\n"
                  << "Error after: " << error_after;

    //clear states
    new_values_.clear();
    graph_.resize(0);
  }

  else
  {
    LOG(WARNING) << "Smoother not ok - States have not been updated for frame " << state_key;
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
    throw;
  }
  catch (const gtsam::IndeterminantLinearSystemException& e)
  {
    LOG(WARNING) << e.what();
    const gtsam::Key& var = e.nearbyVariable();
    gtsam::Symbol symb(var);

    LOG(ERROR) << "ERROR: Variable has type '" << symb.chr() << "' "
               << "and index " << symb.index() << std::endl;
     throw;
  }
  catch (const gtsam::ValuesKeyAlreadyExists& e)
  {
    LOG(WARNING) << e.what();
    const gtsam::Key& var = e.key();
    gtsam::Symbol symb(var);

    LOG(ERROR) << "ERROR: Variable has type '" << symb.chr() << "' "
               << "and index " << symb.index() << std::endl;
     throw;
  }
}

void FactorGraphManager::setupNoiseModels(const BackendParams& params)
{
  gtsam::noiseModel::Base::shared_ptr huberObjectMotion;
  gtsam::noiseModel::Base::shared_ptr huberPoint3D;
  LOG(INFO) << "Setting up noise models for backend";
  cameraPosePrior_ = gtsam::noiseModel::Isotropic::Sigma(6u, params.var_camera_prior);

  staticLmkNoise_ = gtsam::noiseModel::Isotropic::Sigma(3u, params.var_static_lmk);

  dynamicLmkNoise_ = gtsam::noiseModel::Isotropic::Sigma(3u, params.var_dynamic_lmk);

  odomNoise_ = gtsam::noiseModel::Isotropic::Sigma(6u, params.var_odom);

  objectMotionNoise_ = gtsam::noiseModel::Isotropic::Sigma(3u, params.var_obj_motion);

  objectMotionSmoothingNoise_ = gtsam::noiseModel::Isotropic::Sigma(6u, params.var_obj_motion_smooth);

  if (params.use_robust_kernel)
  {
    LOG(INFO) << "Using robust kernal";
    // assuming that this doesnt mess with with original pointer as we're reassigning the member ptrs
    staticLmkNoise_ = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(params.k_huber_3d_points), staticLmkNoise_);
    
    dynamicLmkNoise_ = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(params.k_huber_3d_points), dynamicLmkNoise_);


  //   objectMotionNoiseModel = gtsam::noiseModel::Robust::Create(
  //       gtsam::noiseModel::mEstimator::Huber::Create(params.k_huber_obj_motion), objectMotionNoiseModel);
  }
}

}  // namespace vdo