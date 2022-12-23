#include "Optimizer.h"
#include "FrontendOutput.h"
#include "utils/Metrics.h"

// #include "dependencies/g2o/g2o/types/vertex_se3.h"
// #include "dependencies/g2o/g2o/types/vertex_pointxyz.h"
// #include "dependencies/g2o/g2o/types/edge_se3.h"
// #include "dependencies/g2o/g2o/types/edge_se3_pointxyz.h"
// #include "dependencies/g2o/g2o/types/edge_se3_prior.h"
// #include "dependencies/g2o/g2o/core/block_solver.h"
// #include "dependencies/g2o/g2o/core/optimization_algorithm_levenberg.h"
// #include "dependencies/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
// #include "dependencies/g2o/g2o/solvers/linear_solver_csparse.h"
// #include "dependencies/g2o/g2o/solvers/linear_solver_eigen.h"
// #include "dependencies/g2o/g2o/solvers/linear_solver_dense.h"

namespace vdo
{
IncrementalOptimizer::IncrementalOptimizer(const BackendParams& params, const Camera& camera)
  : FactorGraphManager(params), camera_(camera)
{
  params_.print();
}

BackendOutput::Ptr IncrementalOptimizer::process(const FrontendOutput& input)
{
  BackendOutput::Ptr output = nullptr;
  if (state == State::kBoostrap)
  {
    output = processBoostrap(input);
  }
  else if (state == State::kNominal)
  {
    output = processNominal(input);
  }

  return output;
}

BackendOutput::Ptr IncrementalOptimizer::processBoostrap(const FrontendOutput& input)
{
  LOG(INFO) << input.frame_id_;
  gtsam::Key state_key = input.frame_id_;
  LOG(INFO) << state_key;
  const gtsam::Pose3 estimated_pose = input.estimated_pose_;
  const GroundTruthInputPacket::ConstOptional ground_truth = input.ground_truth_;

  const Frame::Ptr& frame = input.frame_;

  // first pose so we add pose prior
  addCameraPose(state_key, estimated_pose);
  addCameraPosePrior(state_key, estimated_pose);

  // handleStaticFeatures(frame->static_features);
  // odometry

  optimize(state_key);

  state = State::kNominal;
  return nullptr;
}

BackendOutput::Ptr IncrementalOptimizer::processNominal(const FrontendOutput& input)
{
  LOG(INFO) << input.frame_id_;
  gtsam::Key state_key = input.frame_id_;
  LOG(INFO) << state_key;
  CHECK_GE(state_key, 0u);
  const gtsam::Pose3 estimated_pose = input.estimated_pose_;
  const GroundTruthInputPacket::ConstOptional ground_truth = input.ground_truth_;
  const Frame::Ptr& frame = input.frame_;

  double t_error_before_opt, r_error_before_opt, t_error_after_opt, r_error_after_opt;
  calculatePoseError(estimated_pose, ground_truth->X_wc, t_error_before_opt, r_error_before_opt);

  // first pose so we add pose prior
  gtsam::Key pose_key = addCameraPose(state_key, estimated_pose);
  // handleStaticFeatures(frame->static_features);

  // // TODO:for now!
  // // get prior camera motion
  // gtsam::Key prev_pose_key = poseKey(state_key - 1);
  // gtsam::Pose3 previous_pose = state_.at<gtsam::Pose3>(prev_pose_key);
  // gtsam::Pose3 odometry = previous_pose.inverse() * estimated_pose;

  // addBetweenFactor(state_key - 1, state_key, odometry);

  // optimize(state_key);

  // LOG(INFO) << "Num features added " << num_static_features_added;
  // // get updated pose
  // gtsam::Pose3 best_pose = state_.at<gtsam::Pose3>(pose_key);

  // LOG(INFO) << "Opt pose\n" << best_pose;

  // calculatePoseError(best_pose, ground_truth->X_wc, t_error_after_opt, r_error_after_opt);
  // LOG(INFO) << std::fixed << "ATE Errors:\n"
  //           << "Error before: t - " << t_error_before_opt << ", r - " << r_error_before_opt << "\n"
  //           << "Error after: t - " << t_error_after_opt << ", r - " << r_error_after_opt << "\n";

  num_static_features_added = 0;
  BackendOutput::Ptr output = std::make_shared<BackendOutput>();
  // output->estimated_pose_ = best_pose;
  return output;
}

void IncrementalOptimizer::handleStaticFeatures(const TrackletIdFeatureMap& static_features)
{
  // this should handle all new features currently in the map
  for (const auto& tracklet_id_feature : static_features)
  {
    const Feature& feature = *tracklet_id_feature.second;
    if (feature.inlier)
    {
      const size_t tracklet_id = feature.tracklet_id;
      const size_t frame_id = feature.frame_id;

      gtsam::Key pose_key = poseKey(frame_id);
      // LOG(INFO) << pose_key;
      gtsam::Symbol symb(pose_key);

      // LOG(INFO) << "pose symbol '" << symb.chr() << "' "
      //           << "and index " << symb.index();

      // if the landmark is already in the map add it directly
      gtsam::Key potential_landmark_key = staticLandmarkKey(tracklet_id);
      // gtsam::Symbol plk(potential_landmark_key);
      // LOG(INFO) << "pose symbol '" << plk.chr() << "' "
      //           << "and index " << plk.index();
      if (isKeyInGraph(potential_landmark_key))
      {
        Landmark lmk_camera;
        camera_.backProject(feature.keypoint, feature.depth, &lmk_camera);
        gtsam::Key lmk_key = addLandmarkToGraph(tracklet_id, frame_id, lmk_camera);
        // sanity check
        CHECK_EQ(potential_landmark_key, lmk_key);
      }
      else
      {
        // add it to the feature map
        collectFeature(feature);
      }
    }
  }

  // LOG(INFO) <<
  // go through tracklets and check length (these should correspond to a landmarks age)
  static constexpr size_t kMinTrackletLength = 1u;
  std::vector<size_t> tracklet_ids_to_delete;  // which tracklets have now been added to the map
  for (const auto& [tracklet_id, features] : unadded_static_tracklets_)
  {
    gtsam::Key potential_landmark_key = staticLandmarkKey(tracklet_id);
    gtsam::Symbol plk(potential_landmark_key);
    CHECK(!isKeyInGraph(potential_landmark_key))
        << "key " << plk.chr() << " index " << plk.index() << " should not be in the graph yet!!";

    if (features.size() >= kMinTrackletLength)
    {
      // go through all the features and add them
      for (const auto& feature : features)
      {
        const size_t frame_id = feature.frame_id;
        // sanity check
        CHECK_EQ(tracklet_id, feature.tracklet_id);
        Landmark lmk_camera;
        camera_.backProject(feature.keypoint, feature.depth, &lmk_camera);
        gtsam::Symbol lmk_key = addLandmarkToGraph(tracklet_id, frame_id, lmk_camera);
        // sanity check
        CHECK_EQ(potential_landmark_key, lmk_key);

        // mark this tracklet as added
        tracklet_ids_to_delete.push_back(tracklet_id);
      }
    }
  }

  // remove landmakrs from the map as they should all now appear in the graph
  for (size_t ids_to_remove : tracklet_ids_to_delete)
  {
    gtsam::Key lmk_key = staticLandmarkKey(ids_to_remove);
    // sanity check
    CHECK(isKeyInGraph(lmk_key));
    unadded_static_tracklets_.erase(ids_to_remove);
  }
}

// landmarks in camera frame
void IncrementalOptimizer::collectFeature(const Feature& feature)
{
  CHECK(feature.inlier);
  const size_t& tracklet_id = feature.tracklet_id;
  // if not already in tracklet map, create new vector and add
  if (unadded_static_tracklets_.find(tracklet_id) != unadded_static_tracklets_.end())
  {
    unadded_static_tracklets_.at(tracklet_id).push_back(feature);
  }
  else
  {
    unadded_static_tracklets_.insert({ tracklet_id, { feature } });
  }
}

gtsam::Key IncrementalOptimizer::addLandmarkToGraph(const size_t tracklet_id, const size_t frame_id,
                                                    const Landmark& lmk_camera)
{
  gtsam::Key pose_key = poseKey(frame_id);
  gtsam::Symbol lmk_key = addStaticLandmark(tracklet_id, pose_key, lmk_camera);
  static_feature_slots_.insert({ lmk_key, std::make_pair(tracklet_id, frame_id) });

  num_static_features_added++;

  return lmk_key;
}

}  // namespace vdo