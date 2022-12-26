#pragma once

#include "Types.h"
#include "Frame.h"
#include "Camera.h"
#include "Frontend-Definitions.h"
#include "Backend-Definitions.h"
#include "FactorGraphManager.h"
#include "Frontend-Definitions.h"

namespace vdo
{
class FrontendOutput;

struct PoseOptimizationFlow2Cam
{
  PoseOptimizationFlow2Cam(const Camera& camera);
  
  void operator()(Frame::Ptr previous_frame, Frame::Ptr current_frame);

  const Camera camera_;
};

class IncrementalOptimizer : public FactorGraphManager
{
public:
  VDO_POINTER_TYPEDEFS(IncrementalOptimizer);

  // pair of tracklet ID -> frame ID so we can find the features later
  using FeatureTrackletIdPair = std::pair<size_t, size_t>;
  // gtsam key of the value in the graph to some meta data about the original observed feature
  using KeyToFeatureMap = std::map<gtsam::Key, FeatureTrackletIdPair>;
  // tracklet ids to features vector
  using TrackletsMap = std::map<size_t, Features>;

  IncrementalOptimizer(const BackendParams& params, const Camera& camera);

  BackendOutput::Ptr process(const FrontendOutput& input);

private:
  BackendOutput::Ptr processBoostrap(const FrontendOutput& input);
  BackendOutput::Ptr processNominal(const FrontendOutput& input);

  void handleStaticFeatures(const FeaturePtrs& static_features);
  // assume inliers
  void collectFeature(const Feature& feature);

  // helper function
  gtsam::Key addLandmarkToGraph(const size_t tracklet_id, const size_t frame_id, const Landmark& lmk_camera);

private:
  Camera camera_;
  State state{ State::kBoostrap };

  KeyToFeatureMap static_feature_slots_;   // all keys here should be in the isam graph and correlate to a staitc lmk
                                           // being optimzised
  TrackletsMap unadded_static_tracklets_;  // map of tracklets and used to update the map. the length should correspond
                                           // to the features age

  // debug stats per framer
  int num_static_features_added = 0;
};

}  // namespace vdo