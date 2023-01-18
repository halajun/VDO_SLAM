#include "Objects.h"
#include "Frame.h"
#include "Types.h"
#include "Camera.h"

#include <vector>
#include <glog/logging.h>

namespace vdo
{

Landmarks ObjectObservation::collectLandmarks(const Camera& camera, boost::optional<const gtsam::Pose3&> pose) const
{
  FeaturePtrs features = collectFeatures();
  Landmarks landmarks;
  for (const Feature::Ptr& feature : features)
  {
    Landmark lmk;
    camera.backProject(feature->keypoint, feature->depth, &lmk);

    if (pose)
    {
      lmk = pose->transformFrom(lmk);
    }
    landmarks.push_back(lmk);
  }
  return landmarks;
}
FeaturePtrs ObjectObservation::collectFeatures() const
{
  FeaturePtrs features;
  CHECK(frame_);
  for (TrackletId tracklet_id : object_features_)
  {
    Feature::Ptr feature = frame_->getDynamicByTrackletId(tracklet_id);
    CHECK(feature);
    features.push_back(feature);
  }
  return features;
}

}  // namespace vdo
