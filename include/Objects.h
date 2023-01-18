#pragma once

#include "Frame.h"
#include "Types.h"
#include <boost/optional.hpp>
#include <gtsam/geometry/Pose3.h>

#include <vector>

namespace vdo
{
class Camera;

// represents the collection of observations on a single object in a frame
struct ObjectObservation
{
  int object_label_;
  TrackletIds object_features_;
  Frame::Ptr frame_;             // frame with the actual tracklet information in it
  gtsam::Point2 object_center_;  // the average position of the object features
  double average_flow_;          // average flow oover the object

  inline size_t getFrameId() const
  {
    return frame_->frame_id_;
  }

  // collects the landmarks for this object
  // if pose given, the landmarks will be transformed into the frame of the given pose (using Pose3::transformFrom)
  Landmarks collectLandmarks(const Camera& camera, boost::optional<const gtsam::Pose3&> pose = boost::none) const;
  FeaturePtrs collectFeatures() const;
};

using ObjectObservations = std::vector<ObjectObservation>;

}  // namespace vdo
