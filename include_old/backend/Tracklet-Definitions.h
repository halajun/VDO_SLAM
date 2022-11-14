#pragma once

#include "backend/Observation.h"
#include "utils/macros.h"
#include "utils/types.h"

#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <glog/logging.h>
#include <vector>

namespace VDO_SLAM
{
// template<size_t N>
// inline gtsam:: Tracklet::convert(FrameId frame_id, FeatureId point_id, Map* map);
using StaticTrackletManager = TrackletManager<gtsam::Point3, 3>;
using DynamicTrackletManager = TrackletManager<gtsam::Point3, 3>;

}  // namespace VDO_SLAM