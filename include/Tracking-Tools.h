#pragma once

#include "Macros.h"
#include "Types.h"

#include <gtsam/base/Vector.h>



namespace vdo {

class Frame;

gtsam::Vector3 obtainFlowDepth(const Frame& frame, TrackletId tracklet_id);

void determineOutlierIds(const TrackletIds& inliers, const TrackletIds& tracklets, TrackletIds& outliers);

}