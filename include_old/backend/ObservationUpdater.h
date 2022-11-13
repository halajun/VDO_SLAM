#pragma once 


#include "utils/macros.h"
#include "utils/types.h"
#include "backend/Observation.h"

#include <gtsam/nonlinear/Values.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/Pose3.h>

#include <sstream>


namespace VDO_SLAM {

// //forward
// class Map;

// template<typename T>
// class AbstracUpdater {
// public:
//     using Ptr = std::shared_ptr<AbstracUpdater<T>>;

//     AbstracUpdater() {}
//     virtual ~AbstracUpdater() = default;

//     T operator()(FrameId frame_id, FeatureId feature_id) {
//         return update(frame_id, feature_id);
//     }

//     virtual T update(FrameId frame_id, FeatureId feature_id) = 0;
// };


// class StaticPointUpdater : public AbstracUpdater<gtsam::Point3> {

// public:
//     StaticPointUpdater

// };


}