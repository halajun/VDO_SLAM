#include "Tracking.h"

namespace vdo {

Tracking::Tracking(const TrackingParams& params_, const Camera& camera_)
: params(params_), camera(camera_) 
{

    feature_detector = vdo::make_unique<ORBextractor>(
        params.n_features,
        static_cast<float>(params.scale_factor),
        params.n_levels,
        params.init_threshold_fast,
        params.min_threshold_fast
    );
}

gtsam::Pose3 Tracking::process(const InputPacket& input, GroundTruthInputPacket::ConstOptional ground_truth) {
    if(state == State::kBoostrap) {
        return processBoostrap(input, ground_truth);
    }
    else if(state == State::kNominal) {
        return processNominal(input, ground_truth);
    }
}


} //vdo