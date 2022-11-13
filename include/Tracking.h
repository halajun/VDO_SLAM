#pragma once

#include "Macros.h"
#include "Camera.h"
#include "ORBextractor.h"

namespace vdo {



struct TrackingParams {
    //tracking points params
    size_t max_tracking_points_bg;
    size_t max_tracking_points_obj;

    //scene flow thresholds
    double scene_flow_magnitude;
    double scene_flow_percentage;

    //depth thresholds
    double depth_background_thresh;
    double depth_obj_thresh;

    double depth_scale_factor;

    //ORB detector params
    int n_features;
    double scale_factor;
    int n_levels;
    int init_threshold_fast;
    int min_threshold_fast;
};

class Tracking {

public:
    VDO_POINTER_TYPEDEFS(Tracking);

    enum class State {
        kBoostrap,
        kNominal
    };

    Tracking(const TrackingParams& params_, const Camera& camera_);

    gtsam::Pose3 process(const InputPacket& input, 
        GroundTruthInputPacket::ConstOptional ground_truth = boost::none);

private:

    gtsam::Pose3 processBoostrap(const InputPacket& input, 
        GroundTruthInputPacket::ConstOptional ground_truth = boost::none);

    gtsam::Pose3 processNominal(const InputPacket& input, 
        GroundTruthInputPacket::ConstOptional ground_truth = boost::none);


private:
    TrackingParams params;
    Camera camera;

    ORBextractor::UniquePtr feature_detector;

    State state { State::kBoostrap };



};

}