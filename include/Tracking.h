#pragma once

#include "Macros.h"

namespace vdo {

struct TrackingParams {
    //tracking points params
    size_t max_tracking_points_bg;
    size_t max_tracking_points_obj;

    //scene flow thresholds
    double scene_flow_magnitude;
    double scene_flow_percentage;

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

    Tracking(const TrackingParams& params);

};

}