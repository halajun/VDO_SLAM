#pragma once 


#include "utils/macros.h"
#include "utils/types.h"

#include <gtsam/nonlinear/Values.h>

namespace VDO_SLAM {

//only works for single "thing" observation...
//eg gtsam::Point3
template<class T>
class Observation {
    public:
        using Ptr = std::shared_ptr<Observation<T>>;

        Observation(const FrameId& frame_id_,  const FeatureId& point_id_, int position_, int tracklet_id_)
            :   point_id(point_id_),
                frame_id(frame_id_),
                tracklet_position(position_),
                tracklet_id(tracklet_id_) {}

        // T measurement;
        const FrameId frame_id;
        const FeatureId point_id;
        const int tracklet_position;
        const int tracklet_id;
        //non-explicit gtsam key
        int key = -1;
        bool was_added = false;

};



}