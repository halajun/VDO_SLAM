#pragma once 


#include "utils/macros.h"
#include "utils/types.h"

#include <gtsam/nonlinear/Values.h>

namespace VDO_SLAM {

//only works for single "thing" observation...
template<class T>
class Observation {
    //All public... for now
    public:
        VDO_SLAM_POINTER_TYPEDEFS(Observation);

        Observation(const T& value_, const gtsam::Key& key_, const gtsam::Key& pose_key_, const FrameId& curr_frame_)
            :   measurement(value_),
                key(key_),
                pose_key(pose_key_),
                curr_frame(curr_frame_) {}

        T measurement;
        gtsam::Key key;
        gtsam::Key pose_key;
        FrameId curr_frame;

};

}