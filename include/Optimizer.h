#pragma once

#include "Frame.h"

namespace vdo {

struct PoseOptimizationFlow2Cam {

    void operator()(Frame::Ptr previous_frame_, Frame::Ptr current_frame_);

};

}