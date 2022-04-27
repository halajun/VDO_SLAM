#pragma once

#include "utils/macros.h"

namespace VDO_SLAM {

class Display {

    public:
        VDO_SLAM_POINTER_TYPEDEFS(Display);

        Display() = default;
        virtual ~Display() = default;

        virtual void process() = 0;


};

} //VDO_SLAM