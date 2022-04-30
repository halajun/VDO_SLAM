#include "vizualizer/DisplayParams.h"

#include <glog/logging.h>
#include <opencv2/core/core.hpp>

#include <fstream>
#include <iostream>

namespace VDO_SLAM {

void DisplayParams::print() const {
    std::stringstream out;
    out << " Using 2D viz " << use_2d_viz << "\n";
    out << " Using 2D viz " << use_2d_viz << "\n";

}

}