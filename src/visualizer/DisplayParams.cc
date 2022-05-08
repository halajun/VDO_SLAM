#include "visualizer/DisplayParams.h"

#include <glog/logging.h>
#include <opencv2/core/core.hpp>

#include <fstream>
#include <iostream>
#include <memory>

namespace VDO_SLAM {

void DisplayParams::print() const {
    std::stringstream out;
    out << " Using 2D viz " << use_2d_viz << "\n";
    out << " Using 3D viz " << use_3d_viz << "\n";
    out << " Display Input " << display_input << "\n";
    out << " Display Frame " << display_frame << "\n";
    LOG(INFO) << out.str();

}

DisplayParams::Ptr DisplayParams::loadFromCvFileStorage(const cv::FileStorage& fs) {
    DisplayParams::Ptr params = std::make_shared<DisplayParams>();
    params->use_2d_viz = fs["Viz.use_2d"];
    params->use_3d_viz = fs["Viz.use_3d"];
    params->display_input = fs["Viz.displayInput"];
    params->display_frame = fs["Viz.displayFrame"];
}

}