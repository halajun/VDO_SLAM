#include "visualizer/DisplayParams.h"
#include "utils/UtilsOpenCv.h"

#include <glog/logging.h>
#include <opencv2/core/core.hpp>

#include <fstream>
#include <iostream>
#include <memory>

namespace VDO_SLAM {

void DisplayParams::print() const {
    std::stringstream out;
    out << "\n -----------Display Params ---------- \n";
    out << " Using 2D viz " << use_2d_viz << "\n";
    out << " Using 3D viz " << use_3d_viz << "\n";
    out << " Display Input " << display_input << "\n";
    out << " Display Frame " << display_frame << "\n";
    LOG(INFO) << out.str();

}

DisplayParams::Ptr DisplayParams::loadFromParamParser(const utils::ParamParser& pp) {
    DisplayParams::Ptr params = std::make_shared<DisplayParams>();
    pp.getParam("Viz.use_2d", &params->use_2d_viz);
    pp.getParam("Viz.use_3d", &params->use_3d_viz);
    pp.getParam("Viz.displayInput", &params->display_input);
    pp.getParam("Viz.displayFrame", &params->display_frame);
    return params;

}


}