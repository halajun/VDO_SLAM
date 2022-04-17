#include "FactorGraph.h"

namespace VDO_SLAM {

FactorGraph::FactorGraph(Map* map_, const cv::Mat& Calib_K_)
    :   map(map_),
        Calib_K(Calib_K_) {}


void FactorGraph::step() {
    
}

};