#pragma once

#include "Map.h"

namespace VDO_SLAM {

class FactorGraph {

    public:
        FactorGraph(Map* map_, const cv::Mat& Calib_K_);

        void step();
        void printStats();

    
        Map* map;
        const cv::Mat Calib_K;

    private:
        //number of times step has been called
        int steps;

};


} //VDO_SLAM