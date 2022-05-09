#pragma once


#include "utils/macros.h"

#include <glog/logging.h>
#include <opencv2/core/core.hpp>



namespace VDO_SLAM {

//Display params for both 2d and 3d display
struct DisplayParams {
    VDO_SLAM_POINTER_TYPEDEFS(DisplayParams);

    bool use_2d_viz = true;
    bool use_3d_viz = false;

    //2D viz params
    //display the 4 input information
    bool display_input = true;
    //visuazlise the per frame information
    bool display_frame = true;

    void print() const;

    static DisplayParams::Ptr loadFromCvFileStorage(const cv::FileStorage& fs);
    static bool cvFileNodeBoolCast(const cv::FileNode& node);

};

} //VDO_SLAM