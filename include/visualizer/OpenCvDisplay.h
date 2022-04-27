#pragma once

#include "utils/macros.h"
#include "visualizer/Display.h"
#include "Frame.h"

#include <opencv2/opencv.hpp>
#include <glog/logging.h>


namespace VDO_SLAM { 

class OpenCvDisplay : public Display {

    public:
        VDO_SLAM_POINTER_TYPEDEFS(OpenCvDisplay);

        OpenCvDisplay() = default;
        ~OpenCvDisplay() = default;

        void addFrame(const Frame& frame);
        void process() override;

    private:
        void drawOpticalFlow(cv::Mat& flow);
        void drawSemanticInstances(cv::Mat& mask);
        void drawFeatures(cv::Mat& )
};
    
} // namespace VDO_SLAM 