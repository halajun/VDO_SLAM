#pragma once

#include "utils/macros.h"
#include "utils/Color.h"
#include "visualizer/Display.h"
#include "visualizer/DisplayTypes.h"
#include "Frame.h"

#include <opencv2/opencv.hpp>
#include <glog/logging.h>


namespace VDO_SLAM { 

class OpenCvDisplay : public Display {

    public:
        VDO_SLAM_POINTER_TYPEDEFS(OpenCvDisplay);

        OpenCvDisplay(DisplayParams::Ptr params_);
        ~OpenCvDisplay() = default;

        void addFrame(const Frame& frame);
        void process() override;

    private:
        void drawInputImages(const Frame& frame);
        void drawFrame(const Frame& frame);

        void drawOpticalFlow(const cv::Mat& flow, cv::Mat& flow_viz);
        void drawSemanticInstances(const cv::Mat& rgb, const cv::Mat& mask, cv::Mat& mask_viz);
        void drawFeatures(const cv::Mat& rgb, const Frame& frame, cv::Mat& frame_viz);

        void addDisplayImages(const cv::Mat& image, const std::string& title);

    private:
        ImageToDisplay::Vector display_images;

        static Color getColourFromInstanceMask(int value);
};
    
} // namespace VDO_SLAM 