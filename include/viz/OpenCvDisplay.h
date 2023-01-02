#pragma once

#include "Macros.h"
#include "Types.h"
#include "viz/Color.h"
#include "viz/Display.h"
#include "viz/DisplayTypes.h"

#include <opencv2/opencv.hpp>
#include <glog/logging.h>
#include <vector>

namespace vdo
{
class Frame;

class OpenCvDisplay : public Display
{
public:
  VDO_POINTER_TYPEDEFS(OpenCvDisplay);

  OpenCvDisplay(DisplayParams::Ptr params_);
  ~OpenCvDisplay() = default;

  void process(const VisualiserInput& viz_input) override;

  static void drawOpticalFlow(const cv::Mat& flow, cv::Mat& flow_viz);
  static void drawSemanticInstances(const cv::Mat& rgb, const cv::Mat& mask, cv::Mat& mask_viz);

private:
  void drawFrame(const Frame& input);
  void drawInputImages(const Frame& frame);

  void drawFeatures(const cv::Mat& rgb, const Frame& frame, cv::Mat& frame_viz);

  void addDisplayImages(const cv::Mat& image, const std::string& title);

private:
  ImageToDisplay::Vector display_images;

  static Color getColourFromInstanceMask(int value);
};

}  // namespace vdo