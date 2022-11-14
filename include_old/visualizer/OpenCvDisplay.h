#pragma once

#include "utils/macros.h"
#include "utils/types.h"
#include "utils/Color.h"
#include "visualizer/Display.h"
#include "visualizer/DisplayTypes.h"
#include "frontend/Frame.h"
#include "backend/Tracklet.h"
#include "backend/Tracklet-Definitions.h"

#include <opencv2/opencv.hpp>
#include <glog/logging.h>
#include <vector>

namespace VDO_SLAM
{
class Map;

struct Display2DInput
{
  Frame frame;
  FeatureTrackletMatrix static_tracklets;
  Map* map;
};

class OpenCvDisplay : public Display
{
public:
  VDO_SLAM_POINTER_TYPEDEFS(OpenCvDisplay);

  OpenCvDisplay(DisplayParams::Ptr params_);
  ~OpenCvDisplay() = default;

  void addInput(const Display2DInput& input);
  void process() override;

private:
  void drawFrame(const Display2DInput& input);
  void drawInputImages(const Frame& frame);

  void drawTracklet(const cv::Mat& rgb, cv::Mat& rgb_tracks, const FeatureTrackletMatrix& static_tracks,
                    const Map* map);

  void drawOpticalFlow(const cv::Mat& flow, cv::Mat& flow_viz);
  void drawSemanticInstances(const cv::Mat& rgb, const cv::Mat& mask, cv::Mat& mask_viz);
  void drawFeatures(const cv::Mat& rgb, const Frame& frame, cv::Mat& frame_viz);

  void addDisplayImages(const cv::Mat& image, const std::string& title);

private:
  ImageToDisplay::Vector display_images;

  static Color getColourFromInstanceMask(int value);

  // for viz and debug currently
  StaticTrackletManager static_tracklets;
};

}  // namespace VDO_SLAM