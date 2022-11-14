#pragma once

#include "utils/macros.h"
#include "visualizer/DisplayParams.h"
#include <opencv2/opencv.hpp>

namespace VDO_SLAM
{
class Display
{
public:
  VDO_SLAM_POINTER_TYPEDEFS(Display);

  Display(DisplayParams::Ptr params_);
  virtual ~Display() = default;

  virtual void process() = 0;

  static cv::Scalar getObjectColour(int label);

protected:
  DisplayParams::Ptr params;
};

}  // namespace VDO_SLAM