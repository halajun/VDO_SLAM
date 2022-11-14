#pragma once

#include "Macros.h"
#include "viz/DisplayParams.h"
#include "viz/Display-Definitions.h"
#include <opencv2/opencv.hpp>

namespace vdo
{
class Display
{
public:
  VDO_POINTER_TYPEDEFS(Display);

  Display(DisplayParams::Ptr params_);
  virtual ~Display() = default;

  virtual void process(const VisualiserInput& viz_input) = 0;

  static cv::Scalar getObjectColour(int label);

protected:
  DisplayParams::Ptr params;
};

}  // namespace vdo