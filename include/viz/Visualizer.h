#pragma once

#include "Macros.h"
#include "viz/OpenCvDisplay.h"
#include "viz/OpenCvVisualizer3D.h"
#include "viz/Display-Definitions.h"

namespace vdo
{
class Visualizer
{
public:
  VDO_POINTER_TYPEDEFS(Visualizer);

  Visualizer(DisplayParams::Ptr params_);

  void process(const VisualiserInput& viz_input);

private:
  OpenCvDisplay display_2d;
  OpenCvVisualizer3D display_3d;
};

}  // namespace vdo