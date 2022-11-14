#include "viz/Visualizer.h"

namespace vdo
{
Visualizer::Visualizer(DisplayParams::Ptr params_) : display_2d(params_), display_3d(params_)
{
}

void Visualizer::process(const VisualiserInput& viz_input)
{
  display_2d.process(viz_input);
  display_3d.process(viz_input);
}

}  // namespace vdo