#pragma once

#include "FrontendOutput.h"

namespace vdo
{
struct VisualiserInput
{
  FrontendOutput::Ptr frontend_output;

  VisualiserInput(FrontendOutput::Ptr frontend_output_) : frontend_output(frontend_output_)
  {
  }
};

}  // namespace vdo