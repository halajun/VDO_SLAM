#pragma once

#include "FrontendOutput.h"
#include "Backend-Definitions.h"

namespace vdo
{
struct VisualiserInput
{
  FrontendOutput::Ptr frontend_output;
  BackendOutput::Ptr backend_output;
  

  VisualiserInput(FrontendOutput::Ptr frontend_output_, BackendOutput::Ptr backend_output_) 
  : frontend_output(frontend_output_), backend_output(backend_output_)
  {
  }
};

}  // namespace vdo