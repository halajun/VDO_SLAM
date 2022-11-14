#pragma once

#include "Macros.h"
#include "Frame.h"

namespace vdo
{
struct FrontendOutput
{
  VDO_POINTER_TYPEDEFS(FrontendOutput);

  const Frame::Ptr frame;
  FrontendOutput(Frame::Ptr frame_) : frame(frame_)
  {
  }
};

}  // namespace vdo