#pragma once

#include "Macros.h"
#include "Frame.h"
#include "Types.h"

#include <gtsam/geometry/Pose3.h>

namespace vdo
{
struct FrontendOutput
{
  VDO_POINTER_TYPEDEFS(FrontendOutput);

  const Frame::Ptr frame_;  // really just for vizualisation
  const gtsam::Pose3 estimated_pose_;
  const size_t frame_id_;
  const GroundTruthInputPacket::ConstOptional ground_truth_;

  //in camera frame
  TackletIdToLandmark tracklet_landmark_map_;

  FrontendOutput(Frame::Ptr frame)
    : frame_(frame), estimated_pose_(frame->pose_), frame_id_(frame->frame_id_), ground_truth_(frame->ground_truth_)
  {
  }
};

}  // namespace vdo