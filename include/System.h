#pragma once

#include "Macros.h"
#include "Types.h"
#include "Tracking.h"
#include "Optimizer.h"
#include "viz/Visualizer.h"

#include <gtsam/geometry/Pose3.h>
#include <opencv2/opencv.hpp>
#include <boost/optional.hpp>

namespace vdo
{
class System
{
public:
  VDO_POINTER_TYPEDEFS(System);

  System(const std::string& settings_file);

  gtsam::Pose3 TrackRGBD(const InputPacket& input, GroundTruthInputPacket::ConstOptional ground_truth = boost::none);

private:
  Tracking::UniquePtr tracker;
  IncrementalOptimizer::UniquePtr optimizer;
  Visualizer::UniquePtr viz;
};

}  // namespace vdo