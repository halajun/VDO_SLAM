#pragma once

#include "Macros.h"
#include "Types.h"

#include <opencv2/opencv.hpp>

namespace vdo
{
struct Feature
{
  static constexpr InstanceLabel background = 0;

  enum class Type
  {
    STATIC,
    DYNAMIC
  };

  cv::KeyPoint keypoint;
  size_t index = -1;  // the index of the feature in the original vector (eg the keypoints vector)
  size_t frame_id = -1;
  Depth depth = -1;
  size_t tracklet_id = -1;

  Type type;

  cv::Point2d optical_flow;  // the optical flow calculated at this keypoint
  // //the predicted position of this feature in the next frame -> initially this will be calculated
  // with optical flow

  //as we're moving forward this is actually the previous point as the flow is backwards.
  //the matching actually happens with the previous point
  cv::KeyPoint predicted_keypoint;


  InstanceLabel instance_label{ background };
};

using Features = std::vector<Feature>;

}  // namespace vdo