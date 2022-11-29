#pragma once

#include "Macros.h"
#include "Types.h"

#include <opencv2/opencv.hpp>

namespace vdo
{
struct TrackingParams
{
  // tracking points params
  size_t max_tracking_points_bg;
  size_t max_tracking_points_obj;

  // scene flow thresholds
  double scene_flow_magnitude;
  double scene_flow_percentage;

  // depth thresholds
  double depth_background_thresh;
  double depth_obj_thresh;

  double depth_scale_factor;

  // ORB detector params
  int n_features;
  double scale_factor;
  int n_levels;
  int init_threshold_fast;
  int min_threshold_fast;
};

struct Observation
{
  enum class Type
  {
    OPTICAL_FLOW,
    DETECTION
  };

  // want to be const but then deleted copty constructor
  cv::KeyPoint keypoint;
  size_t tracklet_id;
  Type type;
  size_t age;

  Observation(const cv::KeyPoint& kp, size_t tracklet_id_, const Type& type_, size_t age_)
    : keypoint(kp), tracklet_id(tracklet_id_), type(type_), age(age_)
  {
  }
};

struct Feature
{
  VDO_POINTER_TYPEDEFS(Feature);
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
  size_t age = 0;  // how many times this landmark has been seen

  Type type;

  cv::Point2d optical_flow;  // the optical flow calculated at this keypoint
  // //the predicted position of this feature in the next frame -> initially this will be calculated
  // with optical flow

  bool inlier = true;

  // as we're moving forward this is actually the previous point as the flow is backwards.
  // the matching actually happens with the previous point
  cv::KeyPoint predicted_keypoint;

  InstanceLabel instance_label{ background };
};

using Features = std::vector<Feature>;
using Observations = std::vector<Observation>;

using TrackletIdFeatureMap = std::map<std::size_t, Feature::Ptr>;

}  // namespace vdo