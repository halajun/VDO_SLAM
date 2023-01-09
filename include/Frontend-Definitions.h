#pragma once

#include "Macros.h"
#include "Types.h"
#include "utils/Metrics.h"

#include <opencv2/opencv.hpp>
#include <boost/serialization/access.hpp>

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

struct VisualMeasurement
{
  cv::KeyPoint keypoint;
  size_t frame_id = -1;
  Depth depth = -1;
  size_t tracklet_id = -1;
};

struct Feature : public VisualMeasurement
{
  VDO_POINTER_TYPEDEFS(Feature);
  static constexpr InstanceLabel background = 0;

  enum class Type
  {
    STATIC,
    DYNAMIC
  };

  size_t index = -1;  // the index of the feature in the original vector (eg the keypoints vector)
  size_t age = 0;     // how many times this landmark has been seen

  Type type;

  cv::Point2d optical_flow;  // the optical flow calculated at this keypoint
  gtsam::Point3 scene_flow; //in the camera reference frame (t-1)
  // //the predicted position of this feature in the next frame -> initially this will be calculated
  // with optical flow

  bool inlier = true;

  // as we're moving forward this is actually the previous point as the flow is backwards.
  // the matching actually happens with the previous point
  cv::KeyPoint predicted_keypoint;

  InstanceLabel instance_label{ background }; //

  //object id -> only valid for dynamic object
  int object_id {-1};
};


struct FrontendMetrics {
  gtsam::Pose3 pose;
  gtsam::Pose3 gt_pose;
  Timestamp timestamp;
  size_t frame_id;

  //specific metrics for plotting etc
  ErrorPair ate_before_flow;
  ErrorPair rte_before_flow;

  ErrorPair ate_after_flow;
  ErrorPair rte_after_flow;

  template<class Archive>
  void serialize(Archive & ar, const unsigned int)
  {
      ar & BOOST_SERIALIZATION_NVP(pose);
      ar & BOOST_SERIALIZATION_NVP(gt_pose);
      ar & boost::serialization::make_nvp("timestamp", timestamp);
      ar & boost::serialization::make_nvp("frame_id", frame_id);
      ar & BOOST_SERIALIZATION_NVP(ate_before_flow);
      ar & BOOST_SERIALIZATION_NVP(rte_before_flow);
      ar & BOOST_SERIALIZATION_NVP(ate_after_flow);
      ar & BOOST_SERIALIZATION_NVP(rte_after_flow);
  }

};

using Features = std::vector<vdo::Feature>;
using FeaturePtrs = std::vector<vdo::Feature::Ptr>;
using Observations = std::vector<vdo::Observation>;

using TrackletIdFeatureMap = std::map<std::size_t, vdo::Feature::Ptr>;
using ObjectInstanceFeatureMap = std::map<InstanceLabel, FeaturePtrs>;

}  // namespace vdo