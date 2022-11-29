#pragma once

#include "ORBextractor.h"
#include "Frontend-Definitions.h"  //for TrackingParams
#include "Camera.h"
#include "Frame.h"
#include "Macros.h"

#include <opencv2/features2d.hpp>

namespace vdo
{
class FeatureTracker
{
public:
  VDO_POINTER_TYPEDEFS(FeatureTracker);

  FeatureTracker(const TrackingParams& tracking_params, const Camera& camera);

  // only for static stuff
  void trackFeatures(const Frame::Ptr& previous_frame, Frame::Ptr current_frame, size_t& n_optical_flow,
                     size_t& n_new_tracks);

private:
  Observations detectNewFeatures(const cv::Mat& img, const cv::Mat& mask = cv::Mat());
  size_t opticalFlowTrack(const Frame::Ptr& previous_frame_, Observations& observations);

  void prepareRgbForDetection(const cv::Mat& rgb, cv::Mat& mono);

  // constructs a mask to be used for the feature detection
  // the mask is constructed by looking at all the Features in the previous frame
  // and masking them out if they are tracked (ie age > 0) and within a certain distance
  void constructFeatureMask(const Frame::Ptr& previous_frame, cv::Mat& mask);

private:
  TrackingParams tracking_params_;
  Camera camera_;
  ORBextractor::UniquePtr feature_detector_;

  // Actual feature detector implementation.
  // cv::Ptr<cv::Feature2D> feature_detector_;
};

}  // namespace vdo