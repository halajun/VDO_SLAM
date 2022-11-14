#pragma once

#include "Macros.h"
#include "Types.h"
#include "Frontend-Definitions.h"
#include "ORBextractor.h"
#include "Camera.h"

#include <gtsam/geometry/Pose3.h>

namespace vdo
{
class Frame
{
public:
  VDO_POINTER_TYPEDEFS(Frame);

  Frame(const ImagePacket& images_, Timestamp timestamp_, size_t frame_id_, const CameraParams& cam_params_);

  inline const ImagePacket& Images() const
  {
    return images;
  }
  inline const Features& StaticFeatures() const
  {
    return static_features;
  }
  inline const Features& DynamicFeatures() const
  {
    return dynamic_features;
  }

  void detectFeatures(ORBextractor::UniquePtr& detector);
  void projectKeypoints(const Camera& camera);
  // //at each detected keypoint, process the flow and construct a predicted position of the feature in the next
  // //frame using the flow
  // void predictStaticFeatureCorrespondences(double depth_background_thresh);
  void processStaticFeatures(double depth_background_thresh);
  void processDynamicFeatures(double depth_object_thresh);

  void drawStaticFeatures(cv::Mat& image) const;
  void drawDynamicFeatures(cv::Mat& image) const;

private:
  // takes the input image and converts it to mono (if it is not already);
  void prepareRgbForDetection(const cv::Mat& rgb, cv::Mat& mono);

  void undistortKeypoints(const KeypointsCV& distorted, KeypointsCV& undistorted);

private:
  const ImagePacket images;  // must be const to ensure unchangable references to images
  const Timestamp timestamp;
  const size_t frame_id;
  const CameraParams cam_params;

  // all keypoints as detected by orb -> they will then be undistorted
  KeypointsCV keypoints;
  // depths of each keypoint (as taken from the depth map)
  cv::Mat descriptors;

  // //the predicted position of the static keypoints using the optical flow vector
  // std::vector<cv::KeyPoint> predicted_keypoints_static;
  // //the predicted optical flow value at each of the predicted KP's. Initalised with the current optical flow
  // //should have the same size as predicted_keypoints_static
  // std::vector<cv::Point2d> predicted_optical_flow;
  Features static_features;
  Landmarks static_landmarks;

  Features dynamic_features;
  Landmarks dynamic_landmarks;
};

}  // namespace vdo