#pragma once

#include "Macros.h"
#include "Types.h"
#include "Frontend-Definitions.h"
#include "ORBextractor.h"
#include "Camera.h"


#include <map>
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

  Feature::Ptr getStaticFeature(std::size_t tracklet_id) const;
  // inline const Features& StaticFeatures() const
  // {
  //   return static_features;
  // }
  // inline const Features& DynamicFeatures() const
  // {
  //   return dynamic_features;
  // }

  inline const Landmarks& StaticLandmarks() const
  {
    return static_landmarks;
  }
  inline const Landmarks& DynamicLandmarks() const
  {
    return dynamic_landmarks;
  }

  //clears current observations, redetects features using the detector and updates the static tracksd
  void refreshFeatures(ORBextractor::UniquePtr& detector, double depth_background_thresh);

  //we can either detect features or add new ones (from optical flow)
  //HACK: for now
  void addStaticFeatures(const Observations& observations_);
  // void addFeatures(const Features& features_);
  // void addKeypoints(const KeypointsCV& keypoints_);
  void detectFeatures(ORBextractor::UniquePtr& detector);
  // static and dynamic?
  void projectKeypoints(const Camera& camera);
  // //at each detected keypoint, process the flow and construct a predicted position of the feature in the next
  // //frame using the flow
  // void predictStaticFeatureCorrespondences(double depth_background_thresh);
  void processStaticFeatures(double depth_background_thresh);
  void processDynamicFeatures(double depth_object_thresh);

  void drawStaticFeatures(cv::Mat& image) const;
  void drawDynamicFeatures(cv::Mat& image) const;

  // public for now
  gtsam::Pose3 pose;
  //not sure if best repreented by pose3
  gtsam::Pose3 motion_model;
  GroundTruthInputPacket::ConstOptional ground_truth{ boost::none };

private:
  // takes the input image and converts it to mono (if it is not already);
  void prepareRgbForDetection(const cv::Mat& rgb, cv::Mat& mono);

  void undistortKeypoints(const KeypointsCV& distorted, KeypointsCV& undistorted);

//HACK: for now
public:
  const ImagePacket images;  // must be const to ensure unchangable references to images
  const Timestamp timestamp;
  const size_t frame_id;
  const CameraParams cam_params;

  // all keypoints as detected by orb -> they will then be undistorted
  Observations observations;
  // depths of each keypoint (as taken from the depth map)
  cv::Mat descriptors;

  //TODO: probably make pointers
  TrackletIdFeatureMap static_features;
  Landmarks static_landmarks;  // as projected in the camera frame

  Features dynamic_features;
  Landmarks dynamic_landmarks;  // as projected in the camera frame

  static std::size_t tracklet_id_count; //for static and dynamic
};

}  // namespace vdo