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
struct Frame
{
  VDO_POINTER_TYPEDEFS(Frame);
  FeaturePtrs features_;
  TrackletIdFeatureMap feature_by_tracklet_id_;

  FeaturePtrs dynamic_features_;
  TrackletIdFeatureMap dynamuic_feature_by_tracklet_id_;
  ImagePacket images_;  //there are some situationtions where we want to update the images
  const Timestamp timestamp_;
  const size_t frame_id_;

  ObjectInstanceFeatureMap semantic_instance_map_;

  gtsam::Pose3 pose_;  // in world
  gtsam::Pose3 motion_model_;
  GroundTruthInputPacket::ConstOptional ground_truth_{ boost::none };

  Frame(const ImagePacket& images, Timestamp timestamp, size_t frame_id);

  inline Feature::Ptr getByTrackletId(size_t tracklet_id) const {
    if(feature_by_tracklet_id_.find(tracklet_id) == feature_by_tracklet_id_.end()) {
      return nullptr;
    }
    else {
      return feature_by_tracklet_id_.at(tracklet_id);
    }
  }

  inline Feature::Ptr getDynamicByTrackletId(size_t tracklet_id) const {
    if(dynamuic_feature_by_tracklet_id_.find(tracklet_id) == dynamuic_feature_by_tracklet_id_.end()) {
      return nullptr;
    }
    else {
      return dynamuic_feature_by_tracklet_id_.at(tracklet_id);
    }
  }


};

// class Frame
// {
// public:
//   VDO_POINTER_TYPEDEFS(Frame);

//   Frame(const ImagePacket& images_, Timestamp timestamp_, size_t frame_id_, const CameraParams& cam_params_);

//   inline const ImagePacket& Images() const
//   {
//     return images;
//   }

//   Feature::Ptr getStaticFeature(std::size_t tracklet_id) const;

//   // a quick validation check that we have the same number of Features as landmaks
//   // and that number is > 0
//   bool staticLandmarksValid() const;

//   // clears current observations, redetects features using the detector and updates the static tracksd
//   //
//   void constructFeatures(const Observations& observations_, double depth_background_thresh);

//   // //we can either detect features or add new ones (from optical flow)
//   // //HACK: for now
//   // void addStaticFeatures(const Observations& observations_);
//   // // void addFeatures(const Features& features_);
//   // // void addKeypoints(const KeypointsCV& keypoints_);
//   // void detectFeatures(ORBextractor::UniquePtr& detector);
//   // static and dynamic?
//   void projectKeypoints(const Camera& camera);
//   // //at each detected keypoint, process the flow and construct a predicted position of the feature in the next
//   // //frame using the flow

//   void processStaticFeatures(double depth_background_thresh);
//   void processDynamicFeatures(double depth_object_thresh);

//   void drawStaticFeatures(cv::Mat& image) const;
//   void drawDynamicFeatures(cv::Mat& image) const;

//   // public for now
//   gtsam::Pose3 pose;  // in world

//   // not sure if best repreented by pose3
//   gtsam::Pose3 motion_model;
//   GroundTruthInputPacket::ConstOptional ground_truth{ boost::none };

// private:
//   // takes the input image and converts it to mono (if it is not already);
//   void prepareRgbForDetection(const cv::Mat& rgb, cv::Mat& mono);

//   void undistortKeypoints(const KeypointsCV& distorted, KeypointsCV& undistorted);

//   // HACK: for now
// public:
//   const ImagePacket images;  // must be const to ensure unchangable references to images
//   const Timestamp timestamp;
//   const size_t frame_id;
//   const CameraParams cam_params;

//   // all keypoints as detected by orb -> they will then be undistorted
//   Observations observations;
//   // depths of each keypoint (as taken from the depth map)
//   cv::Mat descriptors;

//   // TODO: probably make pointers
//   TrackletIdFeatureMap static_features;
//   Landmarks static_landmarks;  // as projected in the camera frame

//   Features dynamic_features;
//   Landmarks dynamic_landmarks;  // as projected in the camera frame

//   static std::size_t tracklet_id_count;  // for static and dynamic
// };

}  // namespace vdo