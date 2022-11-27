#include "FeatureTracker.h"

#include <glog/logging.h>

namespace vdo {

FeatureTracker::FeatureTracker(const TrackingParams& tracking_params, const Camera& camera)
: tracking_params_(tracking_params), camera_(camera) 
{
    feature_detector_ =
      vdo::make_unique<ORBextractor>(tracking_params_.n_features, 
                                     static_cast<float>(tracking_params_.scale_factor), 
                                     tracking_params_.n_levels,
                                     tracking_params_.init_threshold_fast, 
                                     tracking_params_.min_threshold_fast);
    //  static constexpr int edge_threshold =
    //       10;  // Very small bcs we don't use descriptors (yet).
    // static constexpr int first_level = 0;
    // static constexpr int WTA_K = 0;  // We don't use descriptors.

    // #if CV_VERSION_MAJOR == 3
    //   static constexpr int score_type = cv::ORB::HARRIS_SCORE;
    // #else
    //     static constexpr cv::ORB::ScoreType score_type =
    //         cv::ORB::ScoreType::HARRIS_SCORE;
    // #endif
    // static constexpr int patch_size = 2;  // We don't use descriptors (yet).
    // feature_detector_ =
    //       cv::ORB::create(tracking_params.n_features,
    //                       static_cast<float>(tracking_params_.scale_factor),
    //                       tracking_params_.n_levels,
    //                       edge_threshold,
    //                       first_level,
    //                       WTA_K,
    //                       score_type,
    //                       patch_size,
    //                       tracking_params.min_threshold_fast);
}

void FeatureTracker::trackFeatures(const Frame::Ptr&  previous_frame, Frame::Ptr current_frame, size_t& n_optical_flow, size_t& n_new_tracks) {    
    //if previous_frame is null assime first frame and just track detect on current 
    CHECK(current_frame) << "Current frame cannot be null!";
    if(previous_frame == nullptr) {
        n_optical_flow = 0;
        n_new_tracks = 0;
        Observations detected_features = detectNewFeatures(current_frame->images.rgb);
        current_frame->constructFeatures(detected_features, tracking_params_.depth_background_thresh);
        current_frame->projectKeypoints(camera_);

    }
    else {
        CHECK(previous_frame->staticLandmarksValid());
        Observations of_tracked;
        n_optical_flow = opticalFlowTrack(previous_frame, of_tracked);

        Observations tracked_observations(of_tracked);
        n_new_tracks = 0;
        static constexpr size_t kMinTracks = 100;
        if (n_optical_flow < kMinTracks) {
            //apply mask
            cv::Mat mask;
            constructFeatureMask(previous_frame, mask);
            Observations detected_features_with_mask = detectNewFeatures(previous_frame->images.rgb, mask);
            Observations detected_features = detectNewFeatures(previous_frame->images.rgb);

            // LOG(INFO) << "features with/without mask - " << detected_features_with_mask.size() << "/" << detected_features.size();
            previous_frame->constructFeatures(detected_features, tracking_params_.depth_background_thresh);

            Observations newly_tracked;
            n_new_tracks = opticalFlowTrack(previous_frame, newly_tracked);

             //merge the two sets of tracked feautes, some old and some new
            std::copy(newly_tracked.begin(), newly_tracked.end(), std::back_inserter(tracked_observations));
        }


        current_frame->constructFeatures(tracked_observations, tracking_params_.depth_background_thresh);
        current_frame->projectKeypoints(camera_);
    }
}

Observations FeatureTracker::detectNewFeatures(const cv::Mat& img, const cv::Mat& mask) {
    cv::Mat mono;
    prepareRgbForDetection(img, mono);

    Observations observations;
    cv::Mat descriptors;
    KeypointsCV keypoints;

    // feature_detector_->detect(mono, keypoints, mask);
    (*feature_detector_)(mono, mask, keypoints, descriptors);
    LOG(INFO) << "Detected " << keypoints.size() << " kp";
    if (keypoints.size() == 0)
    {
        LOG(ERROR) << "zero features detected";
        //what if we have zero features but some left over observations
    }
    //TODO: make param
    //TODO: this will  add onto the exsiting obs -> we should try and spread the kp's around using something like NMS
    for(const KeypointCV& kp : keypoints) {
        Observation obs(kp, Frame::tracklet_id_count, Observation::Type::DETECTION, 0);
        Frame::tracklet_id_count++;
        observations.push_back(obs);
    }

    return observations;
}


size_t FeatureTracker::opticalFlowTrack(const Frame::Ptr& previous_frame_, Observations& observations) {
    observations.clear();
    const TrackletIdFeatureMap& previous_features = previous_frame_->static_features;
    
    for(const auto& feature_pair : previous_features) {
        const std::size_t& tracklet_id = feature_pair.first;
        const Feature& previous_feature = *feature_pair.second;
        const size_t& age = previous_feature.age;
        KeypointCV kp = previous_feature.predicted_keypoint;


        if(camera_.isKeypointContained(kp, previous_feature.depth) && previous_feature.inlier) {
            size_t new_age = age + 1;
            Observation obs(kp, previous_feature.tracklet_id, Observation::Type::OPTICAL_FLOW, new_age);
            observations.push_back(obs);

        }
    }

    return observations.size();
}

void FeatureTracker::prepareRgbForDetection(const cv::Mat& rgb, cv::Mat& mono)
{
  CHECK(!rgb.empty());
  PLOG_IF(ERROR, rgb.channels() == 1) << "Input image should be RGB (channels == 3), not 1";
  // Transfer color image to grey image
  rgb.copyTo(mono);

  if (mono.channels() == 3)
  {
    cv::cvtColor(mono, mono, CV_RGB2GRAY);
  }
  else if (rgb.channels() == 4)
  {
    cv::cvtColor(mono, mono, CV_RGBA2GRAY);
  }
}

void FeatureTracker::constructFeatureMask(const Frame::Ptr& previous_frame, cv::Mat& mask) {
    CHECK(previous_frame);
    const cv::Mat& rgb = previous_frame->images.rgb;
    const TrackletIdFeatureMap& previous_features = previous_frame->static_features;
    CHECK(previous_frame->staticLandmarksValid());

    mask = cv::Mat(rgb.size(), CV_8U, cv::Scalar(255));
    int ignored = 0;
     for(const auto& feature_pair : previous_features) {
        const Feature& previous_feature = *feature_pair.second;

        //TODO: param
        static constexpr int min_distance_btw_tracked_and_detected_features_ = 40;

        cv::Point kp(previous_feature.keypoint.pt.x, previous_feature.keypoint.pt.y);
        //1 becuase we start the track at the previous frame 
        if(previous_feature.age > 1) {
            ignored++;
            // The mask is interpreted as: 255 -> consider, 0 -> don't consider.
            cv::circle(mask,
                 kp,
                 min_distance_btw_tracked_and_detected_features_,
                 cv::Scalar(0),
                 CV_FILLED);
        }
    }

    LOG(INFO) << "Masked out - " << ignored;
}


}