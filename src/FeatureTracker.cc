#include "FeatureTracker.h"

#include <glog/logging.h>

namespace vdo
{
FeatureTracker::FeatureTracker(const TrackingParams& tracking_params, const Camera& camera)
  : tracking_params_(tracking_params), camera_(camera)
{
  feature_detector_ = vdo::make_unique<ORBextractor>(
      tracking_params_.n_features, static_cast<float>(tracking_params_.scale_factor), tracking_params_.n_levels,
      tracking_params_.init_threshold_fast, tracking_params_.min_threshold_fast);
}

Frame::Ptr FeatureTracker::track(const InputPacket& input_packet, size_t& n_optical_flow, size_t& n_new_tracks)
{
  const ImagePacket& images = input_packet.images;

  if (initial_computation_)
  {
    CHECK(!previous_frame_);
    img_size_ = images.rgb.size();
    computeImageBounds(img_size_, min_x_, max_x_, min_y_, max_y_);

    grid_elements_width_inv_ = static_cast<double>(FRAME_GRID_COLS) / static_cast<double>(max_x_ - min_x_);
    grid_elements_height_inv_ = static_cast<double>(FRAME_GRID_ROWS) / static_cast<double>(max_y_ - min_y_);

    LOG(INFO) << grid_elements_width_inv_ << " " << grid_elements_height_inv_;

    initial_computation_ = false;
  }

  // convert rgb image to mono for detection
  // TODO: make function
  const cv::Mat& rgb = images.rgb;
  const cv::Mat& depth = images.depth;
  const cv::Mat& flow = images.flow;
  const size_t& frame_id = input_packet.frame_id;
  const Timestamp& timestamp = input_packet.timestamp;
  cv::Mat mono;
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

  // run detetions
  cv::Mat descriptors;
  KeypointsCV detected_keypoints;
  (*feature_detector_)(mono, cv::Mat(), detected_keypoints, descriptors);
  LOG(INFO) << "detected - " << detected_keypoints.size();

  FeaturePtrs features;

  // appy tracking
  if (previous_frame_)
  {
    for (Feature::Ptr feature : previous_frame_->features_)
    {
      const size_t tracklet_id = feature->tracklet_id;
      const size_t age = feature->age;
      KeypointCV kp = feature->predicted_keypoint;

      // if camera contained
      if (camera_.isKeypointContained(kp, feature->depth) && feature->inlier)
      {
        //

        size_t new_age = age + 1;
        Feature::Ptr feature = constructStaticFeature(images, kp, new_age, tracklet_id, frame_id);
        if (feature)
        {
          features.push_back(feature);
        }
      }
    }
  }

  n_optical_flow = features.size();
  LOG(INFO) << "tracked with optical flow - " << n_optical_flow;

  const int& min_tracks = tracking_params_.n_features;

  std::vector<std::size_t> grid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

  // Assign Features to Grid Cells
  int n_reserve = (FRAME_GRID_COLS * FRAME_GRID_ROWS) / (0.5 * min_tracks);
  LOG(INFO) << "reserving - " << n_reserve;
  for (unsigned int i = 0; i < FRAME_GRID_COLS; i++)
    for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++)
      grid[i][j].reserve(n_reserve);

  // assign tracked features to grid
  for (Feature::Ptr feature : features)
  {
    const cv::KeyPoint& kp = feature->keypoint;
    int grid_x, grid_y;
    if (posInGrid(kp, grid_x, grid_y))
    {
      grid[grid_x][grid_y].push_back(feature->tracklet_id);
    }
  }

  // only add new features if tracks drop below min tracks
  if (features.size() < min_tracks)
  {
    // iterate over new observations
    for (cv::KeyPoint kp : detected_keypoints)
    {
      // TODO: if not object etc etc

      int grid_x, grid_y;
      if (posInGrid(kp, grid_x, grid_y))
      {
        // only add of we have less than n_reserve ammount
        if (grid[grid_x][grid_y].size() < n_reserve)
        {
          // Feature::Ptr new_feature = std::make_shared<Feature>();
          const size_t age = 0;
          const size_t trackled_id = tracklet_count;
          Feature::Ptr feature = constructStaticFeature(images, kp, age, trackled_id, frame_id);
          if (feature)
          {
            tracklet_count++;
            grid[grid_x][grid_y].push_back(feature->tracklet_id);
            features.push_back(feature);
          }
          // new_feature->age = 0;
          // new_feature->keypoint = kp;
          // new_feature->tracklet_id = tracklet_count;
          // tracklet_count++;

          // new_feature->depth = 1;
          // // new_feature->type = Feature::Type::STATIC;

          // int x = kp.pt.x;
          // int y = kp.pt.y;

          // double flow_xe = static_cast<double>(flow.at<cv::Vec2f>(y, x)[0]);
          // double flow_ye = static_cast<double>(flow.at<cv::Vec2f>(y, x)[1]);
          // new_feature->optical_flow = cv::Point2d(flow_xe, flow_ye);
          // new_feature->predicted_keypoint = cv::KeyPoint(x + flow_xe, y + flow_ye, 0, 0, 0, kp.octave, -1);
        }
      }
    }
  }

  size_t total_tracks = features.size();
  n_new_tracks = total_tracks - n_optical_flow;
  LOG(INFO) << "new tracks - " << n_optical_flow;
  LOG(INFO) << "total tracks - " << total_tracks;

  Frame::Ptr frame = std::make_shared<Frame>(images, timestamp, frame_id);
  frame->features_ = features;
  previous_frame_ = frame;
  return frame;
}

Feature::Ptr FeatureTracker::constructStaticFeature(const ImagePacket& images, cv::KeyPoint& kp, size_t age,
                                                    size_t tracklet_id, size_t frame_id) const
{
  const int x = kp.pt.x;
  const int y = kp.pt.y;
  if (images.semantic_mask.at<int>(y, x) != Feature::background)
  {
    return nullptr;
  }

  if (images.depth.at<double>(y, x) > tracking_params_.depth_background_thresh || images.depth.at<double>(y, x) <= 0)
  {
    return nullptr;
  }

  Feature::Ptr feature = std::make_shared<Feature>();
  feature->keypoint = kp;
  feature->age = age;
  feature->tracklet_id = tracklet_id;
  feature->frame_id = frame_id;
  feature->type = Feature::Type::STATIC;
  // feature->index ?
  feature->instance_label = images.semantic_mask.at<InstanceLabel>(y, x);

  double flow_xe = static_cast<double>(images.flow.at<cv::Vec2f>(y, x)[0]);
  double flow_ye = static_cast<double>(images.flow.at<cv::Vec2f>(y, x)[1]);
  feature->optical_flow = cv::Point2d(flow_xe, flow_ye);
  feature->predicted_keypoint = cv::KeyPoint(x + flow_xe, y + flow_ye, 0, 0, 0, kp.octave, -1);

  Depth d = images.depth.at<double>(y, x);  // be careful with the order  !!!
  if (d > 0)
  {
    feature->depth = d;
  }
  else
  {
    // log warning?
    feature->depth = -1;
  }

  return feature;
}

bool FeatureTracker::posInGrid(const cv::KeyPoint& kp, int& pos_x, int& pos_y) const
{
  pos_x = round((kp.pt.x - min_x_) * grid_elements_width_inv_);
  pos_y = round((kp.pt.y - min_y_) * grid_elements_height_inv_);

  // Keypoint's coordinates are undistorted, which could cause to go out of the image
  if (pos_x < 0 || pos_x >= FRAME_GRID_COLS || pos_y < 0 || pos_y >= FRAME_GRID_ROWS)
    return false;

  return true;
}

void FeatureTracker::computeImageBounds(const cv::Size& size, int& min_x, int& max_x, int& min_y, int& max_y) const
{
  // TODO: distortion
  min_x = 0;
  max_x = size.width;
  min_y = 0;
  max_y = size.height;
}

// void FeatureTracker::trackFeatures(const Frame::Ptr& previous_frame, Frame::Ptr current_frame, size_t&
// n_optical_flow,
//                                    size_t& n_new_tracks)
// {
//   // if previous_frame is null assime first frame and just track detect on current
//   CHECK(current_frame) << "Current frame cannot be null!";
//   if (previous_frame == nullptr)
//   {
//     n_optical_flow = 0;
//     n_new_tracks = 0;
//     Observations detected_features = detectNewFeatures(current_frame->images.rgb);
//     current_frame->constructFeatures(detected_features, tracking_params_.depth_background_thresh);
//     current_frame->projectKeypoints(camera_);
//   }
//   else
//   {
//     CHECK(previous_frame->staticLandmarksValid());
//     Observations of_tracked;
//     n_optical_flow = opticalFlowTrack(previous_frame, of_tracked);

//     Observations tracked_observations(of_tracked);
//     n_new_tracks = 0;
//     static constexpr size_t kMinTracks = 300;
//     if (n_optical_flow < kMinTracks)
//     {
//       // apply mask
//       cv::Mat mask;
//       constructFeatureMask(previous_frame, mask);
//       Observations detected_features_with_mask = detectNewFeatures(previous_frame->images.rgb, mask);
//       Observations detected_features = detectNewFeatures(previous_frame->images.rgb);

//       // LOG(INFO) << "features with/without mask - " << detected_features_with_mask.size() << "/" <<
//       // detected_features.size();
//       previous_frame->constructFeatures(detected_features, tracking_params_.depth_background_thresh);

//       Observations newly_tracked;
//       n_new_tracks = opticalFlowTrack(previous_frame, newly_tracked);

//       // merge the two sets of tracked feautes, some old and some new
//       std::copy(newly_tracked.begin(), newly_tracked.end(), std::back_inserter(tracked_observations));
//     }

//     current_frame->constructFeatures(tracked_observations, tracking_params_.depth_background_thresh);
//     current_frame->projectKeypoints(camera_);
//   }
// }

// Observations FeatureTracker::detectNewFeatures(const cv::Mat& img, const cv::Mat& mask)
// {
//   cv::Mat mono;
//   prepareRgbForDetection(img, mono);

//   Observations observations;
//   cv::Mat descriptors;
//   KeypointsCV keypoints;

//   // feature_detector_->detect(mono, keypoints, mask);
//   (*feature_detector_)(mono, mask, keypoints, descriptors);
//   LOG(INFO) << "Detected " << keypoints.size() << " kp";
//   if (keypoints.size() == 0)
//   {
//     LOG(ERROR) << "zero features detected";
//     // what if we have zero features but some left over observations
//   }
//   // TODO: make param
//   // TODO: this will  add onto the exsiting obs -> we should try and spread the kp's around using something like NMS
//   for (const KeypointCV& kp : keypoints)
//   {
//     Observation obs(kp, Frame::tracklet_id_count, Observation::Type::DETECTION, 0);
//     Frame::tracklet_id_count++;
//     observations.push_back(obs);
//   }

//   return observations;
// }

// size_t FeatureTracker::opticalFlowTrack(const Frame::Ptr& previous_frame_, Observations& observations)
// {
//   observations.clear();
//   const TrackletIdFeatureMap& previous_features = previous_frame_->static_features;

//   for (const auto& feature_pair : previous_features)
//   {
//     const std::size_t& tracklet_id = feature_pair.first;
//     const Feature& previous_feature = *feature_pair.second;
//     const size_t& age = previous_feature.age;
//     KeypointCV kp = previous_feature.predicted_keypoint;

//     if (camera_.isKeypointContained(kp, previous_feature.depth) && previous_feature.inlier)
//     {
//       size_t new_age = age + 1;
//       Observation obs(kp, previous_feature.tracklet_id, Observation::Type::OPTICAL_FLOW, new_age);
//       observations.push_back(obs);
//     }
//   }

//   return observations.size();
// }

// void FeatureTracker::prepareRgbForDetection(const cv::Mat& rgb, cv::Mat& mono)
// {
//   CHECK(!rgb.empty());
//   PLOG_IF(ERROR, rgb.channels() == 1) << "Input image should be RGB (channels == 3), not 1";
//   // Transfer color image to grey image
//   rgb.copyTo(mono);

//   if (mono.channels() == 3)
//   {
//     cv::cvtColor(mono, mono, CV_RGB2GRAY);
//   }
//   else if (rgb.channels() == 4)
//   {
//     cv::cvtColor(mono, mono, CV_RGBA2GRAY);
//   }
// }

// void FeatureTracker::constructFeatureMask(const Frame::Ptr& previous_frame, cv::Mat& mask)
// {
//   CHECK(previous_frame);
//   const cv::Mat& rgb = previous_frame->images.rgb;
//   const TrackletIdFeatureMap& previous_features = previous_frame->static_features;
//   CHECK(previous_frame->staticLandmarksValid());

//   mask = cv::Mat(rgb.size(), CV_8U, cv::Scalar(255));
//   int ignored = 0;
//   for (const auto& feature_pair : previous_features)
//   {
//     const Feature& previous_feature = *feature_pair.second;

//     // TODO: param
//     static constexpr int min_distance_btw_tracked_and_detected_features_ = 40;

//     cv::Point kp(previous_feature.keypoint.pt.x, previous_feature.keypoint.pt.y);
//     // 1 becuase we start the track at the previous frame
//     if (previous_feature.age > 1)
//     {
//       ignored++;
//       // The mask is interpreted as: 255 -> consider, 0 -> don't consider.
//       cv::circle(mask, kp, min_distance_btw_tracked_and_detected_features_, cv::Scalar(0), CV_FILLED);
//     }
//   }

//   LOG(INFO) << "Masked out - " << ignored;
// }

}  // namespace vdo