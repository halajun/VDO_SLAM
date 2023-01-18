#include "FeatureTracker.h"
#include "utils/UtilsOpenCV.h"
#include "Tracking-Tools.h"
#include "viz/Display.h"

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
  const size_t& frame_id = input_packet.frame_id;
  const Timestamp& timestamp = input_packet.timestamp;
  Frame::Ptr frame = std::make_shared<Frame>(images, timestamp, frame_id);

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
  else
  {
    CHECK(previous_frame_);
    tracking_tools::updateFrameMask(previous_frame_, frame);
  }

  FeaturePtrs static_features;
  trackStatic(input_packet, static_features, n_optical_flow, n_new_tracks);

  FeaturePtrs dynamic_features;
  trackDynamic(input_packet, dynamic_features);

  frame->features_ = static_features;
  frame->dynamic_features_ = dynamic_features;

  for (Feature::Ptr feature : frame->features_)
  {
    frame->feature_by_tracklet_id_.insert({ feature->tracklet_id, feature });
  }
  CHECK_EQ(frame->feature_by_tracklet_id_.size(), frame->features_.size());

  for (Feature::Ptr feature : frame->dynamic_features_)
  {
    frame->dynamuic_feature_by_tracklet_id_.insert({ feature->tracklet_id, feature });

    // //really would be nice for this to be a templated function as i use this code all the time
    if (frame->semantic_instance_map_.find(feature->instance_label) == frame->semantic_instance_map_.end())
    {
      frame->semantic_instance_map_.insert({ feature->instance_label, { feature } });
    }
    else
    {
      frame->semantic_instance_map_.at(feature->instance_label).push_back(feature);
    }
  }
  CHECK_EQ(frame->dynamuic_feature_by_tracklet_id_.size(), frame->dynamic_features_.size());

  previous_frame_ = frame;
  return frame;
}

void FeatureTracker::trackStatic(const InputPacket& input_packet, FeaturePtrs& static_features, size_t& n_optical_flow,
                                 size_t& n_new_tracks)
{
  const ImagePacket& images = input_packet.images;
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

  cv::Mat viz;
  rgb.copyTo(viz);

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
  // save detections
  orb_detections_.insert({ frame_id, detected_keypoints });
  // cv::drawKeypoints(viz, detected_keypoints, viz, cv::Scalar(0, 0, 255));
  VLOG(20) << "detected - " << detected_keypoints.size();

  FeaturePtrs features_tracked;
  std::vector<bool> detections_tracked(detected_keypoints.size(), false);

  const int& min_tracks = tracking_params_.n_features;
  std::vector<std::size_t> grid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

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
        // LOG(INFO) << "new_age " << new_age;
        cv::arrowedLine(viz, feature->keypoint.pt, kp.pt, cv::Scalar(255, 0, 0));
        utils::DrawCircleInPlace(viz, kp.pt, cv::Scalar(0, 0, 255));

        Feature::Ptr feature = constructStaticFeature(images, kp, new_age, tracklet_id, frame_id);
        if (feature)
        {
          features_tracked.push_back(feature);
        }
      }
    }
  }

  n_optical_flow = features_tracked.size();
  VLOG(20) << "tracked with optical flow - " << n_optical_flow;

  // Assign Features to Grid Cells
  int n_reserve = (FRAME_GRID_COLS * FRAME_GRID_ROWS) / (0.5 * min_tracks);
  VLOG(20) << "reserving - " << n_reserve;
  // for (unsigned int i = 0; i < FRAME_GRID_COLS; i++)
  //   for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++)
  //     grid[i][j].reserve(n_reserve);

  // assign tracked features to grid
  FeaturePtrs features_assigned;
  for (Feature::Ptr feature : features_tracked)
  {
    const cv::KeyPoint& kp = feature->keypoint;
    int grid_x, grid_y;
    if (posInGrid(kp, grid_x, grid_y))
    {
      grid[grid_x][grid_y].push_back(feature->tracklet_id);
      features_assigned.push_back(feature);
    }
  }

  VLOG(20) << "assigned grid features - " << features_assigned.size();
  // only add new features if tracks drop below min tracks
  if (features_assigned.size() < min_tracks)
  {
    // iterate over new observations
    for (size_t i = 0; i < detected_keypoints.size(); i++)
    {
      // TODO: if not object etc etc
      const KeypointCV& kp = detected_keypoints[i];
      const int& x = kp.pt.x;
      const int& y = kp.pt.y;

      // if not already tracked with optical flow
      if (detections_tracked[i] || images.semantic_mask.at<int>(y, x) != Feature::background)
      {
        continue;
      }

      int grid_x, grid_y;
      if (posInGrid(kp, grid_x, grid_y))
      {
        // only add of we have less than n_reserve ammount
        if (grid[grid_x][grid_y].size() < n_reserve)
        {
          // Feature::Ptr new_feature = std::make_shared<Feature>();
          const size_t age = 0;
          size_t trackled_id = tracklet_count;
          Feature::Ptr feature = constructStaticFeature(images, kp, age, trackled_id, frame_id);
          if (feature)
          {
            tracklet_count++;
            grid[grid_x][grid_y].push_back(feature->tracklet_id);
            features_assigned.push_back(feature);
            // draw an untracked kp
            utils::DrawCircleInPlace(viz, feature->keypoint.pt, cv::Scalar(0, 255, 0));
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

  // cv::imshow("Optical flow", viz);
  // cv::waitKey(1);

  size_t total_tracks = features_assigned.size();
  n_new_tracks = total_tracks - n_optical_flow;
  LOG(INFO) << "new tracks - " << n_new_tracks;
  LOG(INFO) << "total tracks - " << total_tracks;

  static_features.clear();
  static_features = features_assigned;
}

void FeatureTracker::trackDynamic(const InputPacket& input_packet, FeaturePtrs& dynamic_features)
{
  // first dectect dynamic points
  const ImagePacket& images = input_packet.images;
  const cv::Mat& rgb = images.rgb;
  const cv::Mat& depth = images.depth;
  const cv::Mat& flow = images.flow;
  const size_t& frame_id = input_packet.frame_id;
  const Timestamp& timestamp = input_packet.timestamp;
  int step = 3;  // 3

  cv::Mat viz;
  rgb.copyTo(viz);

  FeaturePtrs sampled_features, tracked_features;
  std::vector<InstanceLabel> instance_labels;

  // a mask to show which feature have been with optical flow from the previous frame
  // zeros mean have not been tracked
  cv::Mat tracked_feature_mask = cv::Mat::zeros(rgb.size(), CV_8UC1);

  if (previous_frame_)
  {
    for (Feature::Ptr previous_dynamic_feature : previous_frame_->dynamic_features_)
    {
      const size_t tracklet_id = previous_dynamic_feature->tracklet_id;
      const size_t age = previous_dynamic_feature->age;
      KeypointCV kp = previous_dynamic_feature->predicted_keypoint;

      const int& x = kp.pt.x;
      const int& y = kp.pt.y;

      InstanceLabel predicted_label = images.semantic_mask.at<InstanceLabel>(y, x);

      const KeypointCV& previous_point = previous_dynamic_feature->keypoint;
      // InstanceLabel previous_label = previous_frame_->images_.semantic_mask.at<InstanceLabel>(previous_point.pt.y,
      // previous_point.pt.x); bool same_propogated_label = previous_label == predicted_label;

      if (camera_.isKeypointContained(kp, previous_dynamic_feature->depth) && previous_dynamic_feature->inlier &&
          predicted_label != Feature::background)
      {
        size_t new_age = age + 1;
        double flow_xe = static_cast<double>(images.flow.at<cv::Vec2f>(y, x)[0]);
        double flow_ye = static_cast<double>(images.flow.at<cv::Vec2f>(y, x)[1]);

        // // save correspondences
        Feature::Ptr feature = std::make_shared<Feature>();
        // feature->instance_label = images.semantic_mask.at<InstanceLabel>(y, x);
        feature->instance_label = predicted_label;
        feature->frame_id = frame_id;
        feature->type = Feature::Type::DYNAMIC;
        feature->age = new_age;
        feature->tracklet_id = tracklet_id;
        // the flow is not actually what
        cv::KeyPoint predicted_kp(x + flow_xe, y + flow_ye, 0, 0, 0, 0, -1);
        feature->optical_flow = cv::Point2d(flow_xe, flow_ye);
        feature->predicted_keypoint = predicted_kp;
        feature->keypoint = kp;

        // propogate dynamic object label
        feature->object_id = previous_dynamic_feature->object_id;

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
        // TODO: change to tracked
        sampled_features.push_back(feature);
        tracked_feature_mask.at<uchar>(y, x) = 1;
        instance_labels.push_back(feature->instance_label);
        // calculate scene flow
        // Landmark lmk_current, lmk_previous;
        // camera_.backProject(feature->keypoint, feature->depth, &lmk_current);
        // camera_.backProject(previous_point, previous_dynamic_feature->depth, &lmk_previous);

        // //put both into the world frame
        // lmk_previous = previous_frame_->pose_.transformFrom(lmk_previous);

        cv::arrowedLine(viz, previous_dynamic_feature->keypoint.pt, feature->keypoint.pt,
                        Display::getObjectColour(feature->instance_label));
        // utils::DrawCircleInPlace(viz, feature->keypoint.pt, cv::Scalar(0, 0, 255));
      }
    }
  }

  LOG(INFO) << "Tracked dynamic points - " << sampled_features.size();

  for (int i = 0; i < rgb.rows - step; i = i + step)
  {
    for (int j = 0; j < rgb.cols - step; j = j + step)
    {
      if (images.semantic_mask.at<int>(i, j) == Feature::background)
      {
        continue;
      }

      if (images.depth.at<double>(i, j) >= tracking_params_.depth_obj_thresh || images.depth.at<double>(i, j) <= 0)
      {
        continue;
      }

      double flow_xe = static_cast<double>(images.flow.at<cv::Vec2f>(i, j)[0]);
      double flow_ye = static_cast<double>(images.flow.at<cv::Vec2f>(i, j)[1]);

      int occupied = static_cast<int>(tracked_feature_mask.at<uchar>(i, j));

      // we are within the image bounds?
      if (j + flow_xe < rgb.cols && j + flow_xe > 0 && i + flow_ye < rgb.rows && i + flow_ye > 0 && !occupied)
      {
        // // save correspondences
        Feature::Ptr feature = std::make_shared<Feature>();
        feature->instance_label = images.semantic_mask.at<InstanceLabel>(i, j);
        feature->frame_id = frame_id;
        feature->type = Feature::Type::DYNAMIC;
        feature->age = 0;
        feature->tracklet_id = tracklet_count;
        tracklet_count++;
        // the flow is not actually what
        cv::KeyPoint predicted_kp(j + flow_xe, i + flow_ye, 0, 0, 0, 0, -1);
        feature->optical_flow = cv::Point2d(flow_xe, flow_ye);
        feature->predicted_keypoint = predicted_kp;
        feature->keypoint = cv::KeyPoint(j, i, 0, 0, 0, -1);

        Depth d = images.depth.at<double>(i, j);  // be careful with the order  !!!
        if (d > 0)
        {
          feature->depth = d;
        }
        else
        {
          // log warning?
          feature->depth = -1;
        }
        CHECK_EQ(feature->object_id, -1);
        sampled_features.push_back(feature);
        instance_labels.push_back(feature->instance_label);
        // utils::DrawCircleInPlace(viz, feature->keypoint.pt, cv::Scalar(0, 255, 0));
      }
    }
  }
  dynamic_features = sampled_features;

  cv::imshow("Dynamic flow", viz);
  cv::waitKey(1);

  LOG(INFO) << "Dynamic Sampled " << sampled_features.size();

  std::sort(instance_labels.begin(), instance_labels.end());
  std::vector<InstanceLabel> unique_instance_labels = instance_labels;
  unique_instance_labels.erase(std::unique(unique_instance_labels.begin(), unique_instance_labels.end()),
                               unique_instance_labels.end());

  std::stringstream ss;
  ss << "Unique instance labels: ";
  for (int i = 0; i < unique_instance_labels.size(); ++i)
    ss << unique_instance_labels[i] << " ";
  ss << std::endl;

  LOG(INFO) << ss.str();
}

Feature::Ptr FeatureTracker::constructStaticFeature(const ImagePacket& images, const cv::KeyPoint& kp, size_t age,
                                                    size_t tracklet_id, size_t frame_id) const
{
  const int& x = kp.pt.x;
  const int& y = kp.pt.y;
  const int& rows = images.rgb.rows;
  const int& cols = images.rgb.cols;
  if (images.semantic_mask.at<int>(y, x) != Feature::background)
  {
    return nullptr;
  }

  if (images.depth.at<double>(y, x) > tracking_params_.depth_background_thresh || images.depth.at<double>(y, x) <= 0)
  {
    return nullptr;
  }

  // check flow
  double flow_xe = static_cast<double>(images.flow.at<cv::Vec2f>(y, x)[0]);
  double flow_ye = static_cast<double>(images.flow.at<cv::Vec2f>(y, x)[1]);

  if (!(flow_xe != 0 && flow_ye != 0))
  {
    return nullptr;
  }

  // check predicted flow is within image
  cv::KeyPoint predicted_kp(x + flow_xe, y + flow_ye, 0, 0, 0, kp.octave, -1);
  if (predicted_kp.pt.x >= cols || predicted_kp.pt.y >= rows || predicted_kp.pt.x <= 0 || predicted_kp.pt.y <= 0)
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

  // the flow is not actually what
  feature->optical_flow = cv::Point2d(flow_xe, flow_ye);
  feature->predicted_keypoint = predicted_kp;

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
