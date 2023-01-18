#include "Tracking.h"
#include "ORBextractor.h"
#include "Camera.h"
#include "Frontend-Definitions.h"
#include "utils/UtilsGtsam.h"
#include "utils/UtilsOpenCV.h"
#include "Tracking-Tools.h"
#include "Optimizer.h"
#include "MotionSolver.h"
#include "viz/Display.h"

#include <opencv2/opencv.hpp>
#include <glog/logging.h>
#include <algorithm>
#include <map>

namespace vdo
{
Tracking::Tracking(const TrackingParams& params_, const Camera& camera_)
  : params(params_), camera(camera_), static_tracklet_map_()
{
  feature_tracker = vdo::make_unique<FeatureTracker>(params_, camera_);
}

Tracking::~Tracking()
{
  VLOG(1) << "Writing all metrics to file";
  std::vector<FrontendMetrics*> metric_pointers;
  for (FrontendMetrics& m : metrics)
  {
    metric_pointers.push_back(&m);
  }

  saveArchiveAsXML<std::vector<FrontendMetrics*>>("frontend_metrics.xml", metric_pointers);
}

FrontendOutput::Ptr Tracking::process(const InputPacket& input, GroundTruthInputPacket::ConstOptional ground_truth)
{
  FrontendOutput::Ptr output = nullptr;
  if (state == State::kBoostrap)
  {
    output = processBoostrap(input, ground_truth);
  }
  else if (state == State::kNominal)
  {
    output = processNominal(input, ground_truth);
  }

  return output;
}

void Tracking::updateFromBackend(const BackendOutput& backend_output)
{
  if (!previous_frame_)
  {
    return;
  }
  previous_frame_->pose_ = backend_output.estimated_pose_;
  // update motion model?
}

FrontendOutput::Ptr Tracking::processBoostrap(const InputPacket& input,
                                              GroundTruthInputPacket::ConstOptional ground_truth)
{
  const Timestamp& timestamp = input.timestamp;
  const size_t& frame_id = input.frame_id;
  ImagePacket images;
  preprocessInput(input, images);

  InputPacket processed_input(input.timestamp, input.frame_id, images.rgb, images.depth, images.flow,
                              images.semantic_mask);

  size_t n_optical_flow, n_new_tracks;
  Frame::Ptr frame = feature_tracker->track(processed_input, n_optical_flow, n_new_tracks);
  // updateStaticTrackletMap(frame->features_);
  // frame->detectFeatures(feature_detector);
  // frame->processStaticFeatures(params.depth_background_thresh);
  displayFeatures(*frame);
  // frame->projectKeypoints(camera);
  // use ground truth to initalise pose
  if (ground_truth)
  {
    frame->pose_ = ground_truth->X_wc;
    frame->ground_truth_ = ground_truth;
    LOG(INFO) << "Initalising pose using ground truth " << frame->pose_;
  }
  else
  {
    frame->pose_ = gtsam::Pose3::Identity();
    LOG(INFO) << "Initalising pose identity matrix " << frame->pose_;
  }

  // trackDynamicObjects(frame);

  // set initalisatiobn
  previous_frame_ = frame;
  state = State::kNominal;
  return std::make_shared<FrontendOutput>(frame);
}

FrontendOutput::Ptr Tracking::processNominal(const InputPacket& input,
                                             GroundTruthInputPacket::ConstOptional ground_truth)
{
  FrontendMetrics metric;

  const Timestamp& timestamp = input.timestamp;
  const size_t& frame_id = input.frame_id;
  ImagePacket images;
  preprocessInput(input, images);
  InputPacket processed_input(input.timestamp, input.frame_id, images.rgb, images.depth, images.flow,
                              images.semantic_mask);
  // optical flow tracking failed so we need to redetect all features
  size_t n_optical_flow, n_new_tracks;
  Frame::Ptr frame = feature_tracker->track(processed_input, n_optical_flow, n_new_tracks);

  // estimate initialPos
  // assumes we have set the motion model from the previous frame
  if (!solveInitalCamModel(previous_frame_, frame))
  {
    LOG(ERROR) << "Solve PnP or Motion failed";
  }

  double t_error_before_opt, r_error_before_opt, t_error_after_opt, r_error_after_opt;
  double rel_t_error_before_opt, rel_r_error_before_opt, rel_t_error_after_opt, rel_r_error_after_opt;
  calculatePoseError(frame->pose_, ground_truth->X_wc, t_error_before_opt, r_error_before_opt);
  calculateRelativePoseError(previous_frame_->pose_, frame->pose_, previous_frame_->ground_truth_->X_wc,
                             ground_truth->X_wc, rel_t_error_before_opt, rel_r_error_before_opt);

  PoseOptimizationFlow2Cam flow2camOpt(camera);
  flow2camOpt(previous_frame_, frame);

  calculatePoseError(frame->pose_, ground_truth->X_wc, t_error_after_opt, r_error_after_opt);
  calculateRelativePoseError(previous_frame_->pose_, frame->pose_, previous_frame_->ground_truth_->X_wc,
                             ground_truth->X_wc, rel_t_error_after_opt, rel_r_error_after_opt);
  LOG(INFO) << std::fixed << "ATE Errors:\n"
            << "Error before flow opt: t - " << t_error_before_opt << ", r - " << r_error_before_opt << "\n"
            << "Error after flow opt: t - " << t_error_after_opt << ", r - " << r_error_after_opt << "\n";
  LOG(INFO) << std::fixed << "RTE Errors:\n"
            << "Error before flow opt: t - " << rel_t_error_before_opt << ", r - " << rel_r_error_before_opt << "\n"
            << "Error after flow opt: t - " << rel_t_error_after_opt << ", r - " << rel_r_error_after_opt << "\n";

  if (ground_truth)
  {
    frame->ground_truth_ = ground_truth;
    // frame->pose_ = frame->ground_truth_->X_wc;
    LOG(INFO) << "Initalising pose using ground truth " << frame->ground_truth_->X_wc;
  }
  displayFeatures(*frame);
  tracking_tools::updateMotionModel(previous_frame_, frame);

  ObjectObservations objects;
  trackDynamicObjects(frame, objects);

  // book keeping and collect metrics
  metric.frame_id = frame_id;
  metric.timestamp = timestamp;
  metric.gt_pose = ground_truth->X_wc;
  metric.pose = frame->pose_;

  metric.ate_before_flow.translation = t_error_before_opt;
  metric.ate_before_flow.rot = r_error_before_opt;
  metric.rte_before_flow.translation = rel_t_error_before_opt;
  metric.rte_before_flow.rot = rel_r_error_before_opt;

  metric.ate_after_flow.translation = t_error_after_opt;
  metric.ate_after_flow.rot = r_error_after_opt;
  metric.rte_after_flow.translation = rel_t_error_after_opt;
  metric.rte_after_flow.rot = rel_r_error_after_opt;

  metrics.push_back(metric);

  previous_frame_ = frame;
  FrontendOutput::Ptr output = std::make_shared<FrontendOutput>(frame);
  output->objects_ = objects;

  for (Feature::Ptr feature : frame->features_)
  {
    if (!feature->inlier)
    {
      continue;
    }
    Landmark lmk;
    camera.backProject(feature->keypoint, feature->depth, &lmk);
    output->tracklet_landmark_map_.insert({ feature->tracklet_id, lmk });
  }
  return output;
}

bool Tracking::solveInitalCamModel(Frame::Ptr previous_frame, Frame::Ptr current_frame)
{
  KeypointsCV current_2d;    // current 2d observations
  Landmarks previous_3d;     // corresponding 3d points in the camera frame of the previous pose
  TrackletIds tracklet_ids;  // feature tracklets from the current frame so we can assign values as inliers

  // collect all 2d obs and previous 3d correspondences
  for (Feature::Ptr current_feature : current_frame->features_)
  {
    const std::size_t& tracklet_id = current_feature->tracklet_id;

    Feature::Ptr previous_feature = previous_frame->getByTrackletId(tracklet_id);
    if (!previous_feature)
    {
      continue;
    }

    CHECK_EQ(previous_feature->tracklet_id, current_feature->tracklet_id);
    CHECK_EQ(previous_feature->tracklet_id, tracklet_id);

    // dont think this shoudl happen as we only track (old) points if it is an inlier! see staticTrackOpticalFlow
    if (!previous_feature->inlier)
    {
      current_feature->inlier = false;
      continue;
    }

    float dist = std::sqrt((current_feature->keypoint.pt.x - previous_feature->keypoint.pt.x) *
                               (current_feature->keypoint.pt.x - previous_feature->keypoint.pt.x) +
                           (current_feature->keypoint.pt.y - previous_feature->keypoint.pt.y) *
                               (current_feature->keypoint.pt.y - previous_feature->keypoint.pt.y));
    if (dist > 100)
    {
      // cv::arrowedLine(viz, previous_feature->keypoint.pt, current_feature->keypoint.pt,cv::Scalar(255, 0, 0));
      LOG(INFO) << current_feature->age << " " << previous_feature->age << " " << current_feature->tracklet_id;
    }

    current_2d.push_back(current_feature->keypoint);
    Landmark lmk;
    camera.backProject(previous_feature->keypoint, previous_feature->depth, &lmk);
    previous_3d.push_back(lmk);
    tracklet_ids.push_back(tracklet_id);
  }

  MotionSolver::Options solver_options;
  MotionSolver::Result result =
      MotionSolver::solvePnpOrMotion(current_2d, previous_3d, tracklet_ids, previous_frame->pose_,
                                     previous_frame->motion_model_, camera, solver_options);

  LOG(INFO) << "Motion solved using model "
            << (result.model == MotionSolver::Result::Selection::PNP ? "PnPRansac" : "Motion");

  for (TrackletId inlier_id : result.inliers)
  {
    Feature::Ptr feature = current_frame->getByTrackletId(inlier_id);
    CHECK_NOTNULL(feature);
    feature->inlier = true;
  }

  for (TrackletId outlier_id : result.outliers)
  {
    Feature::Ptr feature = current_frame->getByTrackletId(outlier_id);
    CHECK_NOTNULL(feature);
    feature->inlier = false;
  }

  current_frame->pose_ = result.pose;
  return result.success;
}

// function does a lot -> sets scene flow and marks features with object ID
size_t Tracking::trackDynamicObjects(Frame::Ptr frame, ObjectObservations& objects)
{
  const cv::Mat& rgb = frame->images_.rgb;
  const cv::Mat& semantic_mask = frame->images_.semantic_mask;
  const cv::Mat& flow = frame->images_.flow;

  cv::Mat viz, heat_map;
  rgb.copyTo(viz);
  rgb.copyTo(heat_map);
  objects.clear();

  // std::vector<FeaturePtrs> object_ids;
  // ensure objects are not on the boundary
  static constexpr float kRowBoundary = 25;
  static constexpr float kColBoundary = 50;
  // at least half of the object is visible?
  static constexpr double KCountThreshold = 0.5;
  // semantic map is construced in the feature tracking after dynamicTrack
  for (auto instance_feature_pair : frame->semantic_instance_map_)
  {
    const int instance_label = instance_feature_pair.first;
    FeaturePtrs features = instance_feature_pair.second;
    int object_count = 0;  // number of valid features
    int dynamic_tracklet_count = 0;
    int static_tracklet_count = 0;
    int sf_count = 0;
    int previous_object_id = -1;
    std::vector<int> sf_range(10, 0);
    FeaturePtrs dynamic_features_after_thresh;
    for (Feature::Ptr feature : features)
    {
      // for(size_t j = 0; j < predicted_labels[i].size(); j++ ) {
      Feature::Ptr current_feature = feature;
      const KeypointCV& kp = current_feature->keypoint;
      const float u = kp.pt.x;
      const float v = kp.pt.y;

      // we could not track from the previous one?
      Feature::Ptr previous_feature = previous_frame_->getDynamicByTrackletId(current_feature->tracklet_id);
      // all the features should have the same instance label after propogation from udpateMask...
      // TODO: dont just use the last object id...

      if (v > kRowBoundary || v < (rgb.rows - kRowBoundary) || u > kColBoundary || u < (rgb.cols - kColBoundary))
      {
        object_count++;

        if (previous_feature)
        {
          previous_object_id = previous_feature->object_id;

          // we could track however the previous feature may have been marked as static from optical/scene flow?
          if (previous_feature->type == Feature::Type::DYNAMIC)
            dynamic_tracklet_count++;
          if (previous_feature->type == Feature::Type::STATIC)
            static_tracklet_count++;

          // at this point frame MUST have an pose estimate from the frontend
          // is it to calculate the sceneflow
          Landmark lmk_previous, lmk_current;
          camera.backProject(previous_feature->keypoint, previous_feature->depth, &lmk_previous);
          camera.backProject(current_feature->keypoint, current_feature->depth, &lmk_current);

          gtsam::Pose3 previous_pose = previous_frame_->pose_;
          gtsam::Pose3 current_pose = frame->pose_;
          // convert to world frame
          lmk_previous = previous_pose.transformFrom(lmk_previous);
          lmk_current = current_pose.transformFrom(lmk_current);

          Landmark flow_world = lmk_current - lmk_previous;
          gtsam::Pose3 flow_world_transform(gtsam::Rot3::Identity(), flow_world);
          // this is in the world frame
          // we conver to camera frame which will be the flow from the previous point to the current point
          // //^c_{t-1}F_t = ^cX_{t-1} * ^w_{t-1}F_t * ^cX_{t-1}^{-1}
          gtsam::Pose3 flow_prev_curr_transform = previous_pose.inverse() * flow_world_transform * previous_pose;
          gtsam::Point3 flow_prev_curr = flow_prev_curr_transform.translation();
          feature->scene_flow = flow_prev_curr;

          double scene_flow_norm = flow_prev_curr.norm();

          if (scene_flow_norm > params.scene_flow_magnitude)
          {
            dynamic_features_after_thresh.push_back(current_feature);
            sf_count++;
            // utils::DrawCircleInPlace(viz, feature->keypoint.pt, cv::Scalar(0, 0, 255));
          }

          {
            if (0.0 <= scene_flow_norm && scene_flow_norm < 0.05)
              sf_range[0] = sf_range[0] + 1;
            else if (0.05 <= scene_flow_norm && scene_flow_norm < 0.1)
              sf_range[1] = sf_range[1] + 1;
            else if (0.1 <= scene_flow_norm && scene_flow_norm < 0.2)
              sf_range[2] = sf_range[2] + 1;
            else if (0.2 <= scene_flow_norm && scene_flow_norm < 0.4)
              sf_range[3] = sf_range[3] + 1;
            else if (0.4 <= scene_flow_norm && scene_flow_norm < 0.8)
              sf_range[4] = sf_range[4] + 1;
            else if (0.8 <= scene_flow_norm && scene_flow_norm < 1.6)
              sf_range[5] = sf_range[5] + 1;
            else if (1.6 <= scene_flow_norm && scene_flow_norm < 3.2)
              sf_range[6] = sf_range[6] + 1;
            else if (3.2 <= scene_flow_norm && scene_flow_norm < 6.4)
              sf_range[7] = sf_range[7] + 1;
            else if (6.4 <= scene_flow_norm && scene_flow_norm < 12.8)
              sf_range[8] = sf_range[8] + 1;
            // else if (12.8 <= scene_flow_norm && scene_flow_norm < 25.6)
            //   sf_range[9] = sf_range[9] + 1;
            else
              sf_range[9]++;
          }
        }
      }
    }
    // most parts of this object are on the image boundary
    LOG(INFO) << " object count " << object_count;
    if (object_count < KCountThreshold)
    {
      for (Feature::Ptr feature : features)
      {
        feature->object_id = -1;
      }
      // then continue?
      // LOG(INFO) <<
      continue;
    }

    if (features.size() < 50)
    {
      // LOG(INFO) << "Object " << instance_label << " did not have enough points";
      continue;
    }

    double average_dynamic_tracks = dynamic_tracklet_count / object_count;  // average of how many points
    double average_static_tracks = static_tracklet_count / object_count;
    std::cout << "scene flow distribution:" << std::endl;
    for (int j = 0; j < sf_range.size(); ++j)
      std::cout << sf_range[j] << " ";
    std::cout << std::endl;

    // if not enough of the object is dynamic label all features as static
    double average_flow_count = (double)sf_count / (double)object_count;
    LOG(INFO) << "Num points that are dynamic " << average_flow_count << "/" << params.scene_flow_percentage;
    if (average_flow_count > params.scene_flow_percentage)
    {
      // for(Feature::Ptr feature : dynamic_features_after_thresh) {
      //     utils::DrawCircleInPlace(viz, feature->keypoint.pt, Display::getObjectColour(instance_label));

      // }
      LOG(INFO) << "Object  " << instance_label << " was tracked - " << dynamic_tracklet_count << "/" << object_count;

      ObjectObservation object_observation;
      object_observation.object_label_ = instance_label;
      object_observation.average_flow_ = average_flow_count;
      object_observation.frame_ = frame;

      int center_x = 0, center_y = 0;
      for (Feature::Ptr feature : features)
      {
        utils::DrawCircleInPlace(viz, feature->keypoint.pt, Display::getObjectColour(instance_label));
        feature->object_id = instance_label;
        center_x += feature->keypoint.pt.x;
        center_y += feature->keypoint.pt.y;
        object_observation.object_features_.push_back(feature->tracklet_id);

        gtsam::Point3 normalized_scene_flow = feature->scene_flow.normalized();

        cv::Scalar colour(255 * normalized_scene_flow.x(), 255 * normalized_scene_flow.y(),
                          255 * normalized_scene_flow.z());
        utils::DrawCircleInPlace(heat_map, feature->keypoint.pt, colour);

        // heat_map.at<cv::Vec3f>(feature->keypoint.pt)[0] = feature->scene_flow.x();
        // heat_map.at<cv::Vec3f>(feature->keypoint.pt)[1] = feature->scene_flow.y();
        // heat_map.at<cv::Vec3f>(feature->keypoint.pt)[2] = feature->scene_flow.z();
        // heat_map.at<cv::Vec3f>(feature->keypoint.pt)[0] = feature->scene_flow.norm();
        // heat_map.at<cv::Vec3f>(feature->keypoint.pt)[1] = feature->scene_flow.norm();
        // heat_map.at<cv::Vec3f>(feature->keypoint.pt)[2] = feature->scene_flow.norm();
      }
      center_x /= features.size();
      center_y /= features.size();

      object_observation.object_center_ = gtsam::Point2(center_x, center_y);
      objects.push_back(object_observation);

      cv::putText(viz,                                                // target image
                  std::to_string(instance_label),                     // text
                  cv::Point(center_x, center_y),                      // top-left position
                  cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(118, 185, 0),  // font color
                  3);
    }
  }

  cv::imshow("Scene flow", viz);
  cv::imshow("Flow Heat map", heat_map);
  cv::waitKey(1);
  return objects.size();

  // //for each feature within a unique label try and propogate the previous actual object label

  // LOG(INFO) << "Object ids size - " << object_ids.size();
}

void Tracking::preprocessInput(const InputPacket& input, ImagePacket& images)
{
  input.images.rgb.copyTo(images.rgb);
  input.images.flow.copyTo(images.flow);
  input.images.semantic_mask.copyTo(images.semantic_mask);

  input.images.depth.copyTo(images.depth);
  processInputDepth(input.images.depth, images.depth);
}

void Tracking::processInputDepth(const cv::Mat& disparity, cv::Mat& depth)
{
  for (int i = 0; i < disparity.rows; i++)
  {
    for (int j = 0; j < disparity.cols; j++)
    {
      if (disparity.at<double>(i, j) < 0)
      {
        depth.at<double>(i, j) = 0;
      }
      else
      {
        // if (mTestData==OMD)
        // {
        //     // --- for stereo depth map ---
        //     imD.at<float>(i,j) = mbf/(imD.at<float>(i,j)/mDepthMapFactor);
        //     // --- for RGB-D depth map ---
        //     // imD.at<float>(i,j) = imD.at<float>(i,j)/mDepthMapFactor;
        // }
        // else if (mTestData==KITTI)
        // {
        //     // --- for stereo depth map ---
        //     imD.at<float>(i,j) = mbf/(imD.at<float>(i,j)/mDepthMapFactor);
        //     // --- for monocular depth map ---
        //     // imD.at<float>(i,j) = imD.at<float>(i,j)/500.0;
        // }
        depth.at<double>(i, j) = camera.Baseline() / (disparity.at<double>(i, j) / params.depth_scale_factor);
      }
    }
  }
}

void Tracking::displayFeatures(const Frame& frame)
{
  cv::Mat disp;
  const ImagePacket& images = frame.images_;
  images.rgb.copyTo(disp);

  for (const Feature::Ptr feature : frame.features_)
  {
    cv::Point2d point(feature->keypoint.pt.x, feature->keypoint.pt.y);
    // if (feature.inlier)
    // {
    //   cv::arrowedLine(disp, feature.keypoint.pt, feature.predicted_keypoint.pt, cv::Scalar(255, 0, 0));
    //   cv::putText(disp, std::to_string(feature.tracklet_id),
    //               cv::Point2i(feature.keypoint.pt.x - 10, feature.keypoint.pt.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.3,
    //               cv::Scalar(255, 0, 0));
    // }
    // else
    // {
    //   utils::DrawCircleInPlace(disp, point, cv::Scalar(0, 0, 255), 1);
    // }
    if (feature->age == 0)
    {
      vdo::utils::DrawCircleInPlace(disp, point, cv::Scalar(0, 255, 0), 1);
    }
    else
    {
      vdo::utils::DrawCircleInPlace(disp, point, cv::Scalar(0, 0, 255), 1);
    }
  }

  // for (const Feature::Ptr feature : frame.dynamic_features_)
  // {
  //   cv::Point2d point(feature->keypoint.pt.x, feature->keypoint.pt.y);
  //   // if (feature.inlier)
  //   // {
  //     cv::arrowedLine(disp, feature->keypoint.pt, feature->predicted_keypoint.pt, cv::Scalar(255, 0, 0));
  //     // cv::putText(disp, std::to_string(feature.tracklet_id),
  //   //               cv::Point2i(feature.keypoint.pt.x - 10, feature.keypoint.pt.y - 10), cv::FONT_HERSHEY_SIMPLEX,
  //   0.3,
  //   //               cv::Scalar(255, 0, 0));
  //   // }
  //   // else
  //   // {
  //   // utils::DrawCircleInPlace(disp, point, Display::getObjectColour(feature->instance_label), 1);
  //   // }

  // }

  cv::imshow("Frames", disp);
  cv::waitKey(1);
}

void Tracking::updateStaticTrackletMap(const FeaturePtrs& static_features)
{
  for (Feature::Ptr feature : static_features)
  {
    const size_t tracklet_id = feature->tracklet_id;
    // new tracklet
    if (static_tracklet_map_.find(tracklet_id) == static_tracklet_map_.end())
    {
      FeaturePtrs tracklet{ feature };
      static_tracklet_map_.insert({ tracklet_id, tracklet });
    }
    else
    {
      // add feature to existing tracklet
      static_tracklet_map_.at(tracklet_id).push_back(feature);
    }
  }
}

}  // namespace vdo
