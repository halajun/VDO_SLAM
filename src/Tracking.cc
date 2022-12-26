#include "Tracking.h"
#include "ORBextractor.h"
#include "Camera.h"
#include "Frontend-Definitions.h"
#include "UtilsGtsam.h"
#include "utils/UtilsOpenCV.h"
#include "Tracking-Tools.h"
#include "Optimizer.h"

#include <opencv2/opencv.hpp>
#include <glog/logging.h>
#include <algorithm>

namespace vdo
{
Tracking::Tracking(const TrackingParams& params_, const Camera& camera_) : params(params_), camera(camera_), static_tracklet_map_()
{
  feature_tracker = vdo::make_unique<FeatureTracker>(params_, camera_);
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
    frame->pose_ = gtsam::Pose3::identity();
    LOG(INFO) << "Initalising pose identity matrix " << frame->pose_;
  }

  // set initalisatiobn
  previous_frame_ = frame;
  state = State::kNominal;
  return std::make_shared<FrontendOutput>(frame);
}

FrontendOutput::Ptr Tracking::processNominal(const InputPacket& input,
                                             GroundTruthInputPacket::ConstOptional ground_truth)
{
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
  solveInitalCamModel(previous_frame_, frame);

  double t_error_before_opt, r_error_before_opt, t_error_after_opt, r_error_after_opt;
  calculatePoseError(frame->pose_, ground_truth->X_wc, t_error_before_opt, r_error_before_opt);

  PoseOptimizationFlow2Cam flow2camOpt(camera);
  flow2camOpt(previous_frame_, frame);

   calculatePoseError(frame->pose_, ground_truth->X_wc, t_error_after_opt, r_error_after_opt);
  LOG(INFO) << std::fixed << "ATE Errors:\n"
            << "Error before flow opt: t - " << t_error_before_opt << ", r - " << r_error_before_opt << "\n"
            << "Error after flow opt: t - " << t_error_after_opt << ", r - " << r_error_after_opt << "\n";


  // updateStaticTrackletMap(frame->features_);

  // LOG(INFO) << "After tracking optical flow/new tracks - " << n_optical_flow << "/" <<
  // static_cast<int>(n_new_tracks); LOG(INFO) << "After processing - feature size " << frame->static_features.size();

  // frame->processDynamicFeatures(params.depth_obj_thresh);
  // frame->projectKeypoints(camera);


  if (ground_truth)
  {
    frame->ground_truth_ = ground_truth;
    LOG(INFO) << "Initalising pose using ground truth " << frame->ground_truth_->X_wc;
  }
  displayFeatures(*frame);

  // frame_logger.log(*frame);

  previous_frame_ = frame;
  FrontendOutput::Ptr output = std::make_shared<FrontendOutput>(frame);

  for(Feature::Ptr feature : frame->features_) {
    if(!feature->inlier) {
      continue;
    }
    Landmark lmk;
    camera.backProject(feature->keypoint, feature->depth, &lmk);
    output->tracklet_landmark_map_.insert({feature->tracklet_id, lmk});
  }
  return output;

}

bool Tracking::solveInitalCamModel(Frame::Ptr previous_frame, Frame::Ptr current_frame)
{
  bool solve_in_camera_frame = true;

  std::vector<cv::Point2f> current_2d;
  std::vector<cv::Point3f> previous_3d;
  TrackletIds tracklet_ids;  // feature tracklets from the current frame so we can assign values as inliers

  cv::Mat viz;
  current_frame->images_.rgb.copyTo(viz);

  for(Feature::Ptr current_feature : current_frame->features_) {
    const std::size_t& tracklet_id = current_feature->tracklet_id;

    Feature::Ptr previous_feature = previous_frame->getByTrackletId(tracklet_id);
    if (!previous_feature)
    {
      continue;
    }

    CHECK_EQ(previous_feature->tracklet_id, current_feature->tracklet_id);
    CHECK_EQ(previous_feature->tracklet_id, tracklet_id);

    //dont think this shoudl happen as we only track (old) points if it is an inlier! see staticTrackOpticalFlow
    if (!previous_feature->inlier)
    {
      current_feature->inlier = false;
      continue;
    }

    utils::DrawCircleInPlace(viz, current_feature->keypoint.pt, cv::Scalar(0, 255, 0));
    cv::arrowedLine(viz, previous_feature->keypoint.pt, current_feature->keypoint.pt,cv::Scalar(0, 0, 255));
    utils::DrawCircleInPlace(viz, current_feature->refined_keypoint.pt, cv::Scalar(255, 0, 0), 0.8);

    current_2d.push_back(current_feature->refined_keypoint.pt);
    Landmark lmk;
    camera.backProject(previous_feature->keypoint, previous_feature->depth, &lmk);
    if (solve_in_camera_frame)
    {
      cv::Point3f lmk_f(static_cast<float>(lmk.x()), static_cast<float>(lmk.y()), static_cast<float>(lmk.z()));
      previous_3d.push_back(lmk_f);
      tracklet_ids.push_back(tracklet_id);
    }
    else
    {
      // solve as standard PnP formaulation and solve in world frame
      // lmk is in camera frame. Put into world frame
      Landmark lmk_world = previous_frame->pose_.transformFrom(lmk);
      cv::Point3f lmk_f(static_cast<float>(lmk_world.x()), static_cast<float>(lmk_world.y()),
                        static_cast<float>(lmk_world.z()));
      previous_3d.push_back(lmk_f);
      tracklet_ids.push_back(tracklet_id);
    }

  }

  cv::imshow("Optical flow", viz);
  cv::waitKey(1);

  // TODO: if we have zero tracket features -> this means need to fix the way the frontend takes feature points so we
  // never completely clear our tracks
  if (previous_3d.size() < 5)
  {
    LOG(WARNING) << "Cannot run PnP with < 5 points";
    current_frame->pose_ = previous_frame->pose_;
    return false;
  }

  //sanity check
  CHECK_EQ(previous_3d.size(), tracklet_ids.size());


  LOG(INFO) << "Solving initial camera model with  " << previous_3d.size() << " features";

  const cv::Mat& K = camera.Params().K;
  const cv::Mat& D = camera.Params().D;

  // output
  cv::Mat Rvec(3, 1, CV_64FC1);
  cv::Mat Tvec(3, 1, CV_64FC1);
  cv::Mat Rot(3, 3, CV_64FC1);
  cv::Mat pnp_inliers;  // a [1 x N] vector

  // solve
  int iter_num = 500;
  double reprojectionError = 0.4, confidence = 0.98;  // 0.5 0.3
  cv::solvePnPRansac(previous_3d, current_2d, K, D, Rvec, Tvec, false, iter_num, reprojectionError, confidence,
                     pnp_inliers, cv::SOLVEPNP_AP3P);  // AP3P EPNP P3P ITERATIVE DLS
  // LOG(INFO) << "inliers " << pnp_inliers;

  cv::Rodrigues(Rvec, Rot);
  // vector of tracklet id's that PnP ransac marked as inliers
  TrackletIds pnp_tracklet_inliers, pnp_tracklet_outliers;
  for (size_t i = 0; i < pnp_inliers.rows; i++)
  {
    int inlier_index = pnp_inliers.at<int>(i);
    size_t inlier_tracklet = tracklet_ids[inlier_index];
    pnp_tracklet_inliers.push_back(inlier_tracklet);
  }

  //calculate outliers
  determineOutlierIds(pnp_tracklet_inliers, tracklet_ids, pnp_tracklet_outliers);
  CHECK_EQ((pnp_tracklet_inliers.size() + pnp_tracklet_outliers.size()), tracklet_ids.size());
  CHECK_EQ(pnp_tracklet_inliers.size(), pnp_inliers.rows);
  LOG(INFO) << "Inliers/total = " << pnp_inliers.rows << "/" << previous_3d.size();

  // mark all features in current frame as either inlier or outlier
  for(TrackletId inlier_id : pnp_tracklet_inliers) {
    Feature::Ptr feature = current_frame->getByTrackletId(inlier_id);
    CHECK_NOTNULL(feature);
    feature->inlier = true;
  }

  for(TrackletId outlier_id : pnp_tracklet_outliers) {
    Feature::Ptr feature = current_frame->getByTrackletId(outlier_id);
    CHECK_NOTNULL(feature);
    feature->inlier = false;
  }

  gtsam::Pose3 pose = utils::cvMatsToGtsamPose3(Rot, Tvec).inverse();
  gtsam::Pose3 relative_pose;

  if (solve_in_camera_frame)
  {
    // need to compose this pose and the previous pose as "pose" as solved by PnP should be a relative pose
    pose = previous_frame->pose_ * pose;
  }

  current_frame->pose_ = pose;
  LOG(INFO) << current_frame->pose_;
  return true;
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

  cv::imshow("Frames", disp);
  cv::waitKey(1);
}

void Tracking::updateStaticTrackletMap(const FeaturePtrs& static_features ) {
  for(Feature::Ptr feature : static_features) {
    const size_t tracklet_id = feature->tracklet_id;
    //new tracklet
    if(static_tracklet_map_.find(tracklet_id) == static_tracklet_map_.end()) {
      FeaturePtrs tracklet{feature};
      static_tracklet_map_.insert({tracklet_id, tracklet});
    }
    else {
      //add feature to existing tracklet
      static_tracklet_map_.at(tracklet_id).push_back(feature);
    }
  }
}

}  // namespace vdo