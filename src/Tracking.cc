#include "Tracking.h"
#include "ORBextractor.h"
#include "Camera.h"
#include "Frontend-Definitions.h"
#include "UtilsGtsam.h"
#include "utils/UtilsOpenCV.h"
#include "Tracking-Tools.h"
#include "Optimizer.h"
#include "viz/Display.h"

#include <opencv2/opencv.hpp>
#include <glog/logging.h>
#include <algorithm>
#include <map>

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
  //update motion model?
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

  // trackDynamicObjects(frame);

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
    //assumes we have set the motion model from the previous frame
  solveInitalCamModel(previous_frame_, frame);

  double t_error_before_opt, r_error_before_opt, t_error_after_opt, r_error_after_opt;
  double rel_t_error_before_opt, rel_r_error_before_opt, rel_t_error_after_opt, rel_r_error_after_opt;
  calculatePoseError(frame->pose_, ground_truth->X_wc, t_error_before_opt, r_error_before_opt);
  calculateRelativePoseError(
    previous_frame_->pose_, frame->pose_, previous_frame_->ground_truth_->X_wc, ground_truth->X_wc, 
    rel_t_error_before_opt, rel_r_error_before_opt);

  PoseOptimizationFlow2Cam flow2camOpt(camera);
  flow2camOpt(previous_frame_, frame);

  calculatePoseError(frame->pose_, ground_truth->X_wc, t_error_after_opt, r_error_after_opt);
  calculateRelativePoseError(
    previous_frame_->pose_, frame->pose_, previous_frame_->ground_truth_->X_wc, ground_truth->X_wc, 
    rel_t_error_after_opt, rel_r_error_after_opt);
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
  updateMotionModel(previous_frame_, frame);

  trackDynamicObjects(frame);

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

    // utils::DrawCircleInPlace(viz, current_feature->keypoint.pt, cv::Scalar(0, 255, 0));

    // // if(current_feature->age > ) {
    //       cv::arrowedLine(viz, previous_feature->keypoint.pt, current_feature->keypoint.pt,cv::Scalar(0, 0, 255));
    // // }
     float dist = std::sqrt(
            (current_feature->keypoint.pt.x - previous_feature->keypoint.pt.x) * (current_feature->keypoint.pt.x - previous_feature->keypoint.pt.x) +
            (current_feature->keypoint.pt.y - previous_feature->keypoint.pt.y) * (current_feature->keypoint.pt.y - previous_feature->keypoint.pt.y));
      if(dist > 100) {
        // cv::arrowedLine(viz, previous_feature->keypoint.pt, current_feature->keypoint.pt,cv::Scalar(255, 0, 0));
        LOG(INFO) << current_feature->age << " " << previous_feature->age << " " << current_feature->tracklet_id;
      }

    current_2d.push_back(current_feature->keypoint.pt);
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

  // cv::imshow("Optical flow", viz);
  // cv::waitKey(1);

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
  LOG(INFO) << "PNP Inliers/total = " << pnp_inliers.rows << "/" << previous_3d.size();

  // // mark all features in current frame as either inlier or outlier
  //TODO: after the motion estimation
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

  gtsam::Pose3 pnp_pose = utils::cvMatsToGtsamPose3(Rot, Tvec).inverse();
  gtsam::Pose3 relative_pose;

  if (solve_in_camera_frame)
  {
    // need to compose this pose and the previous pose as "pose" as solved by PnP should be a relative pose
    pnp_pose = previous_frame->pose_ * pnp_pose;
  }

  LOG(INFO) << "pnp pose\n\n" << pnp_pose;
  LOG(INFO) << "motion_model\n\n" << previous_frame->motion_model_;

  //TODO: make functions
  //using current velocity estimate, step from previous pose to current pose
  //pose in in world frame
  gtsam::Pose3 pose_from_motion_model = previous_frame->pose_ * previous_frame->motion_model_;
  //iterate through all the points
  float total_reprojection_error = 0.0;
  float total_reprojection_error_inliers = 0.0;
  int successful_projections = 0;
  TrackletIds motion_tracklet_inliers, motion_tracklet_outliers;
  for(TrackletId i = 0; i < tracklet_ids.size(); i++) {
    const TrackletId tracklet_id = tracklet_ids[i];
    Landmark lmk_camera; //the previous 3d point as projected into the current camera frame using the motion model
    KeypointCV kp_observed; //the 2d observation of the projected kp
    kp_observed.pt = current_2d[i];

    cv::Point3f cv_lmk_previous = previous_3d[i];
    gtsam::Point3 lmk_previous(static_cast<double>(cv_lmk_previous.x),
      static_cast<double>(cv_lmk_previous.y), static_cast<double>(cv_lmk_previous.z));

    if (solve_in_camera_frame)
    {
      //lmk_previous will be in camera frame of the previous camera pose
      //we need to translate it into the current camera pose
      //we use the motion model which should be the transform between C_{t-1} and C_t
      // lmk_camera = previous_frame->motion_model_.inverse() * lmk_previous;
      lmk_camera = previous_frame->motion_model_.inverse() * lmk_previous;

    }
    else
    {
     //lmk_previous will be in the world frame abd we want it in the camera frame
     lmk_camera = pose_from_motion_model.inverse() * lmk_previous;
    }

    //at this point lmk camera is a landmark in the reference frame of camera t and kp is the observation
    //in the image frame
    //calcualte reporojection error
    KeypointCV projected;
    bool is_contained = camera.isLandmarkContained(lmk_camera, projected);
    if(!is_contained) {
      continue;
    }
    else {
      successful_projections++;
    }

    const float u_err = kp_observed.pt.x - projected.pt.x;
    const float v_err = kp_observed.pt.y - projected.pt.y;
    const float reprojection_err = std::sqrt(u_err * u_err + v_err * v_err);
    total_reprojection_error+=reprojection_err;

    if(reprojection_err < static_cast<float>(reprojectionError)) {
      motion_tracklet_inliers.push_back(tracklet_id);
      total_reprojection_error_inliers+=reprojection_err;
      
    }


  }

  //housekeeping
  total_reprojection_error /= tracklet_ids.size();
  total_reprojection_error_inliers /= motion_tracklet_inliers.size();

  LOG(INFO) << "Motion model estimation\n inliers - " << motion_tracklet_inliers.size() 
      << "\ntotal repr error - " << total_reprojection_error 
      << "\ninlier repr error - " << total_reprojection_error_inliers
      << "\nsuccessful projections - " << successful_projections << "/" << tracklet_ids.size();

  //calculate outliers
  determineOutlierIds(motion_tracklet_inliers, tracklet_ids, motion_tracklet_outliers);
  CHECK_EQ((motion_tracklet_inliers.size() + motion_tracklet_outliers.size()), tracklet_ids.size());
  LOG(INFO) << "Motion Inliers/total = " << motion_tracklet_inliers.size() << "/" << tracklet_ids.size();


  current_frame->pose_ = pnp_pose;
  LOG(INFO) << current_frame->pose_;
  return true;
}


void Tracking::trackDynamicObjects(Frame::Ptr frame) {

  const cv::Mat& rgb = frame->images_.rgb;
  const cv::Mat& semantic_mask = frame->images_.semantic_mask;
  const cv::Mat& flow = frame->images_.flow;

  cv::Mat viz;
  rgb.copyTo(viz);


  // std::vector<InstanceLabel> instance_labels;
  // for(Feature::Ptr dynamic_feature : frame->dynamic_features_) {
  //   // if(dynamic_feature->inlier) {
  //     instance_labels.push_back(dynamic_feature->instance_label);
  //   // }
  // }
  // CHECK_EQ(instance_labels.size(), frame->dynamic_features_.size());


  // std::sort(instance_labels.begin(), instance_labels.end());
  // instance_labels.erase(std::unique(instance_labels.begin(), instance_labels.end()), instance_labels.end());
  // std::vector<std::vector<int>> object_features(instance_labels.size());
  // std::vector<FeaturePtrs> predicted_labels(instance_labels.size());


  // for(Feature::Ptr dynamic_feature : frame->dynamic_features_) {
  //    // save object label
  //   for (int j = 0; j < instance_labels.size(); ++j)
  //   {
  //     if (dynamic_feature->instance_label == instance_labels[j])
  //     {
  //       predicted_labels[j].push_back(dynamic_feature);
  //       break;
  //     }
  //   }
  // }
  // std::vector<FeaturePtrs> object_ids;
  //ensure objects are not on the boundary
  static constexpr float kRowBoundary = 25;
  static constexpr float kColBoundary = 50;
  //at least half of the object is visible?
  static constexpr double KCountThreshold = 0.5;
  for(auto instance_feature_pair : frame->semantic_instance_map_) {
    const int instance_label = instance_feature_pair.first;
    FeaturePtrs features = instance_feature_pair.second;
    int count = 0;
    int dynamic_tracklet_count = 0;
    int static_tracklet_count = 0;
    int sf_count = 0;
    int previous_object_id = -1;
    std::vector<int> sf_range(10, 0);
    FeaturePtrs dynamic_features_after_thresh;
    for(Feature::Ptr feature : features) {
    // for(size_t j = 0; j < predicted_labels[i].size(); j++ ) {
      Feature::Ptr current_feature = feature;
      const KeypointCV& kp = current_feature->keypoint;
      const float u = kp.pt.x;
      const float v = kp.pt.y;


      //we could not track from the previous one?
      Feature::Ptr previous_feature = previous_frame_->getDynamicByTrackletId(current_feature->tracklet_id);
      //all the features should have the same instance label after propogation from udpateMask...
      //TODO: dont just use the last object id...

      if(v > kRowBoundary || v < (rgb.rows - kRowBoundary) || u > kColBoundary || u < (rgb.cols - kColBoundary)) {
        count++;

        if(previous_feature) {
          previous_object_id = previous_feature->object_id;

          //we could track however the previous feature may have been marked as static from optical/scene flow?
          if(previous_feature->type == Feature::Type::DYNAMIC) dynamic_tracklet_count++;
          if(previous_feature->type == Feature::Type::STATIC) static_tracklet_count++;

          //at this point frame MUST have an pose estimate from the frontend
          //is it to calculate the sceneflow
          Landmark lmk_previous, lmk_current;
          camera.backProject(previous_feature->keypoint, previous_feature->depth, &lmk_previous);
          camera.backProject(current_feature->keypoint, current_feature->depth, &lmk_current);

          gtsam::Pose3 previous_pose = previous_frame_->pose_;
          gtsam::Pose3 current_pose = frame->pose_;
          //convert to world frame
          lmk_previous = previous_pose.transformFrom(lmk_previous);
          lmk_current = current_pose.transformFrom(lmk_current);

          Landmark flow_world = lmk_current - lmk_previous;
          gtsam::Pose3 flow_world_transform(gtsam::Rot3::identity(), flow_world);
          //this is in the world frame
          //we conver to camera frame which will be the flow from the previous point to the current point
          // //^c_{t-1}F_t = ^cX_{t-1} * ^w_{t-1}F_t * ^cX_{t-1}^{-1}
          gtsam::Pose3 flow_prev_curr_transform = previous_pose.inverse() * flow_world_transform * previous_pose;
          Landmark flow_prev_curr = flow_prev_curr_transform.translation();
          feature->scene_flow = flow_prev_curr;

          double scene_flow_norm = flow_prev_curr.norm();

          if(scene_flow_norm > params.scene_flow_magnitude) {
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
    //most parts of this object are on the image boundary
    double object_count = count/(int)features.size();
    LOG(INFO) <<  " object count " << object_count;
    if(object_count < KCountThreshold ) {
      for(Feature::Ptr feature : features) {
        feature->object_id = -1;
      }
      //then continue?
      // LOG(INFO) << 
      continue;
    }

    if(features.size() < 50) {
      // LOG(INFO) << "Object " << instance_label << " did not have enough points";
      continue;
    }

    double average_dynamic_tracks = dynamic_tracklet_count/count; //average of how many points 
    double average_static_tracks = static_tracklet_count/count;
    std::cout << "scene flow distribution:"  << std::endl;
    for (int j = 0; j < sf_range.size(); ++j)
        std::cout << sf_range[j] << " ";
    std::cout << std::endl;

    //if not enough of the object is dynamic label all features as static
    double average_flow_count = (double)sf_count/(double)count;
    LOG(INFO) << "Num points that are dynamic " << average_flow_count << "/" << params.scene_flow_percentage;
    if(average_flow_count > params.scene_flow_percentage) {
      // for(Feature::Ptr feature : dynamic_features_after_thresh) {
      //     utils::DrawCircleInPlace(viz, feature->keypoint.pt, Display::getObjectColour(instance_label));

      // }
      int center_x = 0, center_y = 0;
      for(Feature::Ptr feature : features) {
        utils::DrawCircleInPlace(viz, feature->keypoint.pt, Display::getObjectColour(instance_label));
        feature->object_id = instance_label;
        center_x += feature->keypoint.pt.x;
        center_y += feature->keypoint.pt.y;
      }
      center_x/=features.size();
      center_y/=features.size();

      cv::putText(viz, //target image
            std::to_string(instance_label), //text
            cv::Point(center_x, center_y), //top-left position
            cv::FONT_HERSHEY_DUPLEX,
            1.0,
            CV_RGB(118, 185, 0), //font color
            3);
    }

    LOG(INFO) << "Object  " << instance_label << " was tracked - " << dynamic_tracklet_count << "/" << count;
  }

  cv::imshow("Scene flow", viz);
  cv::waitKey(1);

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
  //   //               cv::Point2i(feature.keypoint.pt.x - 10, feature.keypoint.pt.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.3,
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