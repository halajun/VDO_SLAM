#include "Tracking.h"
#include "ORBextractor.h"
#include "Camera.h"
#include "Frontend-Definitions.h"
#include "UtilsGtsam.h"

#include <opencv2/opencv.hpp>
#include <glog/logging.h>
#include <algorithm>

namespace vdo
{
Tracking::Tracking(const TrackingParams& params_, const Camera& camera_) : params(params_), camera(camera_)
{
  feature_detector =
      vdo::make_unique<ORBextractor>(params.n_features, static_cast<float>(params.scale_factor), params.n_levels,
                                     params.init_threshold_fast, params.min_threshold_fast);
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

FrontendOutput::Ptr Tracking::processBoostrap(const InputPacket& input,
                                              GroundTruthInputPacket::ConstOptional ground_truth)
{
  const Timestamp& timestamp = input.timestamp;
  const size_t& frame_id = input.frame_id;
  ImagePacket images;
  preprocessInput(input, images);
  // constrict frame
  Frame::Ptr frame = std::make_shared<Frame>(images, timestamp, frame_id, camera.Params());
  frame->detectFeatures(feature_detector);
  frame->processStaticFeatures(params.depth_background_thresh);
  frame->processDynamicFeatures(params.depth_obj_thresh);
  displayFeatures(*frame);
  frame->projectKeypoints(camera);
  // use ground truth to initalise pose
  if (ground_truth)
  {
    frame->pose = ground_truth->X_wc;
    frame->ground_truth = ground_truth;
    LOG(INFO) << "Initalising pose using ground truth " << frame->pose;
  }
  else
  {
    frame->pose = gtsam::Pose3::identity();
    LOG(INFO) << "Initalising pose identity matrix " << frame->pose;
  }

  // set initalisatiobn
  previous_frame = frame;
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
  // constrict frame
  Frame::Ptr frame = std::make_shared<Frame>(images, timestamp, frame_id, camera.Params());
  //optical flow tracking failed so we need to redetect all features
  if(!staticTrackOpticalFlow(previous_frame, frame)) {
    frame->detectFeatures(feature_detector);
  }
  
  frame->processStaticFeatures(params.depth_background_thresh);

  frame->processDynamicFeatures(params.depth_obj_thresh);
  frame->projectKeypoints(camera);
  displayFeatures(*frame);

  //estimate initialPos
  solveInitalCamModel(previous_frame, frame);

  if (ground_truth)
  {
    frame->ground_truth = ground_truth;
    LOG(INFO) << "Initalising pose using ground truth " << frame->ground_truth->X_wc;
  }

  previous_frame = frame;
  return std::make_shared<FrontendOutput>(frame);
}


bool Tracking::staticTrackOpticalFlow(const Frame::Ptr& previous_frame_, Frame::Ptr current_frame_) {
  //TODO: mark feature as inlier/outlier
  const TrackletIdFeatureMap& previous_features = previous_frame_->static_features;
  Observations tracked_observation;
  
  for(const auto& feature_pair : previous_features) {
    const std::size_t& tracklet_id = feature_pair.first;
    const Feature& previous_feature = *feature_pair.second;
    KeypointCV kp = previous_feature.predicted_keypoint;

    //AND is an inlier

    if(camera.isKeypointContained(kp, previous_feature.depth) && previous_feature.inlier) {
      Observation obs;
      obs.keypoint = kp;
      obs.type = Observation::Type::OPTICAL_FLOW;
      obs.tracklet_id = previous_feature.tracklet_id;
      tracked_observation.push_back(obs);

    }
  }

  //TODO: a much more sophisticaed retracking method
  if(tracked_observation.size() < 30) {
    current_frame_->addStaticFeatures(tracked_observation);
    return false;
  }
  else {
      current_frame_->addStaticFeatures(tracked_observation);
      return true;

  }


}

bool Tracking::solveInitalCamModel(Frame::Ptr previous_frame_, Frame::Ptr current_frame_) {
  std::vector<cv::Point2f> current_2d;
  std::vector<cv::Point3f> previous_3d;
  std::vector<size_t> tracklet_ids; //feature tracklets from the current frame so we can assign values as inliers or outliers
  for(const auto& feature_pair : current_frame_->static_features) {
    const std::size_t& tracklet_id = feature_pair.first;

    Feature::Ptr previous_feature = previous_frame_->getStaticFeature(tracklet_id);
    if(!previous_feature) {continue;}

    Feature::Ptr current_feature = feature_pair.second;

    //sanity check
    CHECK_EQ(previous_feature->tracklet_id, current_feature->tracklet_id);
    CHECK_EQ(previous_feature->tracklet_id, tracklet_id);

    // //dont think this shoudl happen as we only track (old) points if it is an inlier! see staticTrackOpticalFlow
    if(!previous_feature->inlier) {continue;}

    current_2d.push_back(current_feature->keypoint.pt);

    Landmark lmk;
    camera.backProject(previous_feature->keypoint, previous_feature->depth, &lmk);
    //lmk is in camera frame. Put into world frame
    Landmark lmk_world = previous_frame->pose.transformFrom(lmk);
    cv::Point3f lmk_f(static_cast<float>(lmk_world.x()), static_cast<float>(lmk_world.y()), static_cast<float>(lmk_world.z()));
    previous_3d.push_back(lmk_f);
    tracklet_ids.push_back(tracklet_id);
  }

  //TODO: if we have zero tracket features -> this means need to fix the way the frontend takes feature points so we never 
  //completely clear our tracks
  if(previous_3d.size() < 5) {
    LOG(WARNING) << "Cannot run PnP with < 5 points";
    current_frame_->pose = previous_frame->pose;
    return false;
  }

  LOG(INFO) << "Solving initial camera model with  " << previous_3d.size() << " features";

  const cv::Mat& K = camera.Params().K;
  const cv::Mat& D = camera.Params().D;

  // output
  cv::Mat Rvec(3, 1, CV_64FC1);
  cv::Mat Tvec(3, 1, CV_64FC1);
  cv::Mat Rot(3, 3, CV_64FC1);
  cv::Mat pnp_inliers; //a [1 x N] vector

  // solve
  int iter_num = 500;
  double reprojectionError = 0.4, confidence = 0.98;  // 0.5 0.3
  cv::solvePnPRansac(previous_3d, current_2d, K, D, Rvec, Tvec, false, iter_num, reprojectionError, confidence,
                     pnp_inliers, cv::SOLVEPNP_AP3P);  // AP3P EPNP P3P ITERATIVE DLS
  // LOG(INFO) << "inliers " << pnp_inliers;

  cv::Rodrigues(Rvec, Rot);
  //vector of tracklet id's that PnP ransac marked as inliers
  std::vector<size_t>  pnp_tracklet_inliers;
  for(size_t i = 0; i < pnp_inliers.rows; i++) {
    int inlier_index = pnp_inliers.at<int>(i);
    size_t inlier_tracklet = tracklet_ids[inlier_index];
    pnp_tracklet_inliers.push_back(inlier_tracklet);
  }

  LOG(INFO) << "Inliers/total = " << pnp_tracklet_inliers.size() << "/" << previous_3d.size();

  //sanity check
  CHECK_EQ(pnp_tracklet_inliers.size(), pnp_inliers.rows);
  //mark all features in current frame as either inlier or outlier
  for(const auto& feature_pair : current_frame_->static_features) {
    //TODO: can massively optimize
    if (std::find(pnp_tracklet_inliers.begin(), pnp_tracklet_inliers.end(), feature_pair.first) != pnp_tracklet_inliers.end()) {
      //it is an inlier
      feature_pair.second->inlier = true;
    }
    else {
      feature_pair.second->inlier = false;
    }
  }


  current_frame_->pose = utils::cvMatsToGtsamPose3(Rot, Tvec).inverse();
  LOG(INFO) << current_frame_->pose;
  return true;

}



// Frame::Ptr Tracking::constructFrame(const ImagePacket& images, Timestamp timestamp, size_t frame_id)
// {
//     Frame::Ptr frame = std::make_shared<Frame>(images, timestamp, frame_id, camera.Params());
//     detectFeatures(frame);
//     displayFeatures(*frame);
//     frame->projectKeypoints(camera);
//     return frame;
// }

void Tracking::preprocessInput(const InputPacket& input, ImagePacket& images)
{
  input.images.rgb.copyTo(images.rgb);
  input.images.flow.copyTo(images.flow);
  input.images.semantic_mask.copyTo(images.semantic_mask);

  input.images.depth.copyTo(images.depth);
  processInputDepth(input.images.depth, images.depth);
}

void Tracking::detectFeatures(Frame::Ptr frame)
{
  frame->detectFeatures(feature_detector);
  frame->processStaticFeatures(params.depth_background_thresh);
  frame->processDynamicFeatures(params.depth_obj_thresh);
}

void Tracking::initaliseFrameTo3D(Frame::Ptr frame)
{
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
  const ImagePacket& images = frame.Images();
  images.rgb.copyTo(disp);

  frame.drawStaticFeatures(disp);
  frame.drawDynamicFeatures(disp);

  cv::imshow("Frames", disp);
  cv::waitKey(1);
}

}  // namespace vdo