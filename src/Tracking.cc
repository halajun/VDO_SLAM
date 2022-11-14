#include "Tracking.h"
#include "ORBextractor.h"
#include "Camera.h"

#include <opencv2/opencv.hpp>
#include <glog/logging.h>

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
  // preprocess depth image
  preprocessInput(input);
  // constrict frame
  Frame::Ptr frame = std::make_shared<Frame>(images, current_timestamp, current_frame_id, camera.Params());
  detectFeatures(frame);
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
  current_frame = frame;
  state = State::kNominal;
  return std::make_shared<FrontendOutput>(frame);
}

FrontendOutput::Ptr Tracking::processNominal(const InputPacket& input,
                                             GroundTruthInputPacket::ConstOptional ground_truth)
{
  preprocessInput(input);
  // constrict frame
  Frame::Ptr frame = std::make_shared<Frame>(images, current_timestamp, current_frame_id, camera.Params());
  detectFeatures(frame);
  displayFeatures(*frame);
  frame->projectKeypoints(camera);
  return std::make_shared<FrontendOutput>(frame);
}

void Tracking::preprocessInput(const InputPacket& input)
{
  current_frame_id = input.frame_id;
  current_timestamp = input.timestamp;

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