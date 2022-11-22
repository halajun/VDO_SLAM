#pragma once

#include "Macros.h"
#include "Camera.h"
#include "ORBextractor.h"
#include "Frame.h"

#include "FrontendOutput.h"

namespace vdo
{
struct TrackingParams
{
  // tracking points params
  size_t max_tracking_points_bg;
  size_t max_tracking_points_obj;

  // scene flow thresholds
  double scene_flow_magnitude;
  double scene_flow_percentage;

  // depth thresholds
  double depth_background_thresh;
  double depth_obj_thresh;

  double depth_scale_factor;

  // ORB detector params
  int n_features;
  double scale_factor;
  int n_levels;
  int init_threshold_fast;
  int min_threshold_fast;
};

class Tracking
{
public:
  VDO_POINTER_TYPEDEFS(Tracking);

  enum class State
  {
    kBoostrap,
    kNominal
  };

  Tracking(const TrackingParams& params_, const Camera& camera_);

  FrontendOutput::Ptr process(const InputPacket& input,
                              GroundTruthInputPacket::ConstOptional ground_truth = boost::none);

private:
  FrontendOutput::Ptr processBoostrap(const InputPacket& input,
                                      GroundTruthInputPacket::ConstOptional ground_truth = boost::none);

  FrontendOutput::Ptr processNominal(const InputPacket& input,
                                     GroundTruthInputPacket::ConstOptional ground_truth = boost::none);

  Frame::Ptr constructFrame(const ImagePacket& images, Timestamp timestamp, size_t frame_id);

  //false if failed and so neeeds to detect new fatures
  bool staticTrackOpticalFlow(const Frame::Ptr& previous_frame_, Frame::Ptr current_frame_);

  //find the observations from the previous frame that should be tracked into the next (current) frame
  void calculateOpticalFlowTracks(const Frame::Ptr& previous_frame_, Observations& observations);
  //using the static features from the previous frame and current frame, calculates and sets the pose 
  //of the current frame. Features are marked as outliers based on PnP ransac and will not be included in the next frame
  bool solveInitalCamModel(Frame::Ptr previous_frame_, Frame::Ptr current_frame_);

  
  void detectFeatures(Frame::Ptr frame);
  void initaliseFrameTo3D(Frame::Ptr frame);

  // calculates the input depth map from the disparity map
  // sets state variables
  void preprocessInput(const InputPacket& input, ImagePacket& images);

  // converts the input disparity mape into an stereo depth map using the
  // stereo baseline and depth map factor
  // expects depth to be sized appropiately
  void processInputDepth(const cv::Mat& disparity, cv::Mat& depth);

  // for viz
  void displayFeatures(const Frame& frame);

private:
  TrackingParams params;
  Camera camera;

  ORBextractor::UniquePtr feature_detector;

  // state data
  State state{ State::kBoostrap };
  // // the frame ID of the current input
  // size_t current_frame_id;
  // Timestamp current_timestamp;
  // // images
  // ImagePacket images;  // the rgb image should be rgb

  Frame::Ptr previous_frame{ nullptr };
};

}  // namespace vdo