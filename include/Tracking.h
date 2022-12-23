#pragma once

#include "Types.h"
#include "Macros.h"
#include "Camera.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "FeatureTracker.h"

#include "FrontendOutput.h"
#include "Frontend-Definitions.h"
#include "Backend-Definitions.h"

#include "utils/Logger.h"

namespace vdo
{
class Tracking
{
public:
  VDO_POINTER_TYPEDEFS(Tracking);

  Tracking(const TrackingParams& params_, const Camera& camera_);

  FrontendOutput::Ptr process(const InputPacket& input,
                              GroundTruthInputPacket::ConstOptional ground_truth = boost::none);
  // for now just pose
  void updateFromBackend(const BackendOutput& backend_output);

private:
  FrontendOutput::Ptr processBoostrap(const InputPacket& input,
                                      GroundTruthInputPacket::ConstOptional ground_truth = boost::none);

  FrontendOutput::Ptr processNominal(const InputPacket& input,
                                     GroundTruthInputPacket::ConstOptional ground_truth = boost::none);

  // using the static features from the previous frame and current frame, calculates and sets the pose
  // of the current frame. Features are marked as outliers based on PnP ransac and will not be included in the next
  // frame
  bool solveInitalCamModel(Frame::Ptr previous_frame_, Frame::Ptr current_frame_);

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

  FeatureTracker::UniquePtr feature_tracker;

  // state data
  State state{ State::kBoostrap };
  // // the frame ID of the current input
  // size_t current_frame_id;
  // Timestamp current_timestamp;
  // // images
  // ImagePacket images;  // the rgb image should be rgb

  Frame::Ptr previous_frame{ nullptr };

  Logger<Frame> frame_logger{ "frame.csv" };
};

}  // namespace vdo