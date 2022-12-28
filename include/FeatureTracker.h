#pragma once

#include "ORBextractor.h"
#include "Frontend-Definitions.h"  //for TrackingParams
#include "Camera.h"
#include "Frame.h"
#include "Macros.h"

#include <opencv2/features2d.hpp>

namespace vdo
{
class FeatureTracker
{
public:
  VDO_POINTER_TYPEDEFS(FeatureTracker);

  FeatureTracker(const TrackingParams& tracking_params, const Camera& camera);
  Frame::Ptr track(const InputPacket& input_packet, size_t& n_optical_flow, size_t& n_new_tracks);

private:
  // feature will be none if checks fail (ie. the point is an object, depth fails etc)
  Feature::Ptr constructStaticFeature(const ImagePacket& images, const cv::KeyPoint& kp, size_t age, size_t tracklet_id,
                                      size_t frame_id) const;
  bool posInGrid(const cv::KeyPoint& kp, int& pos_x, int& pos_y) const;
  void computeImageBounds(const cv::Size& size, int& min_x, int& max_x, int& min_y, int& max_y) const;

private:
  TrackingParams tracking_params_;
  Camera camera_;
  ORBextractor::UniquePtr feature_detector_;
  Frame::Ptr previous_frame_{ nullptr };

  size_t tracklet_count = 0;

  static constexpr int FRAME_GRID_ROWS = 48;
  static constexpr int FRAME_GRID_COLS = 64;

  bool initial_computation_{ true };

  int min_x_;
  int min_y_;
  int max_x_;
  int max_y_;
  cv::Size img_size_;  // set on first computation

  // grid of trackled Id's
  double grid_elements_width_inv_;
  double grid_elements_height_inv_;
};

}  // namespace vdo