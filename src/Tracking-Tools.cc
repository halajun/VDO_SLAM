#include "Tracking-Tools.h"

#include "Macros.h"
#include "Types.h"
#include "Frame.h"
#include "Camera.h"

#include <algorithm>  // std::set_difference, std::sort
#include <vector>     // std::vector
#include <glog/logging.h>


namespace vdo {


gtsam::Vector3 obtainFlowDepth(const Frame& frame, TrackletId tracklet_id) {
    Feature::Ptr feature = frame.getByTrackletId(tracklet_id);
    CHECK_NOTNULL(feature);
    return gtsam::Vector3(feature->optical_flow.x, feature->optical_flow.y, feature->depth);
}


void determineOutlierIds(const TrackletIds& inliers, const TrackletIds& tracklets, TrackletIds& outliers)
{
  VLOG_IF(1, inliers.size() > tracklets.size())
      << "Usage warning: inlier size (" << inliers.size() << ") > tracklets size (" << tracklets.size()
      << "). Are you parsing inliers as tracklets incorrectly?";
  outliers.clear();
  TrackletIds inliers_sorted(inliers.size()), tracklets_sorted(tracklets.size());
  std::copy(inliers.begin(), inliers.end(), inliers_sorted.begin());
  std::copy(tracklets.begin(), tracklets.end(), tracklets_sorted.begin());

  std::sort(inliers_sorted.begin(), inliers_sorted.end());
  std::sort(tracklets_sorted.begin(), tracklets_sorted.end());

  // full set A (tracklets) must be first and inliers MUST be a subset of A for the set_difference function to work
  std::set_difference(tracklets_sorted.begin(), tracklets_sorted.end(), inliers_sorted.begin(), inliers_sorted.end(),
                      std::inserter(outliers, outliers.begin()));
}

void updateMotionModel(const Frame::Ptr& previous_frame, Frame::Ptr current_frame) {
    //T_wc at t-1
    const gtsam::Pose3& X_previous = previous_frame->pose_;
    //T_wc at t
    const gtsam::Pose3& X_current = current_frame->pose_;
    gtsam::Pose3 relative_motion = X_previous.inverse() * X_current;
    //motion is from the camera pose at t-1 -> pose at t
    current_frame->motion_model_ = relative_motion;
}


//for objects!!
void calculateSceneFlow(const Frame::Ptr& previous_frame, Frame::Ptr current_frame, const Camera& camera) {
    VLOG(2) << "Calculating scene flow";
    
    //shoudl check that all scene flows have been updates
    //the order matters here as this is how we do the initial association 
    for(Feature::Ptr feature : current_frame->dynamic_features_) {
        CHECK(feature->instance_label != Feature::background);
        CHECK(feature->depth != -1);

        Landmark lmk_current, lmk_previous;
    }

}


}