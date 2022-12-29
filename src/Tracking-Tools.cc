#include "Tracking-Tools.h"

#include "Macros.h"
#include "Types.h"
#include "Frame.h"
#include "Camera.h"

#include <algorithm>  // std::set_difference, std::sort
#include <vector>     // std::vector
#include <glog/logging.h>


namespace vdo {


bool SortPairInt(const std::pair<int, int>& a, const std::pair<int, int>& b)
{
  return (a.second > b.second);
}



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

void updateFrameMask(const Frame::Ptr& previous_frame, Frame::Ptr current_frame) {
  if(!previous_frame) {
    LOG(WARNING) << "Cannot update frame mask as previous frame is null";
    return;
  }
  CHECK(current_frame);

  const cv::Mat& rgb = previous_frame->images_.rgb;
  const cv::Mat& previous_semantic_mask = previous_frame->images_.semantic_mask;
  const cv::Mat& previous_flow = previous_frame->images_.flow;
  VLOG(2) << "Updating mask";
  std::vector<InstanceLabel> instance_labels;
  for(Feature::Ptr dynamic_feature : previous_frame->dynamic_features_) {
    // if(dynamic_feature->inlier) {
      instance_labels.push_back(dynamic_feature->instance_label);
    // }
  }
CHECK_EQ(instance_labels.size(), previous_frame->dynamic_features_.size());


  std::sort(instance_labels.begin(), instance_labels.end());
  instance_labels.erase(std::unique(instance_labels.begin(), instance_labels.end()), instance_labels.end());
  std::vector<std::vector<int>> object_features(instance_labels.size());


  // collect the predicted labels and semantic labels in vector
  for(size_t i = 0; i < instance_labels.size(); i++) {
    Feature::Ptr dynamic_feature = previous_frame->dynamic_features_[i];
    for(size_t j = 0; j < instance_labels.size(); j++) {
      if(dynamic_feature->instance_label == instance_labels[j]) {
        object_features[j].push_back(i);
        break;
      }
    }
  }

   // check each object label distribution in the coming frame
    int updated_mask_points = 0;
  for(size_t i = 0; i < object_features.size(); i++) {
    std::vector<InstanceLabel> temp_label;
    for(size_t j = 0; j < object_features[i].size(); j++) {
        Feature::Ptr feature = previous_frame->dynamic_features_[object_features[i][j]];
      const KeypointCV& predicted_kp = feature->predicted_keypoint;
      const float u = predicted_kp.pt.x;
      const float v = predicted_kp.pt.y;

      //ensure u and v are sitll inside the frame
      if(u < rgb.cols && u > 0 && v < rgb.rows && v > 0) {
        temp_label.push_back(previous_semantic_mask.at<InstanceLabel>(v, u));
      }
    }

    if(temp_label.size() < 100) {
      LOG(WARNING) << "not enoug points to track object " << i;
      //then do we mark as outliers?
      continue;
    }

    // find label that appears most in LabTmp()
    // (1) count duplicates
    std::map<int, int> label_duplicates;
    for (int k : temp_label) { ++label_duplicates[k];}
    // (2) and sort them by descending order
    std::vector<std::pair<int, int>> sorted;
    for (auto k : label_duplicates) {
      sorted.push_back(std::make_pair(k.first, k.second));
    }
    std::sort(sorted.begin(), sorted.end(), SortPairInt);

    //note reference
    cv::Mat& current_semantic_mask = current_frame->images_.flow;

    // recover the missing mask (time consuming!)
    if (sorted[0].first == 0)
    {
      for (int j = 0; j < rgb.rows; j++)
      {
        for (int k = 0; k < rgb.cols; k++)
        {
          if (previous_semantic_mask.at<InstanceLabel>(j, k) == instance_labels[i])
          {
            const int flow_x = previous_flow.at<cv::Vec2f>(j, k)[0];
            const int flow_y = previous_flow.at<cv::Vec2f>(j, k)[1];
            
            if (k + flow_x < rgb.cols && k + flow_x > 0 && j + flow_y < rgb.rows && j + flow_y > 0)
              current_semantic_mask.at<InstanceLabel>(j + flow_y, k + flow_x) = instance_labels[i];
              updated_mask_points++;
          }
        }
      }
    }

  }

  VLOG(2) << "Updated mask poins - " << updated_mask_points;





}


}