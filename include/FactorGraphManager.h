#pragma once

#include "Macros.h"
#include "Backend-Definitions.h"

#include <memory>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>

namespace vdo
{

class FactorGraphManager
{
public:
  VDO_POINTER_TYPEDEFS(FactorGraphManager);


  FactorGraphManager(const BackendParams& params);

  inline gtsam::Key poseKey(const gtsam::Key frame) { 
    return gtsam::Symbol(kSymbolCameraPose3Key, frame);
  }

  inline gtsam::Key staticLandmarkKey(const size_t tracklet_id) {
    return gtsam::Symbol(kSymbolStaticPoint3Key, tracklet_id);
  }

  gtsam::Key addStaticLandmark(const size_t tracklet_id, const gtsam::Key& pose_key, const gtsam::Point3 lmk_c);
  gtsam::Key addCameraPose(const gtsam::Key frame, const gtsam::Pose3& pose);
  gtsam::Key addCameraPosePrior(const gtsam::Key frame, const gtsam::Pose3& pose);
  void addBetweenFactor(const gtsam::Key from_frame, const gtsam::Key to_frame, const gtsam::Pose3& odometry);

  // checks if the key is already in the isam2 system OR is in the new_values_ object and hence is about to be added
  bool isKeyInGraph(const gtsam::Key key) const;


  bool optimize(const gtsam::Key state_key);

protected:



  bool updateSmoother(const gtsam::NonlinearFactorGraph& new_factors = gtsam::NonlinearFactorGraph(),
                      const gtsam::Values& new_values = gtsam::Values());

protected:
  const BackendParams params_;

  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values new_values_;

  gtsam::Values state_;

  std::unique_ptr<gtsam::ISAM2> smoother_;
  gtsam::ISAM2Result smoother_result_;

  static constexpr unsigned char kSymbolCameraPose3Key = 'X';    // camera pose in world frame
  static constexpr unsigned char kSymbolStaticPoint3Key = 'm';   // static landmark in world frame
  static constexpr unsigned char kSymbolMotion3Key = 'H';        // object motion (pose 3) in camera frame
  static constexpr unsigned char kSymbolDynamicPoint3Key = 'l';  // dynamic landmark in... camera? frame

  gtsam::noiseModel::Diagonal::shared_ptr cameraPosePrior_; // prior added to the camera pose at the first frame to fix the graph
  gtsam::noiseModel::Base::shared_ptr odomNoise_; // noise model between consequative poses constructed from the camera motion estimate 
  gtsam::noiseModel::Base::shared_ptr staticLmkNoise_; //noise model on the 3D projection factor
  gtsam::noiseModel::Base::shared_ptr dynamicLmkNoise_; //noise model on the 3D projection factor on dynamic objects
  gtsam::noiseModel::Base::shared_ptr objectMotionNoise_; //noise model used on the motion landmkark ternary factor (H) 
  gtsam::noiseModel::Base::shared_ptr objectMotionSmoothingNoise_; //noise model used to constrain the relative change in Motion (H_{t-1} -> H_t)



private:
  void setupNoiseModels(const BackendParams& params);
};

}  // namespace vdo