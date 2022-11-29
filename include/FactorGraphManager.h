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
using gtsam::symbol_shorthand::H;  // Pose3 object motion (x,y,z,r,p,y)
using gtsam::symbol_shorthand::L;  // Point3 static Landmark (x,y,z) in world frame
using gtsam::symbol_shorthand::X;  // Pose3 camera pose in world frame (x,y,z,r,p,y)

class FactorGraphManager
{
public:
  VDO_POINTER_TYPEDEFS(FactorGraphManager);

  FactorGraphManager(const BackendParams& params);

  gtsam::Key addStaticLandmark(const size_t tracklet_id, const gtsam::Key& pose_key, const gtsam::Point3 lmk_w);
  gtsam::Key addCameraPose(const gtsam::Key frame, const gtsam::Pose3& pose);
  gtsam::Key addCameraPosePrior(const gtsam::Key frame, const gtsam::Pose3& pose);
  gtsam::Key addBetweenFactor(const gtsam::Key from_frame, const gtsam::Key to_frame, const gtsam::Pose3& odometry);

  // checks if the key is already in the isam2 system OR is in the new_values_ object and hence is about to be added
  bool isKeyInGraph(const gtsam::Key key) const;

  bool optimize();

protected:
  bool updateSmoother(const gtsam::NonlinearFactorGraph& new_factors = gtsam::NonlinearFactorGraph(),
                      const gtsam::Values& new_values = gtsam::Values());

protected:
  const BackendParams params_;
  gtsam::Key state_key_{ 0 };  // probably better to call this frame id or better, just use the frame ID from the
                               // frontend

  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values new_values_;  // better name might be new values?

  gtsam::Values state_;

  std::unique_ptr<gtsam::ISAM2> smoother_;
  gtsam::ISAM2Result smoother_result_;

  static constexpr unsigned char kSymbolCameraPose3Key = 'X';    // camera pose in world frame
  static constexpr unsigned char kSymbolStaticPoint3Key = 'm';   // static landmark in world frame
  static constexpr unsigned char kSymbolMotion3Key = 'H';        // object motion (pose 3) in camera frame
  static constexpr unsigned char kSymbolDynamicPoint3Key = 'l';  // dynamic landmark in... camera? frame

  // TODO: clean up names
  gtsam::noiseModel::Diagonal::shared_ptr cameraPosePrior;
  gtsam::noiseModel::Base::shared_ptr odometryNoiseModel;
  gtsam::noiseModel::Base::shared_ptr point3DNoiseModel;
  gtsam::noiseModel::Base::shared_ptr objectMotionNoiseModel;
  gtsam::noiseModel::Base::shared_ptr objectMotionSmootherNoiseModel;
  gtsam::noiseModel::Base::shared_ptr dynamicPoint3DNoiseModel;

private:
  void setupNoiseModels(const BackendParams& params);
};

}  // namespace vdo