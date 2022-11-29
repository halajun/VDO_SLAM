#include "Optimizer.h"
#include "FrontendOutput.h"

// #include "dependencies/g2o/g2o/types/vertex_se3.h"
// #include "dependencies/g2o/g2o/types/vertex_pointxyz.h"
// #include "dependencies/g2o/g2o/types/edge_se3.h"
// #include "dependencies/g2o/g2o/types/edge_se3_pointxyz.h"
// #include "dependencies/g2o/g2o/types/edge_se3_prior.h"
// #include "dependencies/g2o/g2o/core/block_solver.h"
// #include "dependencies/g2o/g2o/core/optimization_algorithm_levenberg.h"
// #include "dependencies/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
// #include "dependencies/g2o/g2o/solvers/linear_solver_csparse.h"
// #include "dependencies/g2o/g2o/solvers/linear_solver_eigen.h"
// #include "dependencies/g2o/g2o/solvers/linear_solver_dense.h"

namespace vdo
{
IncrementalOptimizer::IncrementalOptimizer(const BackendParams& params, const Camera& camera)
  : FactorGraphManager(params), camera_(camera)
{
  params_.print();
}

BackendOutput::Ptr IncrementalOptimizer::process(const FrontendOutput& input)
{
  BackendOutput::Ptr output = nullptr;
  if (state == State::kBoostrap)
  {
    output = processBoostrap(input);
  }
  else if (state == State::kNominal)
  {
    output = processNominal(input);
  }

  return output;
}

BackendOutput::Ptr IncrementalOptimizer::processBoostrap(const FrontendOutput& input)
{
  const Frame& frame = *input.frame;
  const gtsam::Pose3& estimated_pose = frame.pose;
  // first pose so we add pose prior
  addCameraPose(state_key_, estimated_pose);
  addCameraPosePrior(state_key_, estimated_pose);

  // oodometry

  state_key_++;
  state = State::kNominal;
  return nullptr;
}

BackendOutput::Ptr IncrementalOptimizer::processNominal(const FrontendOutput& input)
{
  state_key_++;
  return nullptr;
}

// landmarks in camera frame
void IncrementalOptimizer::constructLandmarks(const Features& features, Landmarks& landmarks) const
{
  for (const Feature& feature : features)
  {
    if (feature.inlier && feature.age > 2)
    {
      // probably need to do heaps of bookkeepng here ...
      Landmark lmk_c;
      camera_.backProject(feature.keypoint, feature.depth, &lmk_c);
      landmarks.push_back(lmk_c);
    }
  }
}

}  // namespace vdo