#pragma once

#include "Types.h"
#include "Frame.h"
#include "Camera.h"
#include "Frontend-Definitions.h"
#include "Backend-Definitions.h"
#include "FactorGraphManager.h"

namespace vdo
{
class FrontendOutput;

struct PoseOptimizationFlow2Cam
{
  void operator()(Frame::Ptr previous_frame_, Frame::Ptr current_frame_);
};

class IncrementalOptimizer : public FactorGraphManager
{
public:
  VDO_POINTER_TYPEDEFS(IncrementalOptimizer);
  // TODO: params
  IncrementalOptimizer(const BackendParams& params, const Camera& camera);

  BackendOutput::Ptr process(const FrontendOutput& input);

private:
  BackendOutput::Ptr processBoostrap(const FrontendOutput& input);
  BackendOutput::Ptr processNominal(const FrontendOutput& input);

  // TODO:
  // construct landmarks frame
  void constructLandmarks(const Features& features, Landmarks& landmarks) const;
  // validate landmarks
  // add landmarks to graph

private:
  Camera camera_;
  State state{ State::kBoostrap };
};

}  // namespace vdo