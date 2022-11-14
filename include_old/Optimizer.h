/**
 * This file is part of VDO-SLAM.
 *
 * Copyright (C) 2019-2020 Jun Zhang <jun doc zhang2 at anu dot edu doc au> (The Australian National University)
 * For more information see <https://github.com/halajun/VDO_SLAM>
 *
 **/

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <opencv2/core/core.hpp>

// #include "Map.h"
// #include "frontend/Frame.h"
// #include "dependencies/g2o/g2o/types/types_six_dof_expmap.h"

namespace VDO_SLAM
{
class Frame;
class Map;

using namespace std;

class Optimizer
{
public:
  int static PoseOptimizationNew(Frame* pCurFrame, Frame* pLastFrame, vector<int>& TemperalMatch);
  int static PoseOptimizationFlow2Cam(Frame* pCurFrame, Frame* pLastFrame, vector<int>& TemperalMatch);
  cv::Mat static PoseOptimizationObjMot(Frame* pCurFrame, Frame* pLastFrame, const vector<int>& ObjId,
                                        std::vector<int>& InlierID);
  cv::Mat static PoseOptimizationFlow2(Frame* pCurFrame, Frame* pLastFrame, const vector<int>& ObjId,
                                       std::vector<int>& InlierID);
  void static FullBatchOptimization(Map* pMap, const cv::Mat Calib_K);
  void static PartialBatchOptimization(Map* pMap, const cv::Mat Calib_K, const int WINDOW_SIZE);
  void static PartialBatchOptimizationGTSAM(Map* pMap, const cv::Mat Calib_K, const int WINDOW_SIZE);
  cv::Mat static Get3DinWorld(const cv::KeyPoint& Feats2d, const float& Dpts, const cv::Mat& Calib_K,
                              const cv::Mat& CameraPose);
  cv::Mat static Get3DinCamera(const cv::KeyPoint& Feats2d, const float& Dpts, const cv::Mat& Calib_K);
};

}  // namespace VDO_SLAM

#endif  // OPTIMIZER_H
