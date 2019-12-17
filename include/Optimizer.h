/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{

class LoopClosing;

class Optimizer
{
public:
    void static BundleAdjustment(const std::vector<KeyFrame*> &vpKF, const std::vector<MapPoint*> &vpMP,
                                 int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0,
                                 const bool bRobust = true);
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL,
                                       const unsigned long nLoopKF=0, const bool bRobust = true);
    void static LocalBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, Map *pMap);
    int static PoseOptimization(Frame* pFrame);
    int static PoseOptimizationNew(Frame *pCurFrame, Frame *pLastFrame, vector<int> &TemperalMatch);
    int static PoseOptimizationFlow2Cam(Frame *pCurFrame, Frame *pLastFrame, vector<int> &TemperalMatch, const vector<Eigen::Vector2d> &flo_gt, const vector<double> &e_bef);
    cv::Mat static PoseOptimizationObj(Frame *pCurFrame, Frame *pLastFrame, const vector<int> &TemperalMatch, const vector<int> &ObjId, float &repro_e);
    cv::Mat static PoseOptimizationObjTest(Frame *pCurFrame, Frame *pLastFrame, const vector<int> &ObjId);
    cv::Mat static PoseOptimizationObjMot(Frame *pCurFrame, Frame *pLastFrame, const vector<int> &ObjId, const cv::Point2f flo_co);
    cv::Mat static PoseOptimizationObjMotTLS(Frame *pCurFrame, Frame *pLastFrame, const vector<int> &ObjId);
    cv::Mat static PoseOptimizationForBack(Frame *pCurFrame, Frame *pLastFrame, const vector<int> &ObjId);
    cv::Mat static PoseOptimizationFlowDepth(Frame *pCurFrame, Frame *pLastFrame, const vector<int> &ObjId);
    cv::Mat static PoseOptimizationFlowDepth2(Frame *pCurFrame, Frame *pLastFrame, const vector<int> &ObjId);
    cv::Mat static PoseOptimizationFlowDepth3(Frame *pCurFrame, Frame *pLastFrame, const vector<int> &ObjId);
    cv::Mat static PoseOptimizationFlow(Frame *pCurFrame, Frame *pLastFrame, const vector<int> &ObjId);
    cv::Mat static PoseOptimizationFlow2(Frame *pCurFrame, Frame *pLastFrame, const vector<int> &ObjId, const vector<Eigen::Vector2d> &flo_gt, const vector<double> &e_bef, std::vector<int> &InlierID);
    cv::Mat static PoseOptimizationFlow2RanSac(Frame *pCurFrame, Frame *pLastFrame, const vector<int> &ObjId, const vector<Eigen::Vector2d> &flo_gt, const vector<double> &e_bef);
    cv::Mat static PoseOptimizationDepth(Frame *pCurFrame, Frame *pLastFrame, const vector<int> &ObjId);
    void static FullBatchOptimization(Map* pMap, const cv::Mat Calib_K);
    void static PartialBatchOptimization(Map* pMap, const cv::Mat Calib_K, const int WINDOW_SIZE);
    cv::Mat static Get3DinWorld(const cv::KeyPoint &Feats2d, const float &Dpts, const cv::Mat &Calib_K, const cv::Mat &CameraPose);
    cv::Mat static Get3DinCamera(const cv::KeyPoint &Feats2d, const float &Dpts, const cv::Mat &Calib_K);

    // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
    void static OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
                                       const bool &bFixScale);

    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
    static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2, const bool bFixScale);
};

} //namespace ORB_SLAM

#endif // OPTIMIZER_H
