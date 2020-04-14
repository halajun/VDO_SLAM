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


#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

// eigen
#include <Eigen/Core>

#include"Map.h"
#include"Frame.h"
#include"ORBextractor.h"
#include "System.h"

#include <mutex>

namespace ORB_SLAM2
{

using namespace std;

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;

class Tracking
{

    struct BirdEyeVizProperties {
        // Bird-eye visualization
        float birdeye_scale_factor_;
        int birdeye_far_plane_;
        int birdeye_left_plane_;
        int birdeye_right_plane_;

        BirdEyeVizProperties() {
            // Bird-eye visualization module params
            birdeye_scale_factor_ = 13.0;
            birdeye_far_plane_ = 50;
            birdeye_left_plane_ = -20;
            birdeye_right_plane_ = 20;
        }
    };

public:
    Tracking(System* pSys, Map* pMap, const string &strSettingPath, const int sensor);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB, cv::Mat &imD, const cv::Mat &imFlow, const cv::Mat &maskSEM,
                          const cv::Mat &mTcw_gt, const vector<vector<float> > &vObjPose_gt, const double &timestamp,
                          cv::Mat &imTraj, const int &nImage);

    // Sparse Scene Flow Vector
    void GetSceneFlowObj();

    // Delaunay neighbor
    cv::Mat Delaunay(const std::vector<int> &id_dynamic);

    // For flow display on 2d plane
    void DrawLine(cv::KeyPoint &keys, cv::Point2f &flow, cv::Mat &ref_image, const cv::Scalar &color,
                  int thickness=2, int line_type=1, const cv::Point2i &offset=cv::Point2i(0,0));
    void DrawTransparentSquare(cv::Point center, cv::Vec3b color, int radius, double alpha, cv::Mat &ref_image);
    void DrawGridBirdeye(double res_x, double res_z, const BirdEyeVizProperties &viz_props, cv::Mat &ref_image);
    void DrawSparseFlowBirdeye(
        const std::vector<Eigen::Vector3d> &pts, const std::vector<Eigen::Vector3d> &vel,
        const cv::Mat &camera, const BirdEyeVizProperties &viz_props, cv::Mat &ref_image);
    void TransformPointToScaledFrustum(double &pose_x, double &pose_z, const BirdEyeVizProperties &viz_props);

    cv::Mat ObjPoseParsingKT(const std::vector<float> &vObjPose_gt);
    cv::Mat ObjPoseParsingOX(const std::vector<float> &vObjPose_gt);

    cv::Mat InvMatrix(const cv::Mat &T);

    cv::Mat RanSacHorn(const vector<int> &TemperalMatch, const vector<int> &ObjId);

    cv::Mat Find3DAffineTransform(const vector<int> &TemperalMatch, const vector<int> &ObjId);

    cv::Mat GetInitModelCam(const std::vector<int> &MatchId, std::vector<int> &MatchId_sub);
    cv::Mat GetInitModelObj(const std::vector<int> &ObjId, std::vector<int> &ObjId_sub, const int objid);

    void StackObjInfo(std::vector<cv::KeyPoint> &FeatDynObj, std::vector<float> &DepDynObj,
                      std::vector<int> &FeatLabObj);
    std::vector<std::vector<std::pair<int, int> > > GetStaticTrack();
    std::vector<std::vector<std::pair<int, int> > > GetDynamicTrack();
    std::vector<std::vector<std::pair<int, int> > > GetDynamicTrackNew();
    std::vector<std::vector<int> > GetObjTrackTime(std::vector<std::vector<int> > &ObjTrackLab, std::vector<std::vector<int> > &ObjSemanticLab,
                                                   std::vector<std::vector<int> > &vnSMLabGT);

    void GetMetricError(const std::vector<cv::Mat> &CamPose, const std::vector<std::vector<cv::Mat> > &RigMot, const std::vector<std::vector<cv::Mat> > &ObjPosePre,
                        const std::vector<cv::Mat> &CamPose_gt, const std::vector<std::vector<cv::Mat> > &RigMot_gt,
                        const std::vector<std::vector<bool> > &ObjStat);
    void GetVelocityError(const std::vector<std::vector<cv::Mat> > &RigMot, const std::vector<std::vector<cv::Mat> > &PointDyn,
                          const std::vector<std::vector<int> > &FeaLab, const std::vector<std::vector<int> > &RMLab,
                          const std::vector<std::vector<float> > &Velo_gt, const std::vector<std::vector<int> > &TmpMatch,
                          const std::vector<std::vector<bool> > &ObjStat);

    void RenewFrameInfo(const std::vector<int> &TM_sta);

    void UpdateMask();


public:

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;

    // Current Frame
    Frame mCurrentFrame;
    cv::Mat mImGray;

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    cv::Mat mImGrayLast;  // ++++++ new added

    // new added (Nov 14 2019)
    cv::Mat mFlowMap, mFlowMapLast;
    cv::Mat mDepthMap;
    cv::Mat mSegMap, mSegMapLast;

    // transfer the ground truth to use identity matrix as origin
    cv::Mat mOriginInv;

    // Store temperal matching feature index
    bool bFrame2Frame,bFirstFrame,bSecondFrame,bUseGT;  // ++++++ new added
    std::vector<int> TemperalMatch;  // ++++++ new added
    std::vector<int> TemperalMatch_subset;
    std::vector<cv::KeyPoint> mvKeysLastFrame;  // ++++++ new added
    std::vector<cv::KeyPoint> mvKeysCurrentFrame;  // ++++++ new added

    std::vector<cv::KeyPoint> mvTmpObjKeys;
    std::vector<float> mvTmpObjDepth;
    std::vector<int> mvTmpSemObjLabel;
    std::vector<cv::Point2f> mvTmpObjFlowNext;
    std::vector<cv::KeyPoint> mvTmpObjCorres;

    // re-projection error
    std::vector<float> repro_e;

    // save current frame ID
    int f_id;

    // save the global Tracking ID
    int max_id;

    // save stopframe
    int StopFrame;

    // save local batch decision
    bool bLocalBatch;
    bool bGlobalBatch;
    bool bJoint;

    // save timing values
    std::vector<float> all_timing;

    // dataset selection
    bool oxford;

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    // System
    System* mpSystem;

    //Map
    Map* mpMap;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame Info
    Frame mLastFrame;

    //Motion Model
    cv::Mat mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;
};

} //namespace ORB_SLAM

#endif // TRACKING_H
