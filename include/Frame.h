/**
* This file is part of VDO-SLAM.
*
* Copyright (C) 2019-2020 Jun Zhang <jun doc zhang2 at anu dot edu doc au> (The Australian National University)
* For more information see <https://github.com/halajun/VDO_SLAM>
*
**/

#ifndef FRAME_H
#define FRAME_H

#include<vector>

#include "ORBextractor.h"

#include <opencv2/opencv.hpp>

namespace VDO_SLAM
{

using namespace std;

#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class Frame
{
public:
    Frame();

    // Copy constructor.
    Frame(const Frame &frame);

    // Constructor for RGB-D cameras.
    Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const cv::Mat &imFlow, const cv::Mat &maskSEM, const double &timeStamp, ORBextractor* extractor,
          cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, const float &thDepthObj, const int &UseSampleFea);

    // Extract ORB on the image. 0 for left image and 1 for right image.
    void ExtractORB(int flag, const cv::Mat &im);

    // Set the camera pose.
    void SetPose(cv::Mat Tcw);

    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();

    // Returns the camera center.
    inline cv::Mat GetCameraCenter(){
        return mOw.clone();
    }

    // Returns inverse of rotation
    inline cv::Mat GetRotationInverse(){
        return mRwc.clone();
    }

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

    // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
    void ComputeStereoFromRGBD(const cv::Mat &imDepth);

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    cv::Mat UnprojectStereo(const int &i);
    cv::Mat UnprojectStereoStat(const int &i, const bool &addnoise);
    cv::Mat UnprojectStereoObject(const int &i, const bool &addnoise);
    cv::Mat UnprojectStereoObjectCamera(const int &i, const bool &addnoise);
    cv::Mat UnprojectStereoObjectNoise(const int &i, const cv::Point2f of_error);
    cv::Mat ObtainFlowDepthObject(const int &i, const bool &addnoise);
    cv::Mat ObtainFlowDepthCamera(const int &i, const bool &addnoise);

    std::vector<cv::KeyPoint> SampleKeyPoints(const int &rows, const int &cols);

public:

    // Feature extractor. The right is used only in the stereo case.
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;

    // Frame timestamp.
    double mTimeStamp;

    // Calibration matrix and OpenCV distortion parameters.
    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;

    // Stereo baseline multiplied by fx.
    float mbf;

    // Stereo baseline in meters.
    float mb;

    // Threshold close/far points.
    float mThDepth;
    float mThDepthObj;

    // Number of KeyPoints.
    int N;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
    std::vector<cv::KeyPoint> mvKeysUn;

    // Corresponding stereo coordinate and depth for each keypoint.
    // "Monocular" keypoints have a negative value.
    std::vector<float> mvuRight;
    std::vector<float> mvDepth;

    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors, mDescriptorsRight;


    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    // Number of KeyPoints.
    int N_s;

    // Store keypoints and descriptors
    std::vector<cv::KeyPoint> mvStatKeys, mvStatKeysRight;

    // Store dense key points and depths on objects
    std::vector<cv::KeyPoint> mvObjKeys;
    std::vector<float> mvObjDepth;
    std::vector<cv::Mat> mvObj3DPoint;
    // Correspondence for the objects
    std::vector<cv::KeyPoint> mvObjCorres;
    // Optical flow for the objects
    std::vector<cv::Point2f> mvObjFlowGT, mvObjFlowNext;
    // semantic object label of all the foreground features
    std::vector<int> vSemObjLabel;

    // save the object status (false for outlier, true for inlier)  # added 10 Jan 2020 #
    std::vector<bool> bObjStat;



    // depth for each keypoint
    std::vector<float> mvStatDepth;

    // Store the Label Index for each features: -1(outlier/unknown), 0(static), 1...n(object label).
    std::vector<int> vObjLabel;

    // Store the 3D flow vector and the 2D re-projection error vector
    std::vector<cv::Point3f> vFlow_3d;
    std::vector<cv::Point2f> vFlow_2d;

    // Store the motion of objects
    std::vector<cv::Mat> vObjMod;
    std::vector<cv::Mat> vObjPosePre;
    std::vector<cv::Point2f> vSpeed;
    std::vector<int> nModLabel;
    std::vector<int> nSemPosition;
    std::vector<int> vObjBoxID; // bounding box for each object
    std::vector<std::vector<int> > vnObjID; // object id in current frame
    std::vector<std::vector<int> > vnObjInlierID; // object id in current frame
    std::vector<cv::Mat> vObjCentre3D; // 3D in the world coordinate frame
    std::vector<cv::Mat> vObjCentre2D; // 2D in the image plane

    // for initializing motion
    cv::Mat mInitModel;

    std::vector<cv::KeyPoint> mvCorres; // correspondence
    std::vector<cv::Point2f> mvFlow,mvFlowNext; // optical flow
    // std::vector<int> vCorSta; // the status of correspondence, -1 (outliers) 1 (has correspondence)

    // temporal saved
    std::vector<cv::KeyPoint> mvStatKeysTmp;
    std::vector<float> mvStatDepthTmp;
    std::vector<cv::Mat> mvStat3DPointTmp;
    std::vector<int> vSemLabelTmp;
    std::vector<int> vObjLabel_gtTmp;
    int N_s_tmp;

    // inlier ID generated in this frame  (new added Nov 14 2019)
    std::vector<int> nStaInlierID;
    std::vector<int> nDynInlierID;


    // **************** Ground Truth *********************

    std::vector<cv::Mat> vObjPose_gt;
    std::vector<int> nSemPosi_gt;
    std::vector<cv::Mat> vObjMod_gt;
    std::vector<float> vObjSpeed_gt;

    cv::Mat mTcw_gt;
    std::vector<int> vObjLabel_gt; // 0(background), 1...n(instance label)

    // ***************************************************

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    // Flag to identify outlier associations.
    std::vector<bool> mvbOutlier;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Camera pose.
    cv::Mat mTcw;

    // Current and Next Frame id.
    static long unsigned int nNextId;
    long unsigned int mnId;

    // Scale pyramid info.
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    vector<float> mvScaleFactors;
    vector<float> mvInvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;

    // Undistorted Image Bounds (computed once).
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

    static bool mbInitialComputations;


private:

    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    void UndistortKeyPoints();

    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat &imLeft);

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();

    // Rotation, translation and camera center
    cv::Mat mRcw;
    cv::Mat mtcw;
    cv::Mat mRwc;
    cv::Mat mOw; // mtwc
};

}// namespace VDO_SLAM

#endif // FRAME_H
