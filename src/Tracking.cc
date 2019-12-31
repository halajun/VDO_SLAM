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


#include "Tracking.h"

// eigen
#include <Eigen/Core>

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
//#include<imgproc.hpp>

#include <opengv/absolute_pose/methods.hpp>
#include <opengv/absolute_pose/NoncentralAbsoluteAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>

#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>
#include<stdio.h>
#include<math.h>
#include<time.h>

#include<mutex>
#include<unistd.h>

#include <numeric>
#include <algorithm>
#include <map>
#include <random>

//#include "gco/GCoptimization.h"

using namespace std;

// cv::RNG rng;

struct LessPoint2f
{
    bool operator()(const cv::Point2f& lhs, const cv::Point2f& rhs) const
    {
        return (lhs.x == rhs.x) ? (lhs.y < rhs.y) : (lhs.x < rhs.x);
    }
};

struct EnergyDataStruct
{
    const std::vector<cv::Point2d> * const Cur_2d;
    const std::vector<cv::Point3d> * const Pre_3d;
    const cv::Mat Mods;
    const cv::Mat Camera;
    const double lambda;
    const double beta;
    const double repro_error_thres;

    EnergyDataStruct(const std::vector<cv::Point2d> * const _p1,
        const std::vector<cv::Point3d> * const _p2,
        const cv::Mat _mods,
        const cv::Mat _camera,
        const double _lambda,
        const double _beta
        ) :
        Cur_2d(_p1),
        Pre_3d(_p2),
        Mods(_mods),
        Camera(_camera),
        lambda(_lambda),
        beta(_beta),
        repro_error_thres(16)
    {
    }
};

std::vector<std::size_t> findDuplicateIndices(std::vector<int> const & v)
{
    std::vector<std::size_t> indices;
    std::map<int, std::pair<int, std::size_t>> counts;

    for (std::size_t i = 0 ; i < v.size() ; ++i)
    {
        std::size_t amount = ++counts[v[i]].first;
        if (amount == 1)
        {
            counts[v[i]].second = i;
            continue;
        }
        else if (amount == 2)
            indices.push_back(counts[v[i]].second);

        indices.push_back(i);
    }
    return indices;
}

bool sort_pair(const pair<int,float> &a,
              const pair<int,float> &b)
{
    return (a.second < b.second);
}

bool sort_pair_int(const pair<int,int> &a,
              const pair<int,int> &b)
{
    return (a.second > b.second);
}

namespace ORB_SLAM2
{

Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0), bSecondFrame(false)
{
    // Load camera parameters from settings file

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx; // for the cropping 270
    K.at<float>(1,2) = cy; // for the cropping 140
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::MONOCULAR)
        mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    if(sensor==System::STEREO || sensor==System::RGBD)
    {
        mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
        cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }

    if(sensor==System::RGBD)
    {
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(fabs(mDepthMapFactor)<1e-5)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }

}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}


cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const cv::Mat &imMask, const double &timestamp)
{
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
        }
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
        }
    }


    mCurrentFrame = Frame(mImGray,imGrayRight,imMask,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    // Save temperal matches for visualization
    TemperalMatch = vector<int>(mCurrentFrame.N,-1);
    // Initialize object label
    mCurrentFrame.vObjLabel.resize(mCurrentFrame.N,-2);


    // // *********** display stereo matching result *************
    // std::vector<cv::KeyPoint> StereoKeysL, StereoKeysR;
    // std::vector<cv::DMatch> StereoMatches;
    // int count =0;
    // for(int iL=0; iL<mCurrentFrame.N; iL++){ // mCurrentFrame.N
    //     if(mCurrentFrame.vDescIndex[iL]==-1){
    //         continue;
    //     }
    //     StereoKeysL.push_back(mCurrentFrame.mvKeys[iL]);
    //     StereoKeysR.push_back(mCurrentFrame.mvKeysRight[mCurrentFrame.vDescIndex[iL]]);
    //     StereoMatches.push_back(cv::DMatch(count,count,0));
    //     count = count + 1;
    // }
    // cout << "stereo features numeber: " << count <<  endl;
    // cv::Mat img_matches;
    // drawMatches(mImGray, StereoKeysL, imGrayRight, StereoKeysR,
    //             StereoMatches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
    //             vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    // cv::resize(img_matches, img_matches, cv::Size(img_matches.cols/1.75, img_matches.rows/1.75));
    // cv::namedWindow("stereo matches", cv::WINDOW_NORMAL);
    // cv::imshow("stereo matches", img_matches);
    // cv::waitKey(0);

    Track();

    if (timestamp!=0 && (bFrame2Frame == true || bSecondFrame == true))
    {
        // // ********** show the flow vector ************* // //
        cv::Mat flow_image = imRectLeft.clone();
        for (int i = 0; i < mCurrentFrame.N; ++i) {
            if(TemperalMatch[i]!=-1 && mCurrentFrame.vObjLabel[i]!=-1){
                Tracking::DrawTransparentSquare(cv::Point(mCurrentFrame.mvKeys[i].pt.x, mCurrentFrame.mvKeys[i].pt.y), cv::Vec3b(0, 0, 255), 3.0, 0.5, flow_image);
                Tracking::DrawLine(mCurrentFrame.mvKeys[i], mCurrentFrame.vFlow_2d[i], flow_image, cv::Vec3b(255, 0, 0));
            }
        }
        // cv::imshow("Flow Illustration", flow_image);
        // cv::waitKey(0);


        // ************** display label on the image ***************  // //
        cv::Mat label_image = imRectLeft.clone();
        std::vector<cv::KeyPoint> KeyPoints_tmp(1);
        for (int i = 0; i < mCurrentFrame.N; ++i)
        {
            if(mCurrentFrame.vObjLabel[i]==-1)
                continue;
            // if(TemperalMatch[i]==-1)
            //     continue;
            int l = mCurrentFrame.vObjLabel[i];
            // cout << "label: " << l << endl;
            KeyPoints_tmp[0] = mCurrentFrame.mvKeys[i];
            switch (l)
            {
                case 0:
                    cv::drawKeypoints(label_image, KeyPoints_tmp, label_image, cv::Scalar(0,255,255), 1); // yellow
                    break;
                case 1:
                    cv::drawKeypoints(label_image, KeyPoints_tmp, label_image, cv::Scalar(0,0,255), 1); // red
                    break;
                case 2:
                    cv::drawKeypoints(label_image, KeyPoints_tmp, label_image, cv::Scalar(0,255,0), 1); // green
                    break;
                case 3:
                    cv::drawKeypoints(label_image, KeyPoints_tmp, label_image, cv::Scalar(255,0,0), 1); // blue
                    break;
                case 4:
                    cv::drawKeypoints(label_image, KeyPoints_tmp, label_image, cv::Scalar(255,255,0), 1); // cyan
                    break;
                case 5:
                    cv::drawKeypoints(label_image, KeyPoints_tmp, label_image, cv::Scalar(203,192,255), 1); // pink
                    break;
                case 6:
                    cv::drawKeypoints(label_image, KeyPoints_tmp, label_image, cv::Scalar(128, 0, 128), 1); // purple
                    break;
                case 7:
                    cv::drawKeypoints(label_image, KeyPoints_tmp, label_image, cv::Scalar(196,228,255), 1); // bisque
                    break;
                case 8:
                    cv::drawKeypoints(label_image, KeyPoints_tmp, label_image, cv::Scalar(47,255,173), 1); // yellow green
                    break;
                case 9:
                    cv::drawKeypoints(label_image, KeyPoints_tmp, label_image, cv::Scalar(42,42,165), 1); // brown
                    break;
                case 10:
                    cv::drawKeypoints(label_image, KeyPoints_tmp, label_image, cv::Scalar(255,255,255), 1); // white
                    break;
                case 11:
                    cv::drawKeypoints(label_image, KeyPoints_tmp, label_image, cv::Scalar(0,0,0), 1); // black
                    break;
            }
        }
        // cv::imshow("Motion Segmentation Result (Sparse Representation)", label_image);
        // cv::waitKey(0);

        // ************** display them in the same window ****************
        cv::Mat dst;
        cv::vconcat(flow_image, label_image, dst);
        cv::imshow("Flow Vector (upper) and Motion Segmentation (bottom) Results", dst);
        cv::waitKey(0);


    }


    // ************** display temperal matching ***************
    // if(timestamp!=0 && bFrame2Frame == true)
    // {
    //     std::vector<cv::KeyPoint> PreKeys, CurKeys;
    //     std::vector<cv::DMatch> TemperalMatches;
    //     int count =0;
    //     for(int iL=0; iL<mCurrentFrame.N; iL++){
    //         if(TemperalMatch[iL]==-1)
    //             continue;
    //         if(mCurrentFrame.vObjLabel[iL]==-1)
    //             continue;
    //         // if(mCurrentFrame.vSemLabel[iL]==-1)
    //         //     continue;
    //         PreKeys.push_back(mvKeysLastFrame[TemperalMatch[iL]]);
    //         CurKeys.push_back(mCurrentFrame.mvKeys[iL]);
    //         TemperalMatches.push_back(cv::DMatch(count,count,0));
    //         count = count + 1;
    //     }
    //     // cout << "temperal features numeber: " << count <<  endl;

    //     cv::Mat img_matches;
    //     drawMatches(mImGrayLast, PreKeys, mImGray, CurKeys,
    //                 TemperalMatches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
    //                 vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    //     cv::resize(img_matches, img_matches, cv::Size(img_matches.cols/1.25, img_matches.rows/1.25));
    //     cv::namedWindow("temperal matches", cv::WINDOW_NORMAL);
    //     cv::imshow("temperal matches", img_matches);
    //     cv::waitKey(0);
    // }
    // mImGrayLast = mImGray;

    TemperalMatch.clear();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB, cv::Mat &imD, const cv::Mat &imFlow,
                                const cv::Mat &maskSEM, const cv::Mat &mTcw_gt, const vector<vector<float> > &vObjPose_gt,
                                const double &timestamp, cv::Mat &imTraj)
{
    cv::RNG rng((unsigned)time(NULL));

    mImGray = imRGB;

    // preprocess depth  !!! important for kitti dataset
    for (int i = 0; i < imD.rows; i++)
    {
        for (int j = 0; j < imD.cols; j++)
        {
            float dp = imD.at<float>(i,j)/256.0;
            imD.at<float>(i,j) = mbf/dp;
        }
    }

    cv::Mat imDepth = imD;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    // save map in the tracking head (new added Nov 14 2019)
    mDepthMap = imD;
    mFlowMap = imFlow;
    mSegMap = maskSEM;

    // (new added Nov 21 2019)
    if (mState!=NO_IMAGES_YET)
        UpdateMask();

    mCurrentFrame = Frame(mImGray,imDepth,imFlow,maskSEM,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    // ---------------------------------------------------------------------------------------
    // +++++++++++++++++++++++++ For sampled features ++++++++++++++++++++++++++++++++++++++++
    // ---------------------------------------------------------------------------------------

    if(bFirstFrame || bSecondFrame) // those are assigned after the first frame. so not "1st or 2nd frame".
    {
        mCurrentFrame.mvSiftKeys = mLastFrame.mvCorres;
        mCurrentFrame.N_s = mCurrentFrame.mvSiftKeys.size();

        // cout << "current number of features: " << mCurrentFrame.N_s << endl;

        // assign the depth value to each keypoint
        mCurrentFrame.mvSiftDepth = std::vector<float>(mCurrentFrame.N_s,-1);
        for(int i=0; i<mCurrentFrame.N_s; i++)
        {
            const cv::KeyPoint &kp = mCurrentFrame.mvSiftKeys[i];

            const float &v = kp.pt.y;
            const float &u = kp.pt.x;

            if (round(u)<mImGray.cols && round(u)>0 && round(v)<mImGray.rows && round(v)>0)
            {
                float d = imDepth.at<float>(round(v),round(u)); // be careful with the order  !!!

                if(d>0)
                    mCurrentFrame.mvSiftDepth[i] = d;
            }

        }

        // // Assign the semantic label to each extracted feature
        // mCurrentFrame.vSemLabel.resize(mCurrentFrame.N_s,-1);
        // mCurrentFrame.vObjLabel_gt.resize(mCurrentFrame.N_s,-1);
        // for (int i = 0; i < mCurrentFrame.N_s; ++i)
        // {
        //     const int u = mCurrentFrame.mvSiftKeys[i].pt.x;
        //     const int v = mCurrentFrame.mvSiftKeys[i].pt.y;

        //     if (u==maskSEM.cols || v==maskSEM.rows)
        //     {
        //         cout << "boundary warning ! ! !" << endl;
        //         continue;
        //     }

        //     mCurrentFrame.vSemLabel[i] = maskSEM.at<int>(v,u);  // be careful with the order  !!!
        // }

        // *********** Save object keypoints and depths ************

        // std::vector<cv::KeyPoint> ObjKeysTmp;
        // for (int i = 0; i < mLastFrame.mvObjCorres.size(); ++i)
        // {
        //     const float u = mLastFrame.mvObjCorres[i].pt.x;
        //     const float v = mLastFrame.mvObjCorres[i].pt.y;
        //     if (maskSEM.at<int>(v,u)!=0)
        //         ObjKeysTmp.push_back(mLastFrame.mvObjCorres[i]);
        // }
        // mCurrentFrame.mvObjKeys = ObjKeysTmp;
        // mCurrentFrame.mvObjDepth.resize(mCurrentFrame.mvObjKeys.size(),-1);

        // *** first assign current keypoints and depth to last frame
        // *** then assign last correspondences to current frame
        mvTmpObjKeys = mCurrentFrame.mvObjKeys;
        mvTmpObjDepth = mCurrentFrame.mvObjDepth;
        mvTmpSemObjLabel = mCurrentFrame.vSemObjLabel;
        mvTmpObjFlowNext = mCurrentFrame.mvObjFlowNext;
        mvTmpObjCorres = mCurrentFrame.mvObjCorres;

        mCurrentFrame.mvObjKeys = mLastFrame.mvObjCorres;
        mCurrentFrame.mvObjDepth.resize(mCurrentFrame.mvObjKeys.size(),-1);
        mCurrentFrame.vSemObjLabel.resize(mCurrentFrame.mvObjKeys.size(),-1);
        for (int i = 0; i < mCurrentFrame.mvObjKeys.size(); ++i)
        {
            const int u = mCurrentFrame.mvObjKeys[i].pt.x;
            const int v = mCurrentFrame.mvObjKeys[i].pt.y;
            if (u<mImGray.cols && u>0 && v<mImGray.rows && v>0 && imDepth.at<float>(v,u)<25 && imDepth.at<float>(v,u)>0)
            {
                mCurrentFrame.mvObjDepth[i] = imDepth.at<float>(v,u);
                mCurrentFrame.vSemObjLabel[i] = maskSEM.at<int>(v,u);
            }
            else
            {
                // cout << "found a point that is out of image boundary..." << endl;
                mCurrentFrame.mvObjDepth[i] = 0.1;
                mCurrentFrame.vSemObjLabel[i] = 0;
            }
            // cout << "check depth: " << " " << imDepth.at<float>(round(v),round(u)) << " " << imDepth.at<float>(v,u) << endl;
        }

        // **********************************************************

        // cout << "Amount of Dense Features: " << mCurrentFrame.mvObjKeys.size() << endl;
        // // show image
        // cv::Mat img_show;
        // cv::drawKeypoints(mImGray, mCurrentFrame.mvObjKeys, img_show, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
        // cv::imshow("Dense Feature Distribution 2", img_show);
        // cv::waitKey(0);

    }
    else
    {
        // cout << "First frame ! ! ! for initial assignment......" << endl;
        // mCurrentFrame.mvSiftKeys = mCurrentFrame.mvSiftKeysTmp;
        // mCurrentFrame.mvSiftDepth = mCurrentFrame.mvSiftDepthTmp;
        // mCurrentFrame.vSemLabel = mCurrentFrame.vSemLabelTmp;
        // mCurrentFrame.vObjLabel_gt = mCurrentFrame.vObjLabel_gtTmp;
        // mCurrentFrame.N_s = mCurrentFrame.N_s_tmp;

        // save object keypoints and depths
        // int step = 2;
        // for (int i = 0; i < mImGray.rows; i=i+step)
        // {
        //     for (int j = 0; j < mImGray.cols; j=j+step)
        //     {
        //         // check ground truth motion mask
        //         if (maskSEM.at<int>(i,j)!=0)
        //         {
        //             // save pixel location
        //             mCurrentFrame.mvObjKeys.push_back(cv::KeyPoint(j,i,0,0,0,-1));
        //             // save depth
        //             mCurrentFrame.mvObjDepth.push_back(imDepth.at<float>(i,j));
        //         }
        //     }
        // }
        // cout << "Amount of Dense Features: " << mCurrentFrame.mvObjKeys.size() << endl;
        // show image
        // cv::Mat img_show;
        // cv::drawKeypoints(mImGray, mCurrentFrame.mvObjKeys, img_show, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
        // cv::imshow("Dense Feature Distribution 1", img_show);
        // cv::waitKey(0);
    }

    // ---------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------

    // // Assign pose ground truth
    // mCurrentFrame.mTcw_gt = mTcw_gt;
    if (mState==NO_IMAGES_YET)
        mCurrentFrame.mTcw_gt = InvMatrix(mTcw_gt);
    else
        mCurrentFrame.mTcw_gt = InvMatrix(mTcw_gt)*mOriginInv;

    // Assign object pose ground truth
    mCurrentFrame.nSemPosi_gt.resize(vObjPose_gt.size());
    mCurrentFrame.vObjPose_gt.resize(vObjPose_gt.size());
    // mCurrentFrame.vObjBox_gt.resize(vObjPose_gt.size());
    for (int i = 0; i < vObjPose_gt.size(); ++i){
        // (1) label
        mCurrentFrame.nSemPosi_gt[i] = vObjPose_gt[i][1];
        // (2) pose
        mCurrentFrame.vObjPose_gt[i] = ObjPoseParsing(vObjPose_gt[i]);
        // // (3) bounding box
        // float bbox[4] = {vObjPose_gt[i][2],vObjPose_gt[i][3],vObjPose_gt[i][4],vObjPose_gt[i][5]};
        // mCurrentFrame.vObjBox_gt[i] = cv::Mat(1, 4, CV_32F, bbox);
    }

    // Save temperal matches for visualization
    TemperalMatch = vector<int>(mCurrentFrame.N_s,-1);
    // Initialize object label
    mCurrentFrame.vObjLabel.resize(mCurrentFrame.mvObjKeys.size(),-2);

    // *** main ***
    Track();
    // ************

    f_id = f_id + 1;


    // // ********** show the flow vector ************* // //
    // if (timestamp!=0 && (bFrame2Frame == true || bSecondFrame == true))
    // {
    //     cv::Mat flow_image = imRGB;
    //     for (int i = 0; i < mvKeysCurrentFrame.size(); ++i) {
    //         if(TemperalMatch[i]!=-1 && mCurrentFrame.vObjLabel[i]>=0){
    //             if(cv::norm(mCurrentFrame.vFlow_2d[i])>=0){
    //                 Tracking::DrawTransparentSquare(cv::Point(mvKeysCurrentFrame[i].pt.x, mvKeysCurrentFrame[i].pt.y), cv::Vec3b(0, 0, 255), 3.0, 0.5, flow_image);
    //                 Tracking::DrawLine(mvKeysCurrentFrame[i], mCurrentFrame.vFlow_2d[i], flow_image, cv::Vec3b(255, 0, 0));
    //             }
    //         }
    //     }
    //     cv::imshow("Flow Illustration", flow_image);
    //     cv::waitKey(0);
    // }

    // // // ************** display label on the image ***************  // //
    if(timestamp!=0 && (bFrame2Frame == true || bSecondFrame == true))
    {
        std::vector<cv::KeyPoint> KeyPoints_tmp(1);
        // background features
        for (int i = 0; i < mCurrentFrame.mvSiftKeysTmp.size(); i=i+2)
        {
            KeyPoints_tmp[0] = mCurrentFrame.mvSiftKeysTmp[i];
            if(maskSEM.at<int>(KeyPoints_tmp[0].pt.y,KeyPoints_tmp[0].pt.x)!=0)
                continue;
            cv::drawKeypoints(imRGB, KeyPoints_tmp, imRGB, cv::Scalar(0,0,255), 1); // red
        }
        // static and dynamic objects
        for (int i = 0; i < mCurrentFrame.vObjLabel.size(); ++i)
        {
            if(mCurrentFrame.vObjLabel[i]==-1 || mCurrentFrame.vObjLabel[i]==-2)
                continue;
            // int l = mCurrentFrame.vObjLabel[i];
            int l = mCurrentFrame.vSemObjLabel[i];
            // cout << "label: " << l << endl;
            KeyPoints_tmp[0] = mCurrentFrame.mvObjKeys[i];
            switch (l)
            {
                case 0:
                    cv::drawKeypoints(imRGB, KeyPoints_tmp, imRGB, cv::Scalar(0,0,255), 1); // red
                    break;
                case 1:
                    cv::drawKeypoints(imRGB, KeyPoints_tmp, imRGB, cv::Scalar(255, 165, 0), 1);
                    break;
                case 2:
                    cv::drawKeypoints(imRGB, KeyPoints_tmp, imRGB, cv::Scalar(0,255,0), 1);
                    break;
                case 3:
                    cv::drawKeypoints(imRGB, KeyPoints_tmp, imRGB, cv::Scalar(255,255,0), 1);
                    break;
                case 4:
                    cv::drawKeypoints(imRGB, KeyPoints_tmp, imRGB, cv::Scalar(255,192,203), 1);
                    break;
                case 5:
                    cv::drawKeypoints(imRGB, KeyPoints_tmp, imRGB, cv::Scalar(0,255,255), 1);
                    break;
                case 6:
                    cv::drawKeypoints(imRGB, KeyPoints_tmp, imRGB, cv::Scalar(128, 0, 128), 1);
                    break;
                case 7:
                    cv::drawKeypoints(imRGB, KeyPoints_tmp, imRGB, cv::Scalar(255,255,255), 1);
                    break;
                case 8:
                    cv::drawKeypoints(imRGB, KeyPoints_tmp, imRGB, cv::Scalar(255,228,196), 1);
                    break;
                case 9:
                    cv::drawKeypoints(imRGB, KeyPoints_tmp, imRGB, cv::Scalar(180, 105, 255), 1);
                    break;
                case 10:
                    cv::drawKeypoints(imRGB, KeyPoints_tmp, imRGB, cv::Scalar(165,42,42), 1);
                    break;
                case 11:
                    cv::drawKeypoints(imRGB, KeyPoints_tmp, imRGB, cv::Scalar(35, 142, 107), 1);
                    break;
                case 12:
                    cv::drawKeypoints(imRGB, KeyPoints_tmp, imRGB, cv::Scalar(45, 82, 160), 1);
                    break;
                case 41:
                    cv::drawKeypoints(imRGB, KeyPoints_tmp, imRGB, cv::Scalar(60, 20, 220), 1);
                    break;
                // case 1:
                //     cv::drawKeypoints(imRGB, KeyPoints_tmp, imRGB, cv::Scalar(0,255,0), 1); // green 0,255,0
                //     break;
                // case 2:
                //     cv::drawKeypoints(imRGB, KeyPoints_tmp, imRGB, cv::Scalar(255,0,0), 1); // blue
                //     break;
                // case 3:
                //     cv::drawKeypoints(imRGB, KeyPoints_tmp, imRGB, cv::Scalar(255,255,0), 1); // cyan
                //     break;
                // case 4:
                //     cv::drawKeypoints(imRGB, KeyPoints_tmp, imRGB, cv::Scalar(47,255,173), 1); // yellow green
                //     break;
                // case 5:
                //     cv::drawKeypoints(imRGB, KeyPoints_tmp, imRGB, cv::Scalar(128, 0, 128), 1); // purple
                //     break;
                // case 6:
                //     cv::drawKeypoints(imRGB, KeyPoints_tmp, imRGB, cv::Scalar(203,192,255), 1); // pink
                //     break;
                // case 7:
                //     cv::drawKeypoints(imRGB, KeyPoints_tmp, imRGB, cv::Scalar(196,228,255), 1); // bisque
                //     break;
                // case 8:
                //     cv::drawKeypoints(imRGB, KeyPoints_tmp, imRGB, cv::Scalar(42,42,165), 1); // brown
                //     break;
                // case 9:
                //     cv::drawKeypoints(imRGB, KeyPoints_tmp, imRGB, cv::Scalar(255,255,255), 1); // white
                //     break;
                // case 10:
                //     cv::drawKeypoints(imRGB, KeyPoints_tmp, imRGB, cv::Scalar(0,0,0), 1); // black
                //     break;
            }
        }
        cv::imshow("Sparse Static Features and Dense Object Points", imRGB);
        cv::waitKey(1);
    }

    // // ************** show bounding box with speed ***************
    // if(timestamp!=0 && (bFrame2Frame == true || bSecondFrame == true))
    // {
    //     cv::Mat mImBGR(mImGray.size(), CV_8UC3);
    //     cvtColor(mImGray, mImBGR, CV_GRAY2RGB);
    //     for (int i = 0; i < mCurrentFrame.vObjBoxID.size(); ++i)
    //     {
    //         // cout << "ID: " << mCurrentFrame.vObjBoxID[i] << endl;
    //         cv::Point pt1(vObjPose_gt[mCurrentFrame.vObjBoxID[i]][2], vObjPose_gt[mCurrentFrame.vObjBoxID[i]][3]);
    //         cv::Point pt2(vObjPose_gt[mCurrentFrame.vObjBoxID[i]][4], vObjPose_gt[mCurrentFrame.vObjBoxID[i]][5]);
    //         // cout << pt1.x << " " << pt1.y << " " << pt2.x << " " << pt2.y << endl;
    //         cv::rectangle(mImBGR, pt1, pt2, cv::Scalar(0, 140, 255),2);
    //         // string sp_gt = std::to_string(mCurrentFrame.vSpeed[i].y);
    //         string sp_est = std::to_string(mCurrentFrame.vSpeed[i].x);
    //         // sp_gt.resize(5);
    //         sp_est.resize(5);
    //         // string output_gt = "GT:" + sp_gt + "km/h";
    //         string output_est = sp_est + "km/h";
    //         cv::putText(mImBGR, output_est, cv::Point(pt1.x, pt1.y-10), cv::FONT_HERSHEY_DUPLEX, 0.9, CV_RGB(255,140,0), 2);
    //         // cv::putText(mImBGR, output_gt, cv::Point(pt1.x, pt1.y-32), cv::FONT_HERSHEY_DUPLEX, 0.7, CV_RGB(255, 0, 0), 2);
    //     }
    //     cv::imshow("Object Speed", mImBGR);
    //     cv::waitKey(1);
    // }

    // // ************** show trajectory results ***************
    int sta_x = 300, sta_y = 120, radi = 2, thic = 5;  // (160/120/2/5)
    float scale = 6; // 6
    cv::Mat CamPos = InvMatrix(mCurrentFrame.mTcw);
    int x = int(CamPos.at<float>(0,3)*scale) + sta_x;
    int y = int(CamPos.at<float>(2,3)*scale) + sta_y;
    // cv::circle(imTraj, cv::Point(x, y), radi, CV_RGB(255,0,0), thic);
    cv::rectangle(imTraj, cv::Point(x, y), cv::Point(x+10, y+10), cv::Scalar(0,0,255),1);
    cv::rectangle(imTraj, cv::Point(10, 30), cv::Point(550, 60), CV_RGB(0,0,0), CV_FILLED);
    cv::putText(imTraj, "Camera Trajectory (RED SQUARE)", cv::Point(10, 30), cv::FONT_HERSHEY_COMPLEX, 0.6, CV_RGB(255, 255, 255), 1);
    char text[100];
    sprintf(text, "x = %02fm y = %02fm z = %02fm", CamPos.at<float>(0,3), CamPos.at<float>(1,3), CamPos.at<float>(2,3));
    cv::putText(imTraj, text, cv::Point(10, 50), cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar::all(255), 1);
    cv::putText(imTraj, "Object Trajectories (COLORED CIRCLES)", cv::Point(10, 70), cv::FONT_HERSHEY_COMPLEX, 0.6, CV_RGB(255, 255, 255), 1);

    for (int i = 0; i < mCurrentFrame.vObjCentre3D.size(); ++i)
    {
        int x = int(mCurrentFrame.vObjCentre3D[i].at<float>(0,0)*scale) + sta_x;
        int y = int(mCurrentFrame.vObjCentre3D[i].at<float>(0,2)*scale) + sta_y;
        int l = mCurrentFrame.nSemPosition[i];
        // int l = mCurrentFrame.nModLabel[i];
        switch (l)
        {
            case 1:
                cv::circle(imTraj, cv::Point(x, y), radi, CV_RGB(0, 165, 255), thic); // orange
                break;
            case 2:
                cv::circle(imTraj, cv::Point(x, y), radi, CV_RGB(0,255,0), thic); // green
                break;
            case 3:
                cv::circle(imTraj, cv::Point(x, y), radi, CV_RGB(0,255,255), thic); // yellow
                break;
            case 4:
                cv::circle(imTraj, cv::Point(x, y), radi, CV_RGB(203,192,255), thic); // pink
                break;
            case 5:
                cv::circle(imTraj, cv::Point(x, y), radi, CV_RGB(255,255,0), thic); // cyan (yellow green 47,255,173)
                break;
            case 6:
                cv::circle(imTraj, cv::Point(x, y), radi, CV_RGB(128, 0, 128), thic); // purple
                break;
            case 7:
                cv::circle(imTraj, cv::Point(x, y), radi, CV_RGB(255,255,255), thic);  // white
                break;
            case 8:
                cv::circle(imTraj, cv::Point(x, y), radi, CV_RGB(196,228,255), thic); // bisque
                break;
            case 9:
                cv::circle(imTraj, cv::Point(x, y), radi, CV_RGB(180, 105, 255), thic);  // blue
                break;
            case 10:
                cv::circle(imTraj, cv::Point(x, y), radi, CV_RGB(42,42,165), thic);  // brown
                break;
            case 11:
                cv::circle(imTraj, cv::Point(x, y), radi, CV_RGB(35, 142, 107), thic);
                break;
            case 12:
                cv::circle(imTraj, cv::Point(x, y), radi, CV_RGB(45, 82, 160), thic);
                break;
            case 41:
                cv::circle(imTraj, cv::Point(x, y), radi, CV_RGB(60, 20, 220), thic);
                break;
        }
    }

    // imshow( "Camera and Object Trajectories", imTraj);
    // cv::waitKey(0);

    // // ************** display temperal matching ***************
    // if(timestamp!=0 && (bFrame2Frame == true || bSecondFrame == true))
    // {
    //     std::vector<cv::KeyPoint> PreKeys, CurKeys;
    //     std::vector<cv::DMatch> TemperalMatches;
    //     int count =0;
    //     for(int iL=0; iL<mvKeysCurrentFrame.size(); iL++)
    //     {
    //         if(TemperalMatch[iL]==-1)
    //             continue;
    //         // if(checkit[iL]==0)
    //         //     continue;
    //         if(mCurrentFrame.vObjLabel[iL]<=0)
    //             continue;
    //         // if(mCurrentFrame.vSemLabel[iL]==0) // || cv::norm(mCurrentFrame.vFlow_3d[iL])>0.15
    //         //     continue;
    //         // if(cv::norm(mCurrentFrame.vFlow_3d[iL])<0.15)
    //         //     continue;
    //         PreKeys.push_back(mvKeysLastFrame[TemperalMatch[iL]]);
    //         CurKeys.push_back(mvKeysCurrentFrame[iL]);
    //         TemperalMatches.push_back(cv::DMatch(count,count,0));
    //         count = count + 1;
    //     }
    //     // cout << "temperal features numeber: " << count <<  endl;

    //     cv::Mat img_matches;
    //     drawMatches(mImGrayLast, PreKeys, mImGray, CurKeys,
    //                 TemperalMatches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
    //                 vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    //     cv::resize(img_matches, img_matches, cv::Size(img_matches.cols/1.5, img_matches.rows/1.5));
    //     cv::namedWindow("temperal matches", cv::WINDOW_NORMAL);
    //     cv::imshow("temperal matches", img_matches);
    //     cv::waitKey(0);
    // }

    mImGrayLast = mImGray;
    TemperalMatch.clear();
    mSegMapLast = mSegMap;   // new added Nov 21 2019
    mFlowMapLast = mFlowMap; // new added Nov 21 2019

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
{
    mImGray = im;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    else
        mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}

void Tracking::Track()
{
    // cv::RNG rng((unsigned)time(NULL));

    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    if(mState==NOT_INITIALIZED)
    {
        bFirstFrame = true;
        bFrame2Frame = false;
        bSecondFrame = false;

        if(mSensor==System::STEREO || mSensor==System::RGBD)
            StereoInitialization();
        else
            MonocularInitialization();

        mpFrameDrawer->Update(this);

        if(mState!=OK)
            return;
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(!mbOnlyTracking)
        {
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            if(mState==OK)
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();

                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    bOK = TrackReferenceKeyFrame();
                }
                else
                {
                    bOK = TrackWithMotionModel();
                    if(!bOK)
                        bOK = TrackReferenceKeyFrame();
                }
            }
            else
            {
                bOK = Relocalization();
            }
        }
        else
        {
            // Localization Mode: Local Mapping is deactivated

            if(mState==LOST)
            {
                bOK = Relocalization();
            }
            else
            {
                if(!mbVO)
                {
                    // In last frame we tracked enough MapPoints in the map

                    if(!mVelocity.empty())
                    {
                        bOK = TrackWithMotionModel();
                    }
                    else
                    {
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint*> vpMPsMM;
                    vector<bool> vbOutMM;
                    cv::Mat TcwMM;
                    if(!mVelocity.empty())
                    {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.mTcw.clone();
                    }
                    bOKReloc = Relocalization();

                    if(bOKMM && !bOKReloc)
                    {
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        if(mbVO)
                        {
                            for(int i =0; i<mCurrentFrame.N; i++)
                            {
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    else if(bOKReloc)
                    {
                        mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(!mbOnlyTracking)
        {
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        if(bOK)
            mState = OK;
        else
        {
            mState=LOST;
            cout << "LOST TRACK !" << endl;
        }

        // Update drawer
        mpFrameDrawer->Update(this);

        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
            // Update motion model
            if(!mLastFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
                mVelocity = cv::Mat();

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            // Clean VO matches
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // Delete temporal MapPoints
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

            // Check if we need to insert a new keyframe
            if(NeedNewKeyFrame())
                CreateNewKeyFrame();

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5)
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;


        // ---------------------------------------------------------------------------------------
        // ++++++++++++++++++++ Compute Sparse Scene Flow ++++++++++++++++++++++++++++++++++++++++
        // ---------------------------------------------------------------------------------------

        // cout << "start matching......" << endl;
        // // redo the matching to all the features using projecting matching
        // ORBmatcher matcher(0.9,true);
        // int num_matches = matcher.ProjMatching(mCurrentFrame,mLastFrame,TemperalMatch, bSecondFrame);
        // int num_matches = matcher.SearchByQuad(mCurrentFrame,mLastFrame,TemperalMatch);

        // // *********** for sampled features ***********
        // int num_matches = mCurrentFrame.N_s;
        for (int i = 0; i < mCurrentFrame.N_s; ++i){
            TemperalMatch[i] = i;
        }
        // // ********************************************

        // cout << "New Matching result~ ~ ~ ~ ~ ~: " << num_matches << endl;

        // calculate the re-projection error (static features)
        float Rpe_sum = 0, sta_num = 0;
        std::vector<float> flow_error(mCurrentFrame.N_s,0.0);
        std::vector<Eigen::Vector2d> of_gt_cam(mCurrentFrame.N_s);
        std::vector<int> of_range_cam(20,0);
        for (int i = 0; i < mCurrentFrame.N_s; ++i)
        {
            cv::Mat x3D_p = mLastFrame.UnprojectStereoSift(TemperalMatch[i],0);
            cv::Mat Tcw_gt = mCurrentFrame.mTcw_gt;
            cv::Mat x3D_pc = Tcw_gt.rowRange(0,3).colRange(0,3)*x3D_p+Tcw_gt.rowRange(0,3).col(3);

            float xc = x3D_pc.at<float>(0);
            float yc = x3D_pc.at<float>(1);
            float invzc = 1.0/x3D_pc.at<float>(2);
            float u = mCurrentFrame.fx*xc*invzc+mCurrentFrame.cx;
            float v = mCurrentFrame.fy*yc*invzc+mCurrentFrame.cy;
            float u_ = mCurrentFrame.mvSiftKeys[i].pt.x - u;
            float v_ = mCurrentFrame.mvSiftKeys[i].pt.y - v;

            of_gt_cam[i](0) = u - mLastFrame.mvSiftKeys[TemperalMatch[i]].pt.x;
            of_gt_cam[i](1) = v - mLastFrame.mvSiftKeys[TemperalMatch[i]].pt.y;

            float ofe = std::sqrt(u_*u_ + v_*v_);
            flow_error[i] = ofe;

            {
                if (0.0<=ofe && ofe<0.5)
                    of_range_cam[0] = of_range_cam[0] + 1;
                else if (0.5<=ofe && ofe<1.0)
                    of_range_cam[1] = of_range_cam[1] + 1;
                else if (1.0<=ofe && ofe<1.5)
                    of_range_cam[2] = of_range_cam[2] + 1;
                else if (1.5<=ofe && ofe<2.0)
                    of_range_cam[3] = of_range_cam[3] + 1;
                else if (2.0<=ofe && ofe<2.5)
                    of_range_cam[4] = of_range_cam[4] + 1;
                else if (2.5<=ofe && ofe<3.0)
                    of_range_cam[5] = of_range_cam[5] + 1;
                else if (3.0<=ofe && ofe<3.5)
                    of_range_cam[6] = of_range_cam[6] + 1;
                else if (3.5<=ofe && ofe<4.0)
                    of_range_cam[7] = of_range_cam[7] + 1;
                else if (4.0<=ofe && ofe<4.5)
                    of_range_cam[8] = of_range_cam[8] + 1;
                else if (4.5<=ofe && ofe<5.0)
                    of_range_cam[9] = of_range_cam[9] + 1;
                else if (5.0<=ofe && ofe<5.5)
                    of_range_cam[10] = of_range_cam[10] + 1;
                else if (5.5<=ofe && ofe<6.0)
                    of_range_cam[11] = of_range_cam[11] + 1;
                else if (6.0<=ofe && ofe<6.5)
                    of_range_cam[12] = of_range_cam[12] + 1;
                else if (6.5<=ofe && ofe<7.0)
                    of_range_cam[13] = of_range_cam[13] + 1;
                else if (7.0<=ofe && ofe<7.5)
                    of_range_cam[14] = of_range_cam[14] + 1;
                else if (7.5<=ofe && ofe<8.0)
                    of_range_cam[15] = of_range_cam[15] + 1;
                else if (8.0<=ofe && ofe<8.5)
                    of_range_cam[16] = of_range_cam[16] + 1;
                else if (8.5<=ofe && ofe<9.0)
                    of_range_cam[17] = of_range_cam[17] + 1;
                else if (9.0<=ofe && ofe<10.0)
                    of_range_cam[18] = of_range_cam[18] + 1;
                else if (10.0<=ofe)
                    of_range_cam[19] = of_range_cam[19] + 1;
            }

            Rpe_sum = Rpe_sum + ofe;
            sta_num = sta_num + 1.0;
        }

        // cout << "AVG static optical flow error: " << sta_num << " " << Rpe_sum/sta_num << endl;

        // cout << "background optical flow distribution: " << endl;
        // for (int j = 0; j < of_range_cam.size(); ++j)
        //     cout << of_range_cam[j] << " ";
        // cout << endl;

        // Get initial estimate using PnP plus RanSac
        std::vector<int> TemperalMatch_subset;
        cv::Mat iniTcw = GetInitModelCam(TemperalMatch,TemperalMatch_subset);

        std::vector<Eigen::Vector2d> of_gt_in_cam(TemperalMatch_subset.size());
        std::vector<double> e_bef_cam(TemperalMatch_subset.size());
        for (int i = 0; i < TemperalMatch_subset.size(); ++i)
        {
            of_gt_in_cam[i] = of_gt_cam[TemperalMatch_subset[i]];
            e_bef_cam[i] = flow_error[TemperalMatch_subset[i]];
        }


        // cout << "the ground truth pose (inv): " << endl << InvMatrix(mCurrentFrame.mTcw_gt) << endl;
        // cout << "the ground truth pose: " << endl << mCurrentFrame.mTcw_gt << endl;
        // cout << "initial pose: " << endl << iniTcw << endl;
        // // compute the pose with new matching
        mCurrentFrame.SetPose(iniTcw);
        // Optimizer::PoseOptimizationNew(&mCurrentFrame, &mLastFrame, TemperalMatch_subset);
        Optimizer::PoseOptimizationFlow2Cam(&mCurrentFrame, &mLastFrame, TemperalMatch_subset, of_gt_in_cam, e_bef_cam);
        // cout << "pose after update: " << endl << mCurrentFrame.mTcw << endl;

        // Update motion model
        if(!mLastFrame.mTcw.empty())
        {
            cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
            mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
            mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
            mVelocity = mCurrentFrame.mTcw*LastTwc;
        }

        cv::Mat Tcw_est_inv = InvMatrix(mCurrentFrame.mTcw);
        cv::Mat RePoEr_cam = Tcw_est_inv*mCurrentFrame.mTcw_gt;
        // cout << "error matrix: " << endl << RePoEr_cam << endl;
        // cv::Mat T_lc_inv = mCurrentFrame.mTcw*InvMatrix(mLastFrame.mTcw);
        // cv::Mat T_lc_gt = mLastFrame.mTcw_gt*InvMatrix(mCurrentFrame.mTcw_gt);
        // cv::Mat RePoEr_cam = T_lc_inv*T_lc_gt;

        float t_rpe_cam = std::sqrt( RePoEr_cam.at<float>(0,3)*RePoEr_cam.at<float>(0,3) + RePoEr_cam.at<float>(1,3)*RePoEr_cam.at<float>(1,3) + RePoEr_cam.at<float>(2,3)*RePoEr_cam.at<float>(2,3) );
        float trace_rpe_cam = 0;
        for (int i = 0; i < 3; ++i)
        {
            if (RePoEr_cam.at<float>(i,i)>1.0)
                 trace_rpe_cam = trace_rpe_cam + 1.0-(RePoEr_cam.at<float>(i,i)-1.0);
            else
                trace_rpe_cam = trace_rpe_cam + RePoEr_cam.at<float>(i,i);
        }
        cout << std::fixed << std::setprecision(6);
        float r_rpe_cam = acos( (trace_rpe_cam -1.0)/2.0 )*180.0/3.1415926;

        cout << "the relative pose error of estimated camera pose, " << "t: " << t_rpe_cam <<  " R: " << r_rpe_cam << endl;

        // float t_gt_cam = std::sqrt( T_lc_gt.at<float>(0,3)*T_lc_gt.at<float>(0,3) + T_lc_gt.at<float>(1,3)*T_lc_gt.at<float>(1,3) + T_lc_gt.at<float>(2,3)*T_lc_gt.at<float>(2,3) );
        // cout << "the relative pose error of estimated camera pose, " << "t: " << (t_rpe_cam/t_gt_cam)*100 << "%" << " R: " << r_rpe_cam/t_gt_cam << "deg/m" << endl;
        // mpMap->vvCamMotErr_2.push_back(cv::Point2f(t_rpe_cam/t_gt_cam,r_rpe_cam/t_gt_cam));

        mpMap->vvCamMotErr_1.push_back(cv::Point2f(t_rpe_cam,r_rpe_cam));
        mpMap->vmCameraPose_main.push_back(InvMatrix(mCurrentFrame.mTcw));

        // // // image show the matching
        // std::vector<cv::KeyPoint> PreKeys(TemperalMatch_subset.size()), CurKeys(TemperalMatch_subset.size());
        // std::vector<cv::DMatch> TMes;
        // int count_ =0;
        // for (int i = 0; i < TemperalMatch_subset.size(); i=i+3)
        // {
        //     // save key points for visualization
        //     PreKeys[i]=mLastFrame.mvSiftKeys[TemperalMatch_subset[i]];
        //     CurKeys[i]=mCurrentFrame.mvSiftKeys[TemperalMatch_subset[i]];
        //     cv::DMatch aaa = cv::DMatch(count_,count_,0);
        //     TMes.push_back(aaa);
        //     count_ = count_ + 1;
        // }
        // cv::Mat img_matches;
        // drawMatches(mImGrayLast, PreKeys, mImGray, CurKeys,
        //             TMes, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
        //             vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        // cv::resize(img_matches, img_matches, cv::Size(img_matches.cols/1.2, img_matches.rows/1.2));
        // cv::namedWindow("temperal matches", cv::WINDOW_NORMAL);
        // cv::imshow("temperal matches", img_matches);
        // cv::waitKey(0);

        // // **** show the picked points ****
        // std::vector<cv::KeyPoint> PickKeys;
        // for (int j = 0; j < TemperalMatch_subset.size(); ++j){
        //     PickKeys.push_back(mCurrentFrame.mvSiftKeys[TemperalMatch_subset[j]]);
        // }
        // cv::drawKeypoints(mImGray, PickKeys, mImGray, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
        // cv::imshow("KeyPoints and Grid on Background", mImGray);
        // cv::waitKey(0);

        // // -----------------------------------------------------------------------------------------------
        // // -----------------------------------------------------------------------------------------------
        // // -----------------------------------------------------------------------------------------------

        cout << "--------------------------------------------" << endl;
        cout << "..........Dealing with Objects Now.........." << endl;
        cout << "--------------------------------------------" << endl;

        // // === compute sparse scene flow to the found matches ===
        // // GetSceneFlowSift(TemperalMatch);
        // // GetSceneFlow(TemperalMatch);
        GetSceneFlowObj();


        // // ---------------------------------------------------------------------------------------
        // // ++++++++++++++++++++++++++ Separate object with semantic prior ++++++++++++++++++++++++
        // // ---------------------------------------------------------------------------------------

        // find the unique labels in semantic label
        auto UniLab = mCurrentFrame.vSemObjLabel;
        std::sort(UniLab.begin(), UniLab.end());
        UniLab.erase(std::unique( UniLab.begin(), UniLab.end() ), UniLab.end() );

        // cout << "UniqueLabel::: ";
        // for (int i = 0; i < UniLab.size(); ++i)
        //     cout  << UniLab[i] << " ";
        // cout << endl;

        // collect the predicted labels and semantic labels in vector
        std::vector<std::vector<int> > Posi(UniLab.size());
        for (int i = 0; i < mCurrentFrame.vSemObjLabel.size(); ++i)
        {
            // skip outliers
            if (mCurrentFrame.vObjLabel[i]==-1)
                continue;

            // save object label
            for (int j = 0; j < UniLab.size(); ++j)
            {
                if(mCurrentFrame.vSemObjLabel[i]==UniLab[j]){
                    Posi[j].push_back(i);
                    break;
                }
            }
        }

        // // save objects only from Posi() -> ObjId()
        std::vector<std::vector<int> > ObjId;
        std::vector<int> sem_posi; // semantic label position for the objects
        for (int i = 0; i < Posi.size(); ++i)
        {
            // shrink the image to get rid of object parts on the boundary
            float count = 0, count_thres=0.5;
            for (int j = 0; j < Posi[i].size(); ++j)
            {
                const float u = mCurrentFrame.mvObjKeys[Posi[i][j]].pt.x;
                const float v = mCurrentFrame.mvObjKeys[Posi[i][j]].pt.y;
                if ( v<25 || v>(mImGray.rows-25) || u<50 || u>(mImGray.cols-50) )
                    count = count + 1;
            }
            if (count/Posi[i].size()>count_thres)
            {
                cout << "Most part of this object is on the image boundary......" << endl;
                for (int k = 0; k < Posi[i].size(); ++k)
                    mCurrentFrame.vObjLabel[Posi[i][k]] = -1;
                continue;
            }

            // save object that has more than certain number of points
            if (Posi[i].size()>100)
            {
                ObjId.push_back(Posi[i]);
                sem_posi.push_back(UniLab[i]);
            }
            else
            {
                // cout << "object " << UniLab[i] << " found that contains less than 100 points..." << endl;
                for (int k = 0; k < Posi[i].size(); ++k)
                    mCurrentFrame.vObjLabel[Posi[i][k]] = -1;
                continue;
            }
        }

        // // check scene flow distribution of each object
        // // and keep the dynamic object
        float sf_thres=0.12, sf_percent=0.3;
        std::vector<std::vector<int> > ObjIdNew;
        std::vector<int> SemPosNew;
        for (int i = 0; i < ObjId.size(); ++i)
        {

            float obj_center_depth = 0, sf_min=100, sf_max=0, sf_mean=0, sf_count=0;
            std::vector<int> sf_range(10,0);
            for (int j = 0; j < ObjId[i].size(); ++j)
            {
                obj_center_depth = obj_center_depth + mCurrentFrame.mvObjDepth[ObjId[i][j]];
                // const float sf_norm = cv::norm(mCurrentFrame.vFlow_3d[ObjId[i][j]]);
                float sf_norm = std::sqrt(mCurrentFrame.vFlow_3d[ObjId[i][j]].x*mCurrentFrame.vFlow_3d[ObjId[i][j]].x + mCurrentFrame.vFlow_3d[ObjId[i][j]].z*mCurrentFrame.vFlow_3d[ObjId[i][j]].z);
                if (sf_norm<sf_thres)
                    sf_count = sf_count+1;
                if(sf_norm<sf_min)
                    sf_min = sf_norm;
                if(sf_norm>sf_max)
                    sf_max = sf_norm;
                sf_mean = sf_mean + sf_norm;
                {
                    if (0.0<=sf_norm && sf_norm<0.05)
                        sf_range[0] = sf_range[0] + 1;
                    else if (0.05<=sf_norm && sf_norm<0.1)
                        sf_range[1] = sf_range[1] + 1;
                    else if (0.1<=sf_norm && sf_norm<0.2)
                        sf_range[2] = sf_range[2] + 1;
                    else if (0.2<=sf_norm && sf_norm<0.4)
                        sf_range[3] = sf_range[3] + 1;
                    else if (0.4<=sf_norm && sf_norm<0.8)
                        sf_range[4] = sf_range[4] + 1;
                    else if (0.8<=sf_norm && sf_norm<1.6)
                        sf_range[5] = sf_range[5] + 1;
                    else if (1.6<=sf_norm && sf_norm<3.2)
                        sf_range[6] = sf_range[6] + 1;
                    else if (3.2<=sf_norm && sf_norm<6.4)
                        sf_range[7] = sf_range[7] + 1;
                    else if (6.4<=sf_norm && sf_norm<12.8)
                        sf_range[8] = sf_range[8] + 1;
                    else if (12.8<=sf_norm && sf_norm<25.6)
                        sf_range[9] = sf_range[9] + 1;
                }
            }

            // cout << "object center depth: " << obj_center_depth/ObjId[i].size() << endl;
            // cout << "object proportion over whole image: " << ObjId[i].size()*2.0*2.0/465750.0*100 << "%" << endl;
            // cout << "scene flow distribution:"  << endl;
            // for (int j = 0; j < sf_range.size(); ++j)
            //     cout << sf_range[j] << " ";
            // cout << endl;
            // cout << "scene flow statistics: " << sf_count << " " << sf_min << " " << sf_max << " " << sf_mean/ObjId[i].size() << endl;

            if (sf_count/ObjId[i].size()>sf_percent)
            {
                // label this object as static background
                for (int k = 0; k < ObjId[i].size(); ++k)
                    mCurrentFrame.vObjLabel[ObjId[i][k]] = 0;
                continue;
            }
            else if (obj_center_depth/ObjId[i].size()>25.0 || ObjId[i].size()<200)
            {
                // cout << "object " << sem_posi[i] <<" is too far away or too small!" << endl;
                // label this object as far away object
                for (int k = 0; k < ObjId[i].size(); ++k)
                    mCurrentFrame.vObjLabel[ObjId[i][k]] = -1;
                continue;
            }
            else
            {
                // cout << "get new objects!" << endl;
                ObjIdNew.push_back(ObjId[i]);
                SemPosNew.push_back(sem_posi[i]);
            }
        }

        mpMap->vTotObjNum.push_back(ObjId.size());

        // *************************************************************

        // // *** To show the points on object ***
        // for (int i = 0; i < ObjIdNew.size(); ++i)
        // {
        //     // **** show the picked points ****
        //     std::vector<cv::KeyPoint> PickKeys;
        //     for (int j = 0; j < ObjIdNew[i].size(); ++j){
        //         PickKeys.push_back(mCurrentFrame.mvObjKeys[ObjIdNew[i][j]]);
        //     }
        //     cv::drawKeypoints(mImGray, PickKeys, mImGray, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
        //     cv::imshow("KeyPoints and Grid on Background", mImGray);
        //     cv::waitKey(0);
        // }


        // relabel the objects that associate with the objects in last frame
        int max;
        if (bSecondFrame)
            max = 1;
        else
        {
            if (mLastFrame.nModLabel.size()>0)
            {
                auto v = mLastFrame.nModLabel;
                auto result = std::max_element(std::begin(v), std::end(v));
                max = v[std::distance(v.begin(), result)] + 1;
            }
        }
        // save current label id
        std::vector<int> LabId(ObjIdNew.size());
        for (int i = 0; i < ObjIdNew.size(); ++i)
        {
            // save semantic labels in last frame
            std::vector<int> Lb_last;
            for (int k = 0; k < ObjIdNew[i].size(); ++k)
                Lb_last.push_back(mLastFrame.vSemObjLabel[ObjIdNew[i][k]]);

            // find label that appears most in Lb_last()
            // (1) count duplicates
            std::map<int, int> dups;
            for(int k : Lb_last)
                ++dups[k];
            // (2) and sort them by descending order
            std::vector<std::pair<int, int> > sorted;
            for (auto k : dups)
                sorted.push_back(std::make_pair(k.first,k.second));
            std::sort(sorted.begin(), sorted.end(), sort_pair_int);

            // label the object in current frame
            int New_lab = sorted[0].first;
            // cout << " what is in the new label: " << New_lab << endl;
            if (bSecondFrame)
            {
                LabId[i] = max;
                for (int k = 0; k < ObjIdNew[i].size(); ++k)
                    mCurrentFrame.vObjLabel[ObjIdNew[i][k]] = max;
                max = max + 1;
            }
            else
            {
                bool exist = false;
                for (int k = 0; k < mLastFrame.nSemPosition.size(); ++k)
                {
                    if (mLastFrame.nSemPosition[k]==New_lab)
                    {
                        LabId[i] = mLastFrame.nModLabel[k];
                        for (int k = 0; k < ObjIdNew[i].size(); ++k)
                            mCurrentFrame.vObjLabel[ObjIdNew[i][k]] = LabId[i];
                        exist = true;
                        break;
                    }
                }
                if (exist==false)
                {
                    LabId[i] = max;
                    for (int k = 0; k < ObjIdNew[i].size(); ++k)
                        mCurrentFrame.vObjLabel[ObjIdNew[i][k]] = max;
                    max = max + 1;
                }
            }

        }


        // // assign the model label in current frame
        mCurrentFrame.nModLabel = LabId;
        mCurrentFrame.nSemPosition = SemPosNew;

        cout << "motion label: ";
        for (int i = 0; i < LabId.size(); ++i)
            cout <<  LabId[i] << " ";
        cout << endl;

        // // ---------------------------------------------------------------------------------------
        // // ++++++++++++++++++++++++++ Motion Estimation for each object ++++++++++++++++++++++++++
        // // ---------------------------------------------------------------------------------------

        // // string filename = "of_dist.txt";
        // // ofstream save_distr;
        // // save_distr.open(filename.c_str(),ios::trunc);

        // some results to be saved
        std::vector<int> vObjMotID;
        std::vector<cv::Point2f> vObjMotErr_1;
        std::vector<cv::Point2f> vObjMotErr_2;
        std::vector<cv::Point2f> vObjMotErr_3;

        mCurrentFrame.vObjMod.resize(ObjIdNew.size());
        mCurrentFrame.vObjMod_gt.resize(ObjIdNew.size());
        mCurrentFrame.vObjSpeed_gt.resize(ObjIdNew.size());
        mCurrentFrame.vSpeed.resize(ObjIdNew.size());
        mCurrentFrame.vObjBoxID.resize(ObjIdNew.size());
        mCurrentFrame.vObjCentre3D.resize(ObjIdNew.size());
        mCurrentFrame.vnObjID.resize(ObjIdNew.size());
        mCurrentFrame.vnObjInlierID.resize(ObjIdNew.size());
        // repro_e.resize(ObjIdNew.size(),0.0);
        cv::Mat Last_Twc_gt = InvMatrix(mLastFrame.mTcw_gt); // inverse of camera pose
        cv::Mat Curr_Twc_gt = InvMatrix(mCurrentFrame.mTcw_gt); // inverse of camera pose
        for (int i = 0; i < ObjIdNew.size(); ++i)
        {
            // *****************************************************************************
            // cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;

            // get the ground truth object motion
            cv::Mat L_p, L_c, L_w_p, L_w_c, H_p_c; // previous and current and world
            for (int k = 0; k < mLastFrame.nSemPosi_gt.size(); ++k){
                if (mLastFrame.nSemPosi_gt[k]==mCurrentFrame.nSemPosition[i]){
                    // cout << "it is " << mLastFrame.nSemPosi_gt[k] << "!" << endl;
                    L_p = mLastFrame.vObjPose_gt[k];
                    // cout << "what is L_p: " << endl << L_p << endl;
                    L_w_p = Last_Twc_gt*L_p;
                    // cout << "what is L_w_p: " << endl << L_w_p << endl;
                    break;
                }
            }
            for (int k = 0; k < mCurrentFrame.nSemPosi_gt.size(); ++k){
                if (mCurrentFrame.nSemPosi_gt[k]==mCurrentFrame.nSemPosition[i]){
                    // cout << "it is " << mCurrentFrame.nSemPosi_gt[k] << "!" << endl;
                    L_c = mCurrentFrame.vObjPose_gt[k];
                    // cout << "what is L_c: " << endl << L_c << endl;
                    L_w_c = Curr_Twc_gt*L_c;
                    // cout << "what is L_w_c: " << endl << L_w_c << endl;
                    mCurrentFrame.vObjBoxID[i] = k;
                    break;
                }
            }
            cv::Mat L_w_p_inv = InvMatrix(L_w_p);
            H_p_c = L_w_c*L_w_p_inv;
            mCurrentFrame.vObjMod_gt[i] = H_p_c;

            // cout << "ground truth motion of object No. " << mCurrentFrame.nSemPosition[i] << " :" << endl;
            // cout << H_p_c << endl;

            // *****************************************************************************

            std::vector<cv::KeyPoint> PreKeys, CurKeys;
            std::vector<cv::DMatch> TMes;
            std::vector<int> ObjIdTest, of_range(20,0),of_range_x(20,0),of_range_y(20,0);
            std::vector<cv::Point2f> of_dis(mCurrentFrame.mvObjKeys.size());
            std::vector<Eigen::Vector2d> of_gt(mCurrentFrame.mvObjKeys.size());
            cv::Mat ObjCen3D = (cv::Mat_<float>(3,1) << 0.f, 0.f, 0.f);
            // std::vector<float> point_dis(mCurrentFrame.mvObjKeys.size());
            float avg_of = 0, avg_of_x = 0, avg_of_y = 0;
            int x_max=0,y_max=0,x_min=2000,y_min=2000;

            for (int j = 0; j < ObjIdNew[i].size(); ++j)
            {
                // save object centroid
                cv::Mat x3D_c = mCurrentFrame.UnprojectStereoObject(ObjIdNew[i][j],0);
                ObjCen3D = ObjCen3D + x3D_c;

                float x = mCurrentFrame.mvObjKeys[ObjIdNew[i][j]].pt.x;
                float y = mCurrentFrame.mvObjKeys[ObjIdNew[i][j]].pt.y;

                CurKeys.push_back(mCurrentFrame.mvObjKeys[ObjIdNew[i][j]]);

                // *** get the correspondence using ground truth camera pose and object motion. ***
                // (0) move 3D via object motion
                cv::Mat x3D_p = mLastFrame.UnprojectStereoObject(ObjIdNew[i][j],0);
                const cv::Mat R = H_p_c.rowRange(0,3).colRange(0,3);
                const cv::Mat t = H_p_c.rowRange(0,3).col(3);
                cv::Mat x3D_c_est = R*x3D_p+t;

                // get the 2D projection
                // (1) transfer 3d from world to current frame.
                const cv::Mat R_c = mCurrentFrame.mTcw_gt.rowRange(0,3).colRange(0,3);
                const cv::Mat t_c = mCurrentFrame.mTcw_gt.rowRange(0,3).col(3);
                cv::Mat x3D_c_est_cam = R_c*x3D_c_est+t_c;
                // (2) project 3d into current image plane
                const float xc = x3D_c_est_cam.at<float>(0);
                const float yc = x3D_c_est_cam.at<float>(1);
                const float invzc = 1.0/x3D_c_est_cam.at<float>(2);
                const float u = mCurrentFrame.fx*xc*invzc+mCurrentFrame.cx;
                const float v = mCurrentFrame.fy*yc*invzc+mCurrentFrame.cy;

                // save the boundary
                if (x>x_max)
                    x_max = x;
                if (x<x_min)
                    x_min = x;
                if (y>y_max)
                    y_max = y;
                if (y<y_min)
                    y_min = y;

                // // // *** Get ground true correspondence *** // //
                // mCurrentFrame.mvObjKeys[ObjIdNew[i][j]].pt.x = u;
                // mCurrentFrame.mvObjKeys[ObjIdNew[i][j]].pt.y = v;

                // // // *** Get ground optical flow *** // //
                // mLastFrame.mvObjFlowNext[ObjIdNew[i][j]].x = u - mLastFrame.mvObjKeys[ObjIdNew[i][j]].pt.x;
                // mLastFrame.mvObjFlowNext[ObjIdNew[i][j]].y = v - mLastFrame.mvObjKeys[ObjIdNew[i][j]].pt.y;

                const float u_ = x - u;
                const float v_ = y - v;
                const float ofe = std::sqrt(u_*u_ + v_*v_);
                of_dis[ObjIdNew[i][j]].x = std::abs(u_);
                of_dis[ObjIdNew[i][j]].y = std::abs(v_);
                of_gt[ObjIdNew[i][j]](0) = u - mLastFrame.mvObjKeys[ObjIdNew[i][j]].pt.x;
                of_gt[ObjIdNew[i][j]](1) = v - mLastFrame.mvObjKeys[ObjIdNew[i][j]].pt.y;

                // // Statistics of flow normalization
                {
                    if (0.0<=ofe && ofe<0.1)
                        of_range[0] = of_range[0] + 1;
                    else if (0.1<=ofe && ofe<0.2)
                        of_range[1] = of_range[1] + 1;
                    else if (0.2<=ofe && ofe<0.3)
                        of_range[2] = of_range[2] + 1;
                    else if (0.3<=ofe && ofe<0.4)
                        of_range[3] = of_range[3] + 1;
                    else if (0.4<=ofe && ofe<0.5)
                        of_range[4] = of_range[4] + 1;
                    else if (0.5<=ofe && ofe<0.6)
                        of_range[5] = of_range[5] + 1;
                    else if (0.6<=ofe && ofe<0.7)
                        of_range[6] = of_range[6] + 1;
                    else if (0.7<=ofe && ofe<0.8)
                        of_range[7] = of_range[7] + 1;
                    else if (0.8<=ofe && ofe<0.9)
                        of_range[8] = of_range[8] + 1;
                    else if (0.9<=ofe && ofe<1.0)
                        of_range[9] = of_range[9] + 1;
                    else if (1.0<=ofe && ofe<1.1)
                        of_range[10] = of_range[10] + 1;
                    else if (1.1<=ofe && ofe<1.2)
                        of_range[11] = of_range[11] + 1;
                    else if (1.2<=ofe && ofe<1.3)
                        of_range[12] = of_range[12] + 1;
                    else if (1.3<=ofe && ofe<1.4)
                        of_range[13] = of_range[13] + 1;
                    else if (1.4<=ofe && ofe<1.5)
                        of_range[14] = of_range[14] + 1;
                    else if (1.5<=ofe && ofe<1.6)
                        of_range[15] = of_range[15] + 1;
                    else if (1.6<=ofe && ofe<1.7)
                        of_range[16] = of_range[16] + 1;
                    else if (1.7<=ofe && ofe<1.8)
                        of_range[17] = of_range[17] + 1;
                    else if (1.8<=ofe && ofe<1.9)
                        of_range[18] = of_range[18] + 1;
                    else if (1.9<=ofe)
                        of_range[19] = of_range[19] + 1;
                }
                // // Statistics of flow x
                // {
                //     if (0.0<=std::abs(u_) && std::abs(u_)<0.1)
                //         of_range_x[0] = of_range_x[0] + 1;
                //     else if (0.1<=std::abs(u_) && std::abs(u_)<0.2)
                //         of_range_x[1] = of_range_x[1] + 1;
                //     else if (0.2<=std::abs(u_) && std::abs(u_)<0.3)
                //         of_range_x[2] = of_range_x[2] + 1;
                //     else if (0.3<=std::abs(u_) && std::abs(u_)<0.4)
                //         of_range_x[3] = of_range_x[3] + 1;
                //     else if (0.4<=std::abs(u_) && std::abs(u_)<0.5)
                //         of_range_x[4] = of_range_x[4] + 1;
                //     else if (0.5<=std::abs(u_) && std::abs(u_)<0.6)
                //         of_range_x[5] = of_range_x[5] + 1;
                //     else if (0.6<=std::abs(u_) && std::abs(u_)<0.7)
                //         of_range_x[6] = of_range_x[6] + 1;
                //     else if (0.7<=std::abs(u_) && std::abs(u_)<0.8)
                //         of_range_x[7] = of_range_x[7] + 1;
                //     else if (0.8<=std::abs(u_) && std::abs(u_)<0.9)
                //         of_range_x[8] = of_range_x[8] + 1;
                //     else if (0.9<=std::abs(u_) && std::abs(u_)<1.0)
                //         of_range_x[9] = of_range_x[9] + 1;
                //     else if (1.0<=std::abs(u_) && std::abs(u_)<1.1)
                //         of_range_x[10] = of_range_x[10] + 1;
                //     else if (1.1<=std::abs(u_) && std::abs(u_)<1.2)
                //         of_range_x[11] = of_range_x[11] + 1;
                //     else if (1.2<=std::abs(u_) && std::abs(u_)<1.3)
                //         of_range_x[12] = of_range_x[12] + 1;
                //     else if (1.3<=std::abs(u_) && std::abs(u_)<1.4)
                //         of_range_x[13] = of_range_x[13] + 1;
                //     else if (1.4<=std::abs(u_) && std::abs(u_)<1.5)
                //         of_range_x[14] = of_range_x[14] + 1;
                //     else if (1.5<=std::abs(u_) && std::abs(u_)<1.6)
                //         of_range_x[15] = of_range_x[15] + 1;
                //     else if (1.6<=std::abs(u_) && std::abs(u_)<1.7)
                //         of_range_x[16] = of_range_x[16] + 1;
                //     else if (1.7<=std::abs(u_) && std::abs(u_)<1.8)
                //         of_range_x[17] = of_range_x[17] + 1;
                //     else if (1.8<=std::abs(u_) && std::abs(u_)<1.9)
                //         of_range_x[18] = of_range_x[18] + 1;
                //     else if (1.9<=std::abs(u_) && std::abs(u_)<2.0)
                //         of_range_x[19] = of_range_x[19] + 1;
                // }
                // // Statistics of flow y
                // {
                //     if (0.0<=std::abs(v_) && std::abs(v_)<0.025)
                //         of_range_y[0] = of_range_y[0] + 1;
                //     else if (0.025<=std::abs(v_) && std::abs(v_)<0.05)
                //         of_range_y[1] = of_range_y[1] + 1;
                //     else if (0.05<=std::abs(v_) && std::abs(v_)<0.075)
                //         of_range_y[2] = of_range_y[2] + 1;
                //     else if (0.075<=std::abs(v_) && std::abs(v_)<0.1)
                //         of_range_y[3] = of_range_y[3] + 1;
                //     else if (0.1<=std::abs(v_) && std::abs(v_)<0.125)
                //         of_range_y[4] = of_range_y[4] + 1;
                //     else if (0.125<=std::abs(v_) && std::abs(v_)<0.15)
                //         of_range_y[5] = of_range_y[5] + 1;
                //     else if (0.15<=std::abs(v_) && std::abs(v_)<0.175)
                //         of_range_y[6] = of_range_y[6] + 1;
                //     else if (0.175<=std::abs(v_) && std::abs(v_)<0.2)
                //         of_range_y[7] = of_range_y[7] + 1;
                //     else if (0.2<=std::abs(v_) && std::abs(v_)<0.225)
                //         of_range_y[8] = of_range_y[8] + 1;
                //     else if (0.225<=std::abs(v_) && std::abs(v_)<0.25)
                //         of_range_y[9] = of_range_y[9] + 1;
                //     else if (0.25<=std::abs(v_) && std::abs(v_)<0.275)
                //         of_range_y[10] = of_range_y[10] + 1;
                //     else if (0.275<=std::abs(v_) && std::abs(v_)<0.3)
                //         of_range_y[11] = of_range_y[11] + 1;
                //     else if (0.3<=std::abs(v_) && std::abs(v_)<0.325)
                //         of_range_y[12] = of_range_y[12] + 1;
                //     else if (0.325<=std::abs(v_) && std::abs(v_)<0.35)
                //         of_range_y[13] = of_range_y[13] + 1;
                //     else if (0.35<=std::abs(v_) && std::abs(v_)<0.375)
                //         of_range_y[14] = of_range_y[14] + 1;
                //     else if (0.375<=std::abs(v_) && std::abs(v_)<0.4)
                //         of_range_y[15] = of_range_y[15] + 1;
                //     else if (0.4<=std::abs(v_) && std::abs(v_)<0.425)
                //         of_range_y[16] = of_range_y[16] + 1;
                //     else if (0.425<=std::abs(v_) && std::abs(v_)<0.45)
                //         of_range_y[17] = of_range_y[17] + 1;
                //     else if (0.45<=std::abs(v_) && std::abs(v_)<0.475)
                //         of_range_y[18] = of_range_y[18] + 1;
                //     else if (0.475<=std::abs(v_) && std::abs(v_)<0.5)
                //         of_range_y[19] = of_range_y[19] + 1;
                // }

                // get point distance between gt and est
                // cv::Mat x3D_p_noise = mLastFrame.UnprojectStereoObjectNoise(ObjIdNew[i][j],of_dis[ObjIdNew[i][j]]);
                // cv::Mat x3D_c_est_noise = R*x3D_p_noise+t;
                // point_dis[ObjIdNew[i][j]] = std::sqrt( (x3D_c_est.at<float>(0)-x3D_c_est_noise.at<float>(0))*(x3D_c_est.at<float>(0)-x3D_c_est_noise.at<float>(0)) + (x3D_c_est.at<float>(1)-x3D_c_est_noise.at<float>(1))*(x3D_c_est.at<float>(1)-x3D_c_est_noise.at<float>(1)) + (x3D_c_est.at<float>(2)-x3D_c_est_noise.at<float>(2))*(x3D_c_est.at<float>(2)-x3D_c_est_noise.at<float>(2)) );

                // save index of the input for optimization
                ObjIdTest.push_back(ObjIdNew[i][j]);
                avg_of = avg_of + ofe;
                avg_of_x = avg_of_x + std::abs(u_);
                avg_of_y = avg_of_y + std::abs(v_);

            }
            mCurrentFrame.vObjCentre3D[i] = ObjCen3D/ObjIdNew[i].size();
            // cout << "average optical flow error: " << avg_of/ObjIdTest.size() << "/" << avg_of_x/ObjIdTest.size() << "/" << avg_of_y/ObjIdTest.size() << "/" << ObjIdTest.size() << endl;

            mCurrentFrame.vnObjID[i] = ObjIdTest;

            // cout << "object optical flow distribution: " << endl;
            // for (int j = 0; j < of_range.size(); ++j)
            //     cout << of_range[j] << " ";
            // cout << endl;

            // ******************** Construct Grid and Bounding Box ***************************

            // extend to avoid boundary error
            x_min = x_min-1;
            x_max = x_max+1;
            y_min = y_min-1;
            y_max = y_max+1;

            // // draw the grid
            // for(int j=0;j<=div_x;j++)
            //     cv::line(mImGray,cv::Point2f(x_min+j*x_step,y_min),cv::Point2f(x_min+j*x_step,y_max),cv::Scalar(0,255,0));
            // for(int j=0;j<=div_y;j++)
            //     cv::line(mImGray,cv::Point2f(x_min,y_min+j*y_step),cv::Point2f(x_max,y_min+j*y_step),cv::Scalar(0,255,0));
            // cv::drawKeypoints(mImGray, CurKeys, mImGray, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
            // cv::imshow("KeyPoints and Grid on Vehicle", mImGray);
            // cv::waitKey(0);

            // (2) keep sparsity fixed, increase number. (5,10,20,40,80,120,160,200,240)
            std::vector<int> ObjIdTest_in; //  = ObjIdTest
            cv::Point2f flo_mea(0.0,0.0), flo_cov(0.0,0.0);

            // ******* Get initial model and inlier set using PnP RanSac ********
            // ******************************************************************
            mCurrentFrame.mInitModel = GetInitModelObj(ObjIdTest,ObjIdTest_in,i);
            // cv::Mat H_tmp = InvMatrix(mCurrentFrame.mTcw_gt)*mCurrentFrame.mInitModel;
            // cout << "Initial motion estimation: " << endl << H_tmp << endl;
            // cout << "Initial motion estimation: " << endl << mInitModel << endl;

            // cout << "number of pick points: " << ObjIdTest_in.size() << "/" << ObjIdTest.size() << "/" << mCurrentFrame.mvObjKeys.size() << endl;

            // flo_mea = flo_mea/(float)ObjIdTest_in.size();
            // float point_error_mean = 0;
            cv::Mat ObjCentre3D_pre = (cv::Mat_<float>(3,1) << 0.f, 0.f, 0.f);
            std::vector<Eigen::Vector2d> of_gt_in(ObjIdTest_in.size());
            std::vector<double> e_bef(ObjIdTest_in.size());
            for (int j = 0; j < ObjIdTest_in.size(); ++j)
            {

                // compute object center 3D
                cv::Mat x3D_p = mLastFrame.UnprojectStereoObject(ObjIdTest_in[j],1);
                ObjCentre3D_pre = ObjCentre3D_pre + x3D_p;
                // point_error_mean = point_error_mean + point_dis[ObjIdTest_in[j]];
                // const float tmp_x = (of_dis[ObjIdTest_in[j]].x - flo_mea.x)*(of_dis[ObjIdTest_in[j]].x - flo_mea.x);
                // const float tmp_y = (of_dis[ObjIdTest_in[j]].y - flo_mea.y)*(of_dis[ObjIdTest_in[j]].y - flo_mea.y);
                e_bef[j] = std::sqrt((of_dis[ObjIdTest_in[j]].x*of_dis[ObjIdTest_in[j]].x) + (of_dis[ObjIdTest_in[j]].y*of_dis[ObjIdTest_in[j]].y));
                // flo_cov.x = flo_cov.x + tmp_x;
                // flo_cov.y = flo_cov.y + tmp_y;
                of_gt_in[j] = of_gt[ObjIdTest_in[j]];
            }
            ObjCentre3D_pre = ObjCentre3D_pre/ObjIdTest_in.size();
            // flo_cov = flo_cov/(float)ObjIdTest_in.size();
            // point_error_mean = point_error_mean/(float)ObjIdTest_in.size();
            // cout << "mean 3D point error: " << point_error_mean << endl;
            // cout << "mean and variance: " << flo_mea.x << " " << flo_mea.y << " " << flo_cov.x << " " << flo_cov.y << endl;

            // // **** show the picked points ****
            // std::vector<cv::KeyPoint> PickKeys;
            // for (int j = 0; j < ObjIdTest_in.size(); ++j){
            //     // PickKeys.push_back(mCurrentFrame.mvSiftKeys[ObjIdTest[j]]);
            //     PickKeys.push_back(mCurrentFrame.mvObjKeys[ObjIdTest_in[j]]);
            // }
            // cv::drawKeypoints(mImGray, PickKeys, mImGray, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
            // cv::imshow("KeyPoints and Grid on Vehicle", mImGray);
            // cv::waitKey(0);


            // ********************************************************************************

            // // // // image show the matching on each object
            // for (int j = 0; j < ObjIdTest.size(); ++j)
            // {
            //     // save key points for visualization
            //     PreKeys.push_back(mLastFrame.mvObjKeys[ObjIdTest[j]]);
            //     CurKeys.push_back(mCurrentFrame.mvObjKeys[ObjIdTest[j]]);
            //     TMes.push_back(cv::DMatch(count,count,0));
            //     count = count + 1;
            // }
            // cout << "count count: " << count << endl;
            // cv::Mat img_matches;
            // drawMatches(mImGrayLast, PreKeys, mImGray, CurKeys,
            //             TMes, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
            //             vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
            // cv::resize(img_matches, img_matches, cv::Size(img_matches.cols/1.0, img_matches.rows/1.0));
            // cv::namedWindow("temperal matches", cv::WINDOW_NORMAL);
            // cv::imshow("temperal matches", img_matches);
            // cv::waitKey(0);


            // // save object motion and label
            std::vector<int> InlierID;
            // mCurrentFrame.vObjMod[i] = GetObjMod(TemperalMatch,ObjId[i]);
            // cv::Mat Obj_X_tmp = Optimizer::PoseOptimizationObj(&mCurrentFrame,&mLastFrame,TemperalMatch,ObjIdNew[i],repro_e[i]);
            // cv::Mat Obj_X_tmp = Optimizer::PoseOptimizationObj(&mCurrentFrame,&mLastFrame,TemperalMatch,ObjIdTest,repro_e[i]);
            // cv::Mat Obj_X_tmp = Optimizer::PoseOptimizationObjTest(&mCurrentFrame,&mLastFrame,ObjIdTest_in);
            // cv::Mat Obj_X_tmp = Optimizer::PoseOptimizationForBack(&mCurrentFrame,&mLastFrame,ObjIdTest_in);
            // cv::Mat Obj_X_tmp = Optimizer::PoseOptimizationObjMotTLS(&mCurrentFrame,&mLastFrame,ObjIdTest_in);
            // cv::Mat Obj_X_tmp = Optimizer::PoseOptimizationFlowDepth(&mCurrentFrame,&mLastFrame,ObjIdTest_in);
            // cv::Mat Obj_X_tmp = Optimizer::PoseOptimizationFlowDepth2(&mCurrentFrame,&mLastFrame,ObjIdTest_in);
            // cv::Mat Obj_X_tmp = Optimizer::PoseOptimizationFlowDepth3(&mCurrentFrame,&mLastFrame,ObjIdTest_in);
            // cv::Mat Obj_X_tmp = Optimizer::PoseOptimizationDepth(&mCurrentFrame,&mLastFrame,ObjIdTest_in);
            // cv::Mat Obj_X_tmp = Optimizer::PoseOptimizationFlow(&mCurrentFrame,&mLastFrame,ObjIdTest_in);
            // cv::Mat Obj_X_tmp = Optimizer::PoseOptimizationFlow2RanSac(&mCurrentFrame,&mLastFrame,ObjIdTest_in,of_gt_in,e_bef);
            cv::Mat Obj_X_tmp = Optimizer::PoseOptimizationFlow2(&mCurrentFrame,&mLastFrame,ObjIdTest_in,of_gt_in,e_bef,InlierID);
            mCurrentFrame.vObjMod[i] = InvMatrix(mCurrentFrame.mTcw)*Obj_X_tmp;
            // mCurrentFrame.vObjMod[i] = Optimizer::PoseOptimizationObjMot(&mCurrentFrame,&mLastFrame,ObjIdTest_in,flo_cov);

            mCurrentFrame.vnObjInlierID[i] = InlierID;

            // cout << "computed motion of object No. " << mCurrentFrame.nSemPosition[i] << " :" << endl;
            // cout << mCurrentFrame.vObjMod[i] << endl;

            // ***********************************************************************************************

            // // ***** get the ground truth object speed here *****
            cv::Mat sp_gt_v;
            sp_gt_v = H_p_c.rowRange(0,3).col(3) - (cv::Mat::eye(3,3,CV_32F)-H_p_c.rowRange(0,3).colRange(0,3))*ObjCentre3D_pre; // L_w_p.rowRange(0,3).col(3) or ObjCentre3D_pre
            float sp_gt_norm = std::sqrt( sp_gt_v.at<float>(0)*sp_gt_v.at<float>(0) + sp_gt_v.at<float>(1)*sp_gt_v.at<float>(1) + sp_gt_v.at<float>(2)*sp_gt_v.at<float>(2) );
            // mCurrentFrame.vObjSpeed_gt[i] = sp_gt_norm*36;
            cv::Mat sp_gt_v2;
            sp_gt_v2 = L_w_p.rowRange(0,3).col(3) - L_w_c.rowRange(0,3).col(3);
            mCurrentFrame.vObjSpeed_gt[i] = std::sqrt( sp_gt_v2.at<float>(0)*sp_gt_v2.at<float>(0) + sp_gt_v2.at<float>(1)*sp_gt_v2.at<float>(1) + sp_gt_v2.at<float>(2)*sp_gt_v2.at<float>(2) )*36;

            // // ***** calculate the estimated object speed *****
            cv::Mat sp_est_v;
            sp_est_v = mCurrentFrame.vObjMod[i].rowRange(0,3).col(3) - (cv::Mat::eye(3,3,CV_32F)-mCurrentFrame.vObjMod[i].rowRange(0,3).colRange(0,3))*ObjCentre3D_pre;
            float sp_est_norm = std::sqrt( sp_est_v.at<float>(0)*sp_est_v.at<float>(0) + sp_est_v.at<float>(1)*sp_est_v.at<float>(1) + sp_est_v.at<float>(2)*sp_est_v.at<float>(2) );

            cout << "estimated and ground truth object speed: " << sp_est_norm*36 << "km/h " << sp_gt_norm*36 << "km/h" << endl;
            // // **** final speed error ****
            // cv::Mat sp_dis = sp_gt_v - sp_est_v;
            // float sp_dis_norm = std::sqrt( sp_dis.at<float>(0)*sp_dis.at<float>(0) + sp_dis.at<float>(1)*sp_dis.at<float>(1) + sp_dis.at<float>(2)*sp_dis.at<float>(2) );
            float sp_dis_norm = std::abs(sp_est_norm-sp_gt_norm);

            mCurrentFrame.vSpeed[i].x = sp_est_norm*36;
            mCurrentFrame.vSpeed[i].y = sp_gt_norm*36;


            // // ************** calculate the relative pose error *****************
            // // ******************************************************************

            // Errors are measured in percent (for translation) and in degrees per meter (for rotation)

            // (1) old proposed metric
            cv::Mat ObjMot_inv = InvMatrix(mCurrentFrame.vObjMod[i]);
            cv::Mat RePoEr = ObjMot_inv*H_p_c;

            // (2) Mina's proposed metric
            // cv::Mat L_w_c_est = mCurrentFrame.vObjMod[i]*L_w_p;
            // cv::Mat L_w_c_est_inv = InvMatrix(L_w_c_est);
            // cv::Mat RePoEr = L_w_c_est_inv*L_w_c;

            // (3) Viorela's proposed metric
            // cv::Mat H_p_c_body = L_w_p_inv*L_w_c;
            // cv::Mat H_p_c_est_inv = InvMatrix(mCurrentFrame.vObjMod[i]);
            // cv::Mat H_p_c_body_est_inv = L_w_p_inv*H_p_c_est_inv*L_w_p;
            // cv::Mat RePoEr = H_p_c_body_est_inv*H_p_c_body;

            // (4) Metric on body-fixed
            // cv::Mat H_p_c_body = L_w_p_inv*L_w_c;
            // cv::Mat H_p_c_body_est_inv = InvMatrix(mCurrentFrame.vObjMod[i]);
            // cv::Mat RePoEr = H_p_c_body_est_inv*H_p_c_body;

            float t_rpe = std::sqrt( RePoEr.at<float>(0,3)*RePoEr.at<float>(0,3) + RePoEr.at<float>(1,3)*RePoEr.at<float>(1,3) + RePoEr.at<float>(2,3)*RePoEr.at<float>(2,3) );
            // float trace_rpe = RePoEr.at<float>(0,0) + RePoEr.at<float>(1,1) + RePoEr.at<float>(2,2);
            float trace_rpe = 0;
            for (int i = 0; i < 3; ++i)
            {
                if (RePoEr.at<float>(i,i)>1.0)
                     trace_rpe = trace_rpe + 1.0-(RePoEr.at<float>(i,i)-1.0);
                else
                    trace_rpe = trace_rpe + RePoEr.at<float>(i,i);
            }
            float r_rpe = acos( ( trace_rpe -1.0 )/2.0 )*180.0/3.1415926;

            float t_gt = std::sqrt( H_p_c.at<float>(0,3)*H_p_c.at<float>(0,3) + H_p_c.at<float>(1,3)*H_p_c.at<float>(1,3) + H_p_c.at<float>(2,3)*H_p_c.at<float>(2,3) );
            // float t_gt = std::sqrt( H_p_c_body.at<float>(0,3)*H_p_c_body.at<float>(0,3) + H_p_c_body.at<float>(1,3)*H_p_c_body.at<float>(1,3) + H_p_c_body.at<float>(2,3)*H_p_c_body.at<float>(2,3) );
            // float trace_gt = L_w_c.at<float>(0,0) + L_w_c.at<float>(1,1) + L_w_c.at<float>(2,2);
            // float r_gt = acos( ( trace_gt -1.0 )/2.0 )*180.0/3.1415926;

            cout << "the relative pose error of the object, " << "t: " << (t_rpe/t_gt)*100 << "%" << " R: " << r_rpe/t_gt << "deg/m" << endl;
            cout << "the relative pose error of the object, " << "t: " << t_rpe <<  " R: " << r_rpe << endl;
            cout << "the object speed error, " << "s: " << sp_dis_norm/sp_gt_norm*100 << "%" << endl;

            vObjMotID.push_back(mCurrentFrame.nSemPosition[i]);
            vObjMotErr_1.push_back(cv::Point2f(t_rpe,r_rpe));
            vObjMotErr_2.push_back(cv::Point2f(t_rpe/t_gt,r_rpe/t_gt));
            vObjMotErr_3.push_back(cv::Point2f(sp_dis_norm/sp_gt_norm,sp_gt_norm*36));


            // // **************************************************************************
            // *****************************************************************************
        }

        // ****** Renew Current frame information *******
        RenewFrameInfo(TemperalMatch_subset);

        // // ====== Update from current to last frames ======
        mvKeysLastFrame = mLastFrame.mvSiftKeys;  // new added (1st Dec)  mvSiftKeys <-> mvKeys
        mvKeysCurrentFrame = mCurrentFrame.mvSiftKeys; // new added (12th Sep)

        mLastFrame = Frame(mCurrentFrame);  // this is very important!!!
        // mLastFrame.mvObjKeys = mvTmpObjKeys;  // new added Jul 30 2019
        // mLastFrame.mvObjDepth = mvTmpObjDepth;  // new added Jul 30 2019
        // mLastFrame.vSemObjLabel = mvTmpSemObjLabel; // new added Aug 2 2019
        mLastFrame.mvObjKeys = mCurrentFrame.mvObjKeys;  // new added Nov 19 2019
        mLastFrame.mvObjDepth = mCurrentFrame.mvObjDepth;  // new added Nov 19 2019
        mLastFrame.vSemObjLabel = mCurrentFrame.vSemObjLabel; // new added Nov 19 2019

        mLastFrame.mvSiftKeys = mCurrentFrame.mvSiftKeysTmp; // new added Jul 30 2019
        mLastFrame.mvSiftDepth = mCurrentFrame.mvSiftDepthTmp;  // new added Jul 30 2019

        // *****************************************************************
        // ********* save some stuffs for showing object results. **********
        // *****************************************************************

        mpMap->vvObjMotID.push_back(vObjMotID);
        mpMap->vvObjMotErr_1.push_back(vObjMotErr_1);
        mpMap->vvObjMotErr_2.push_back(vObjMotErr_2);
        mpMap->vvObjMotErr_3.push_back(vObjMotErr_3);

        // **********************************************************
        // ********* save some stuffs for graph structure. **********
        // **********************************************************

        // (1) detected static features, corresponding depth and associations
        mpMap->vpFeatSta.push_back(mCurrentFrame.mvSiftKeysTmp);
        mpMap->vfDepSta.push_back(mCurrentFrame.mvSiftDepthTmp);
        mpMap->vp3DPointSta.push_back(mCurrentFrame.mvSift3DPointTmp);  // (new added Dec 12 2019)
        mpMap->vnAssoSta.push_back(mCurrentFrame.nStaInlierID);         // (new added Nov 14 2019)
        // ORBmatcher matcher(0.9,true);
        // mpMap->vnAssoSta.push_back(matcher.SearchByOF(mCurrentFrame.mvSiftKeys,mCurrentFrame.mvSiftKeysTmp));

        // (2) detected dynamic object features, corresponding depth and associations
        // std::vector<cv::KeyPoint> FeatDynObj;
        // std::vector<float> DepDynObj;
        // std::vector<int> FeatLabObj;
        // StackObjInfo(FeatDynObj,DepDynObj,FeatLabObj);
        // mpMap->vpFeatDyn.push_back(FeatDynObj);
        // mpMap->vfDepDyn.push_back(DepDynObj);
        // mpMap->vnFeatLabel.push_back(FeatLabObj);
        mpMap->vpFeatDyn.push_back(mCurrentFrame.mvObjKeys);           // (new added Nov 20 2019)
        mpMap->vfDepDyn.push_back(mCurrentFrame.mvObjDepth);           // (new added Nov 20 2019)
        mpMap->vp3DPointDyn.push_back(mCurrentFrame.mvObj3DPoint);     // (new added Dec 12 2019)
        mpMap->vnAssoDyn.push_back(mCurrentFrame.nDynInlierID);        // (new added Nov 20 2019)
        mpMap->vnFeatLabel.push_back(mCurrentFrame.vObjLabel);         // (new added Nov 20 2019)

        // (3) save static feature tracklets
        mpMap->TrackletSta = GetStaticTrack();
        // (4) save dynamic feature tracklets
        // mpMap->TrackletDyn = GetDynamicTrack();
        mpMap->TrackletDyn = GetDynamicTrackNew();  // (new added Nov 20 2019)

        // (5) camera pose
        cv::Mat CameraPoseTmp = InvMatrix(mCurrentFrame.mTcw);
        mpMap->vmCameraPose.push_back(CameraPoseTmp);
        mpMap->vmCameraPose_RF.push_back(CameraPoseTmp);
        // (6) Rigid motions and label, including camera (label=0) and objects (label>0)
        std::vector<cv::Mat> Mot_Tmp;
        std::vector<int> Mot_Lab_Tmp;
        // (6.1) Save Camera Motion and Label
        cv::Mat CameraMotionTmp = InvMatrix(mVelocity);
        Mot_Tmp.push_back(CameraMotionTmp);
        Mot_Lab_Tmp.push_back(0);
        // (6.2) Save Object Motions and Label
        for (int i = 0; i < mCurrentFrame.vObjMod.size(); ++i)
        {
            Mot_Tmp.push_back(mCurrentFrame.vObjMod[i]);
            Mot_Lab_Tmp.push_back(mCurrentFrame.nModLabel[i]);
        }
        // (6.3) Save to The Map
        mpMap->vmRigidMotion.push_back(Mot_Tmp);
        mpMap->vmRigidMotion_RF.push_back(Mot_Tmp);
        mpMap->vnRMLabel.push_back(Mot_Lab_Tmp);

        // -------------------- Ground Truth ---------------------

        // (7) Ground Truth Camera Pose
        cv::Mat CameraPoseTmpGT = InvMatrix(mCurrentFrame.mTcw_gt);
        mpMap->vmCameraPose_GT.push_back(CameraPoseTmpGT);

        // (8) Ground Truth Rigid Motions
        std::vector<cv::Mat> Mot_Tmp_gt;
        // (8.1) Save Camera Motion
        cv::Mat CameraMotionTmp_gt = mLastFrame.mTcw_gt*InvMatrix(mCurrentFrame.mTcw_gt);
        Mot_Tmp_gt.push_back(CameraMotionTmp_gt);
        // (8.2) Save Object Motions
        for (int i = 0; i < mCurrentFrame.vObjMod_gt.size(); ++i)
        {
            Mot_Tmp_gt.push_back(mCurrentFrame.vObjMod_gt[i]);
        }
        // (8.3) Save to The Map
        mpMap->vmRigidMotion_GT.push_back(Mot_Tmp_gt);

        // (9) Ground Truth Camera and Object Speeds
        std::vector<float> Speed_Tmp_gt;
        // (9.1) Save Camera Speed
        Speed_Tmp_gt.push_back(1.0);
        // (9.2) Save Object Motions
        for (int i = 0; i < mCurrentFrame.vObjSpeed_gt.size(); ++i)
        {
            Speed_Tmp_gt.push_back(mCurrentFrame.vObjSpeed_gt[i]);
        }
        // (9.3) Save to The Map
        mpMap->vfAllSpeed_GT.push_back(Speed_Tmp_gt);

        // (10) Computed Camera and Object Speeds
        std::vector<cv::Mat> Centre_Tmp;
        // (10.1) Save Camera Speed
        cv::Mat CameraCentre = (cv::Mat_<float>(3,1) << 0.f, 0.f, 0.f);
        Centre_Tmp.push_back(CameraCentre);
        // (10.2) Save Object Motions
        for (int i = 0; i < mCurrentFrame.vObjCentre3D.size(); ++i)
        {
            Centre_Tmp.push_back(mCurrentFrame.vObjCentre3D[i]);
        }
        // (10.3) Save to The Map
        mpMap->vmRigidCentre.push_back(Centre_Tmp);


        // ---------------------------------------------------------------------------------------
        // ---------------------------------------------------------------------------------------
        // ---------------------------------------------------------------------------------------
    }

    // =================================================================================================
    // ============== Partial batch optimize on all the measurements (local optimization) ==============
    // =================================================================================================

    int WINDOW_SIZE = 10, OVERLAP_SIZE = 1;
    if ( (f_id-OVERLAP_SIZE+1)%(WINDOW_SIZE-OVERLAP_SIZE)==0 && f_id>=WINDOW_SIZE-1)
    {
        cout << "-------------------------------------------" << endl;
        cout << "! ! ! ! Partial Batch Optimization ! ! ! ! " << endl;
        cout << "-------------------------------------------" << endl;
        // Get Partial Batch Optimization
        Optimizer::PartialBatchOptimization(mpMap,mK,WINDOW_SIZE);
    }

    // =================================================================================================
    // ============== Full batch optimize on all the measurements (global optimization) ================
    // =================================================================================================

    if (f_id==70) // bFrame2Frame f_id>=2
    {
        // Metric Error BEFORE Optimization
        GetMetricError(mpMap->vmCameraPose,mpMap->vmRigidMotion,mpMap->vmCameraPose_GT,mpMap->vmRigidMotion_GT);

        // Get Full Batch Optimization
        Optimizer::FullBatchOptimization(mpMap,mK);

        // Metric Error AFTER Optimization
        GetMetricError(mpMap->vmCameraPose_RF,mpMap->vmRigidMotion_RF,mpMap->vmCameraPose_GT,mpMap->vmRigidMotion_GT);
    }


    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if(!mCurrentFrame.mTcw.empty())
    {
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState==LOST);
    }
    else
    {
        // This can happen if tracking is lost
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }

}


void Tracking::StereoInitialization()
{
    cout << "Initialization ........" << endl;

    // initialize frame id
    f_id = 0;

    // initialize the 3d points
    {
        // static
        std::vector<cv::Mat> mv3DPointTmp;
        for (int i = 0; i < mCurrentFrame.mvSiftKeysTmp.size(); ++i)
        {
            mv3DPointTmp.push_back(Optimizer::Get3DinCamera(mCurrentFrame.mvSiftKeysTmp[i], mCurrentFrame.mvSiftDepthTmp[i], mK));
        }
        mCurrentFrame.mvSift3DPointTmp = mv3DPointTmp;
        // dynamic
        std::vector<cv::Mat> mvObj3DPointTmp;
        for (int i = 0; i < mCurrentFrame.mvObjKeys.size(); ++i)
        {
            mvObj3DPointTmp.push_back(Optimizer::Get3DinCamera(mCurrentFrame.mvObjKeys[i], mCurrentFrame.mvObjDepth[i], mK));
        }
        mCurrentFrame.mvObj3DPoint = mvObj3DPointTmp;
        // cout << "see the size 1: " << mCurrentFrame.mvSiftKeysTmp.size() << " " << mCurrentFrame.mvSift3DPoint.size() << endl;
        // cout << "see the size 2: " << mCurrentFrame.mvObjKeys.size() << " " << mCurrentFrame.mvObj3DPoint.size() << endl;
    }

    // (1) save detected static features and corresponding depth
    mpMap->vpFeatSta.push_back(mCurrentFrame.mvSiftKeysTmp);  // modified Nov 14 2019
    mpMap->vfDepSta.push_back(mCurrentFrame.mvSiftDepthTmp);  // modified Nov 14 2019
    mpMap->vp3DPointSta.push_back(mCurrentFrame.mvSift3DPointTmp);  // modified Dec 17 2019
    // (2) save detected dynamic object features and corresponding depth
    mpMap->vpFeatDyn.push_back(mCurrentFrame.mvObjKeys);  // modified Nov 19 2019
    mpMap->vfDepDyn.push_back(mCurrentFrame.mvObjDepth);  // modified Nov 19 2019
    mpMap->vp3DPointDyn.push_back(mCurrentFrame.mvObj3DPoint);  // modified Dec 17 2019
    // (3) save camera pose
    mpMap->vmCameraPose.push_back(cv::Mat::eye(4,4,CV_32F));
    mpMap->vmCameraPose_RF.push_back(cv::Mat::eye(4,4,CV_32F));
    mpMap->vmCameraPose_GT.push_back(cv::Mat::eye(4,4,CV_32F));

    if(mCurrentFrame.N>300)
    {
        // Set Frame pose to the origin
        if(false)
            mCurrentFrame.SetPose(mCurrentFrame.mTcw_gt);  // +++++  new added +++++
        else
        {
            mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));  // +++++  new added +++++
            mpMap->vmCameraPose_main.push_back(mCurrentFrame.mTcw); // added for save trajectory icra2020
            mpMap->vmCameraPose_orb.push_back(mCurrentFrame.mTcw);  // added for save trajectory icra2020

            mOriginInv = cv::Mat::eye(4,4,CV_32F);
        }
        // bFirstFrame = false;
        // cout << "current pose: " << endl << mCurrentFrame.mTcw_gt << endl;
        // cout << "current pose inverse: " << endl << mOriginInv << endl;

        // Create KeyFrame
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

        // Insert KeyFrame in the map
        mpMap->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        for(int i=0; i<mCurrentFrame.N;i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap);
                pNewMP->AddObservation(pKFini,i);
                pKFini->AddMapPoint(pNewMP,i);
                pNewMP->ComputeDistinctiveDescriptors();
                pNewMP->UpdateNormalAndDepth();
                mpMap->AddMapPoint(pNewMP);

                mCurrentFrame.mvpMapPoints[i]=pNewMP;
            }
        }

        cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;

        mpLocalMapper->InsertKeyFrame(pKFini);

        mLastFrame = Frame(mCurrentFrame);  //  important !!!
        mLastFrame.mvObjKeys = mCurrentFrame.mvObjKeys; // new added Jul 30 2019
        mLastFrame.mvObjDepth = mCurrentFrame.mvObjDepth;  // new added Jul 30 2019
        mLastFrame.vSemObjLabel = mCurrentFrame.vSemObjLabel; // new added Aug 2 2019

        mLastFrame.mvSiftKeys = mCurrentFrame.mvSiftKeysTmp; // new added Jul 30 2019
        mLastFrame.mvSiftDepth = mCurrentFrame.mvSiftDepthTmp;  // new added Jul 30 2019
        mLastFrame.N_s = mCurrentFrame.N_s_tmp; // new added Nov 14 2019
        mvKeysLastFrame = mLastFrame.mvSiftKeys; // +++ new added +++

        mnLastKeyFrameId=mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints=mpMap->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        mState=OK;
    }
}

void Tracking::MonocularInitialization()
{

    if(!mpInitializer)
    {
        // Set Reference Frame
        if(mCurrentFrame.mvKeys.size()>100)
        {
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            if(mpInitializer)
                delete mpInitializer;

            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }
    }
    else
    {
        printf("Try init..");
        // Try to initialize
        if((int)mCurrentFrame.mvKeys.size()<=100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        // Find correspondences
        printf("Try match..");
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences
        if(nmatches<100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        printf("Try tri..\n");
        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            printf("nmatches0: %d", nmatches);
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }
            printf(" .. nmatches1: %d\n", nmatches);

            // Set Frame Poses
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();
        } else {
            printf("\tFailed to init..\n");
        }
    }
}

void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);


    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpMap->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

    Optimizer::GlobalBundleAdjustemnt(mpMap,20);

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
    {
        cout << "Wrong initialization, reseting..." << endl;
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
        }
    }

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;
}

void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}

void Tracking::UpdateRefKeyFrame(vector<MapPoint*> vpMapPointsKF)
{
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mpReferenceKF->N);
    for(int i=0; i<mpReferenceKF->N;i++)
    {
        float z = mpReferenceKF->mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert 1000 points sorted by depth
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = vpMapPointsKF[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
        {
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            cv::Mat x3D = mpReferenceKF->UnprojectStereo(i);
            MapPoint* pNewMP = new MapPoint(x3D,mpReferenceKF,mpMap);

            vpMapPointsKF[i]=pNewMP;

            nPoints++;
        }

        if(nPoints>1000){  // 1000 for underwater dataset
            break;
        }
    }
    // cout << "Update reference keyframe ! ! !  " << "temporal point: " << nPoints << endl;
}


bool Tracking::TrackReferenceKeyFrame()
{
    cout << "Track With Reference Keyframe...(RF)" << endl;
    bFrame2Frame = false;
    bSecondFrame = true;
    ORBmatcher matcher(0.7,true);  // 0.7  0.9

    // vector<MapPoint*> vpMapPointsKF = mpReferenceKF->GetMapPointMatches();
    // UpdateRefKeyFrame(vpMapPointsKF);
    // int nmatches = matcher.SearchByQuadKeyFrame(mpReferenceKF,mCurrentFrame,vpMapPointsKF);

    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    vector<MapPoint*> vpMapPointMatches;
    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches,TemperalMatch);

    // cout << "initial matches: " << nmatches << endl;

    if(nmatches<15)  // 15  10
        return false;

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw);

    Optimizer::PoseOptimization(&mCurrentFrame);

    mpMap->vmCameraPose_orb.push_back(InvMatrix(mCurrentFrame.mTcw));

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                // mCurrentFrame.vObjLabel[i] = -1;
                // TemperalMatch[i]= -1;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    // cout << "final inliers: " << nmatchesMap << endl;
    return nmatchesMap>=10;
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame.SetPose(Tlr*pRef->GetPose());

    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR) // || !mbOnlyTracking
      return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    for(int i=0; i<mLastFrame.N;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
        {
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            MapPoint* pNewMP = new MapPoint(x3D,mpMap,&mLastFrame,i);

            mLastFrame.mvpMapPoints[i]=pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if(vDepthIdx[j].first>mThDepth && nPoints>200){   // 100 200
            break;
        }
    }
    // cout << "Update last frame ! ! !  " << "temporal point: " << nPoints << endl;
}

bool Tracking::TrackWithMotionModel()
{

    cout << "Track With Motion Model...(MM)" << endl;
    bFrame2Frame = true;
    bSecondFrame = false;
    ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    cv::Mat iniTcw = mVelocity*mLastFrame.mTcw;

    mCurrentFrame.SetPose(iniTcw);

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // // Project points seen in previous frame
    int th;
    if(mSensor!=System::STEREO)
        th=15;
    else
        th=7; // 7 15
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR,TemperalMatch);

    // If few matches, uses a wider window search
    if(nmatches<20)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR,TemperalMatch);
    }

    // Search matches by quad matching, i.e., matches must fit to
    // last left, right and current left, right frames simultaneously
    // clock_t start, ends;
    // double time_counter;
    // start = clock();
    // int nmatches = matcher.SearchByQuad(mCurrentFrame,mLastFrame,TemperalMatch);
    // ends = clock();
    // time_counter = (double)(ends-start)/CLOCKS_PER_SEC*1000;
    // cout << "matching time: " << time_counter << endl;

    // cout << "initial matches: " << nmatches << endl;
    // cout << nmatches << " ";

    if(nmatches<20)
        return false;

    // ************** This part carries out an initial pose estimation using P3P method *******************************************
    // double dmatch, dinlier;
    // {
    //   Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
    //   K(0,0) = mCurrentFrame.fx; K(1,1) = mCurrentFrame.fy; K(0,2) = mCurrentFrame.cx; K(1,2) = mCurrentFrame.cy; K(2,2) = 1.0;
    //   opengv::bearingVectors_t bearingVectors;
    //   std::vector<int> camCorrespondences;
    //   opengv::points_t points;
    //   opengv::translations_t camOffsets;
    //   opengv::rotations_t camRotations;

    //   // prepare the 2d and 3d measurements
    //   Eigen::Vector3d tmp;
    //   std::vector<int> indm;
    //   for(int i=0; i< mCurrentFrame.N; i++){
    //     if(mCurrentFrame.mvpMapPoints[i]){

    //         // just use the background points on semantically segmented background
    //         // if(mCurrentFrame.vSemLabel[i]!=0)
    //         //     continue;

    //         // get the 3d points
    //         cv::Mat p3d = mCurrentFrame.mvpMapPoints[i]->GetWorldPos();

    //         // convert 2d measurements to bearingvectors
    //         if(mCurrentFrame.mvuRight[i]==-1){
    //             tmp(0,0) = mCurrentFrame.mvKeysUn[i].pt.x; tmp(1,0) = mCurrentFrame.mvKeysUn[i].pt.y; tmp(2,0) = 1.0;
    //             bearingVectors.push_back(K.inverse()*tmp);
    //             camCorrespondences.push_back(0);
    //             tmp(0,0) = p3d.at<float>(0); tmp(1,0) = p3d.at<float>(1); tmp(2,0) = p3d.at<float>(2);
    //             points.push_back(tmp);
    //             indm.push_back(i);
    //         }
    //         else{
    //             tmp(0,0) = mCurrentFrame.mvKeysUn[i].pt.x; tmp(1,0) = mCurrentFrame.mvKeysUn[i].pt.y; tmp(2,0) = 1.0;
    //             bearingVectors.push_back(K.inverse()*tmp);
    //             camCorrespondences.push_back(0);
    //             tmp(0,0) = mCurrentFrame.mvKeysUn[i].pt.x; tmp(1,0) = mCurrentFrame.mvuRight[i]; tmp(2,0) = 1.0;
    //             bearingVectors.push_back(K.inverse()*tmp);
    //             camCorrespondences.push_back(1);
    //             tmp(0,0) = p3d.at<float>(0); tmp(1,0) = p3d.at<float>(1); tmp(2,0) = p3d.at<float>(2);
    //             points.push_back(tmp);
    //             points.push_back(tmp);
    //             indm.push_back(i);
    //             indm.push_back(i);
    //         }
    //     }

    //   }
    //   // normalize
    //   for(int i=0; i< bearingVectors.size(); i++){
    //       bearingVectors[i] = bearingVectors[i] / bearingVectors[i].norm();
    //   }

    //   // set the 2 cameras relationship to the viewpoint. here set the left camera as the viewpoint.
    //   camOffsets.push_back(Eigen::Vector3d::Zero());
    //   camRotations.push_back(Eigen::Matrix3d::Identity(3,3));
    //   Eigen::Vector3d camOffset; camOffset << mCurrentFrame.mb, 0.0, 0.0;
    //   camOffsets.push_back(camOffset);
    //   camRotations.push_back(Eigen::Matrix3d::Identity(3,3));

    //   //create a non-central absolute adapter
    //   opengv::absolute_pose::NoncentralAbsoluteAdapter adapter(
    //       bearingVectors,
    //       camCorrespondences,
    //       points,
    //       camOffsets,
    //       camRotations);

    //   //Create a AbsolutePoseSacProblem and Ransac
    //   //The method is set to GP3P
    //   opengv::sac::Ransac<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem> ransac;
    //   std::shared_ptr<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem> absposeproblem_ptr(
    //       new opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem(adapter,opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem::GP3P));
    //   ransac.sac_model_ = absposeproblem_ptr;
    //   ransac.threshold_ = 5e-05; // 1.0 - cos(atan(sqrt(2.0)*0.5/K(0,0)));  8e-06 9e-06 5e-05
    //   ransac.max_iterations_ = 500;
    //   ransac.computeModel();

    //   dmatch = nmatches;
    //   dinlier = ransac.inliers_.size();
    //   cout << "inlier rate: " << dinlier/dmatch << " " << dinlier << endl;

    //   // Only use the P3P result if the inlier rate > 45% or inlier number >80
    //   if (dinlier/dmatch >= 0.45 || dinlier >= 60){

    //       // Update the pose in current frame
    //       opengv::transformation_t A = ransac.model_coefficients_;
    //       Eigen::Matrix4d TwcTemperal = Eigen::Matrix4d::Identity();
    //       TwcTemperal.topLeftCorner(3,3) = A.topLeftCorner(3,3).transpose();
    //       TwcTemperal.block(0, 3, 3, 1) = -A.topLeftCorner(3,3).transpose()*A.block(0, 3, 3, 1);
    //       mCurrentFrame.mTcw.at<float>(0,0)=TwcTemperal(0,0);mCurrentFrame.mTcw.at<float>(0,1)=TwcTemperal(0,1);mCurrentFrame.mTcw.at<float>(0,2)=TwcTemperal(0,2);mCurrentFrame.mTcw.at<float>(0,3)=TwcTemperal(0,3);
    //       mCurrentFrame.mTcw.at<float>(1,0)=TwcTemperal(1,0);mCurrentFrame.mTcw.at<float>(1,1)=TwcTemperal(1,1);mCurrentFrame.mTcw.at<float>(1,2)=TwcTemperal(1,2);mCurrentFrame.mTcw.at<float>(1,3)=TwcTemperal(1,3);
    //       mCurrentFrame.mTcw.at<float>(2,0)=TwcTemperal(2,0);mCurrentFrame.mTcw.at<float>(2,1)=TwcTemperal(2,1);mCurrentFrame.mTcw.at<float>(2,2)=TwcTemperal(2,2);mCurrentFrame.mTcw.at<float>(2,3)=TwcTemperal(2,3);

    //       // Discard outliers
    //       std::vector<int> indinout(mCurrentFrame.N,0);
    //       for(int i =0; i<ransac.inliers_.size(); i++){
    //         indinout[indm[ransac.inliers_[i]]] = 1;
    //       }

    //       for(int i =0; i<mCurrentFrame.N; i++){
    //         if(mCurrentFrame.mvpMapPoints[i]){
    //             // just use the background points on semantically segmented background
    //             // if(mCurrentFrame.vSemLabel[i]!=0)
    //             //     continue;
    //             if(indinout[i]==0){
    //                 mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
    //                 mCurrentFrame.mvbOutlier[i]=false;
    //                 // TemperalMatch[i]= -1;
    //                 nmatches--;
    //             }
    //         }
    //       }

    //     }

    // }


    // **************************************************************************************************************************

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);

    mpMap->vmCameraPose_orb.push_back(InvMatrix(mCurrentFrame.mTcw));

    int count1 = 0, count2 = 0;
    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                count1 = count1 + 1;
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                // mCurrentFrame.vObjLabel[i] = -1;
                // TemperalMatch[i]= -1;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0){
                count2 = count2 + 1;
                nmatchesMap++;
            }
        }
    }

    // cout << "Outlier/inlier number after optimization: " << count1 << " / " << count2 << endl;

    if(mbOnlyTracking)
    {
        mbVO = nmatchesMap<10;
        return nmatches>20;
    }

    // Don't track local map if matches found
    // in this frame is less than 20
    mbVO = nmatchesMap<20;
    // double dinlier_final = nmatchesMap;
    // cout << "final inliers: " << dinlier_final/dinlier << " " << nmatchesMap << endl;
    // cout << "final inliers: " << nmatchesMap << endl;
    // cout << nmatchesMap << " " << dinlier_final/dmatch << endl;
    // cout << nmatchesMap << endl;

    if(nmatchesMap>=10)  // nmatchesMap>=10
        return true;
    else
        return false;

}

bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.
    // cout << "Track Local Map~ ~ ~ ~ ~ ~" << endl;

    UpdateLocalMap();

    SearchLocalPoints();

    // Optimize Pose
    Optimizer::PoseOptimization(&mCurrentFrame);
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if(!mbOnlyTracking)
                {
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            else if(mSensor==System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);

        }
    }

    // cout << "local matching inliers: " << mnMatchesInliers << endl;

    // Decide if the tracking was successful
    // More restrictive if there was a relocalization recently
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50){
        cout << "Track local map FAIL! -  - ! -  - !1" << endl;
        return false;
    }

    if(mnMatchesInliers<30){
        cout << "Track local map FAIL! -  - ! -  - !2" << endl;
        return false;
    }
    else
        return true;
}


bool Tracking::NeedNewKeyFrame()
{
    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // cout << "match in reference keyframe: " << nRefMatches << endl;

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;
    if(mSensor!=System::MONOCULAR)
    {
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
    }

    // cout << "close or nonclose: " << nTrackedClose << " " << nNonTrackedClose << endl;

    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);  // 100/250 70/50

    // Thresholds
    float thRefRatio = 0.75f;  // 0.75f  0.90f
    if(nKFs<2)
        thRefRatio = 0.4f;

    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
    // Condition 1c: tracking is weak
    const bool c1c =  mSensor!=System::MONOCULAR && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>15);  // 15

    if((c1a||c1b||c1c)&&c2)  // (c1a||c1b||c1c)&&c2
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR)
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
    if(!mpLocalMapper->SetNotStop(true))
        return;

    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    if(mSensor!=System::MONOCULAR)
    {
        mCurrentFrame.UpdatePoseMatrices();

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if(bCreateNew)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap);
                    pNewMP->AddObservation(pKF,i);
                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>200)  //  100  200
                    break;
            }
            // cout << "Create New Keyframe ! ! ! - - - ! ! ! " << endl;
            // cout << "Points in the new keyframe: " << nPoints << endl;
        }
    }


    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD)
            th=3;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }
}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}


void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }

    }

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization()
{
    cout << "relocalisation......" << endl;
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i],TemperalMatch);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void Tracking::Reset()
{

    cout << "System Reseting" << endl;
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    if(mpViewer)
        mpViewer->Release();
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}

// ---------------------------------------------------------------------------------------
// ++++++++++++++++++++++++++++++++++++++ New added ++++++++++++++++++++++++++++++++++++++
// ---------------------------------------------------------------------------------------

void Tracking::UpdatePose(const vector<int> &TemperalMatch)
{
    cv::Mat Mod = cv::Mat::eye(4,4,CV_32F);

    // construct input
    std::vector<cv::Point2f> cur_2d;
    std::vector<cv::Point3f> pre_3d;
    for (int i = 0; i < TemperalMatch.size(); ++i)
    {
        if (TemperalMatch[i]==-1)
            continue;
        // if (mCurrentFrame.vObjLabel[i]==-1)
        //     continue;

        cv::Point2f tmp_2d;
        tmp_2d.x = mCurrentFrame.mvSiftKeys[i].pt.x;
        tmp_2d.y = mCurrentFrame.mvSiftKeys[i].pt.y;
        cur_2d.push_back(tmp_2d);
        cv::Point3f tmp_3d;
        cv::Mat x3D_p = mLastFrame.UnprojectStereoSift(TemperalMatch[i],0);
        tmp_3d.x = x3D_p.at<float>(0);
        tmp_3d.y = x3D_p.at<float>(1);
        tmp_3d.z = x3D_p.at<float>(2);
        pre_3d.push_back(tmp_3d);
    }

    // camera matrix & distortion coefficients
    cv::Mat camera_mat(3, 3, CV_64FC1);
    cv::Mat distCoeffs = cv::Mat::zeros(1, 4, CV_64FC1);
    camera_mat.at<double>(0, 0) = mK.at<float>(0,0);
    camera_mat.at<double>(1, 1) = mK.at<float>(1,1);
    camera_mat.at<double>(0, 2) = mK.at<float>(0,2);
    camera_mat.at<double>(1, 2) = mK.at<float>(1,2);
    camera_mat.at<double>(2, 2) = 1.0;

    // output
    // cv::Mat Rvec = mCurrentFrame.Rvec;
    // cv::Mat Tvec = mCurrentFrame.Tvec;
    cv::Mat Rvec(3, 1, CV_64FC1);
    cv::Mat Tvec(3, 1, CV_64FC1);
    cv::Mat d(3, 3, CV_64FC1);
    cv::Mat inliers;

    // solve
    int iter_num = 500;
    double reprojectionError = 2.0, confidence = 0.98;
    cv::solvePnPRansac(pre_3d, cur_2d, camera_mat, distCoeffs, Rvec, Tvec, true,
               iter_num, reprojectionError, confidence, inliers, cv::SOLVEPNP_ITERATIVE);

    cout << "Inliers number/Total features number: " << inliers.rows << "/" << TemperalMatch.size() << endl;

    cv::Rodrigues(Rvec, d);

    // assign the result to current pose
    Mod.at<float>(0,0) = d.at<double>(0,0); Mod.at<float>(0,1) = d.at<double>(0,1); Mod.at<float>(0,2) = d.at<double>(0,2); Mod.at<float>(0,3) = Tvec.at<double>(0,0);
    Mod.at<float>(1,0) = d.at<double>(1,0); Mod.at<float>(1,1) = d.at<double>(1,1); Mod.at<float>(1,2) = d.at<double>(1,2); Mod.at<float>(1,3) = Tvec.at<double>(1,0);
    Mod.at<float>(2,0) = d.at<double>(2,0); Mod.at<float>(2,1) = d.at<double>(2,1); Mod.at<float>(2,2) = d.at<double>(2,2); Mod.at<float>(2,3) = Tvec.at<double>(2,0);

    mCurrentFrame.SetPose(Mod);

}

void Tracking::GetSceneFlowSift(const vector<int> &TemperalMatch)
{
    // Threshold
    // int max_dist = 90, max_lat = 30;
    double fps = 10, max_velocity_ms = 40;

    // Initialization
    int N = mCurrentFrame.N_s;
    mCurrentFrame.vFlow_3d.resize(N);
    mCurrentFrame.vFlow_2d.resize(N);

    const cv::Mat Rcw = mCurrentFrame.mTcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tcw = mCurrentFrame.mTcw.rowRange(0,3).col(3);

    // Main loop
    for (int i = 0; i < N; ++i)
    {
        // filter
        if(mCurrentFrame.mvSiftDepth[i]>mThDepth || mCurrentFrame.mvSiftDepth[i]<=0 || mLastFrame.mvSiftDepth[TemperalMatch[i]]>mThDepth
           || mLastFrame.mvSiftDepth[TemperalMatch[i]]<=0 || TemperalMatch[i]==-1)
        {
            mCurrentFrame.vObjLabel[i]=-1;
            // cout << "above depth threshold: " << mCurrentFrame.mvSiftDepth[i] << " " << mLastFrame.mvSiftDepth[TemperalMatch[i]] << endl;
            continue;
        }

        // get the 3d flow
        cv::Mat x3D_p = mLastFrame.UnprojectStereoSift(TemperalMatch[i],0);
        cv::Mat x3D_c = mCurrentFrame.UnprojectStereoSift(i,0);

        // cout << "3d points: " << x3D_p << " " << x3D_c << endl;

        // // threshold latitude and distance
        // if (std::fabs(x3D_c.at<float>(0)) > max_lat || std::fabs(x3D_c.at<float>(2)) > max_dist
        //     || std::fabs(x3D_p.at<float>(0)) > max_lat || std::fabs(x3D_p.at<float>(2)) > max_dist)
        // {
        //     cout << "latitude and distance (ppppp): " << std::fabs(x3D_c.at<float>(0)) << " " << std::fabs(x3D_c.at<float>(2)) << endl;
        //     cout << "latitude and distance (ccccc): " << std::fabs(x3D_p.at<float>(0)) << " " << std::fabs(x3D_p.at<float>(2)) << endl;
        //     mCurrentFrame.vObjLabel[i]=-1;
        //     continue;
        // }

        cv::Point3f flow3d;
        flow3d.x = x3D_c.at<float>(0) - x3D_p.at<float>(0);
        flow3d.y = x3D_c.at<float>(1) - x3D_p.at<float>(1);
        flow3d.z = x3D_c.at<float>(2) - x3D_p.at<float>(2);

        // cout << "3d points: " << mCurrentFrame.vFlow_3d[i] << endl;

        // threshold the velocity
        if(cv::norm(flow3d)*fps > max_velocity_ms)
        {
            mCurrentFrame.vObjLabel[i]=-1;
            continue;
        }

        mCurrentFrame.vFlow_3d[i] = flow3d;

        // get the 2D re-projection error vector
        // (1) transfer 3d from world to current frame.
        cv::Mat x3D_pc = Rcw*x3D_p+tcw;
        // (2) project 3d into current image plane
        float xc = x3D_pc.at<float>(0);
        float yc = x3D_pc.at<float>(1);
        float invzc = 1.0/x3D_pc.at<float>(2);
        float u = mCurrentFrame.fx*xc*invzc+mCurrentFrame.cx;
        float v = mCurrentFrame.fy*yc*invzc+mCurrentFrame.cy;

        mCurrentFrame.vFlow_2d[i].x = mCurrentFrame.mvSiftKeys[i].pt.x - u;
        mCurrentFrame.vFlow_2d[i].y = mCurrentFrame.mvSiftKeys[i].pt.y - v;

        // cout << "2d errors: " << mCurrentFrame.vFlow_2d[i] << endl;

    }

}

void Tracking::GetSceneFlowObj()
{
    // // Threshold // //
    // int max_dist = 90, max_lat = 30;
    // double fps = 10, max_velocity_ms = 40;
    // double max_depth = 30;

    // Initialization
    int N = mCurrentFrame.mvObjKeys.size();
    mCurrentFrame.vFlow_3d.resize(N);
    // mCurrentFrame.vFlow_2d.resize(N);

    std::vector<Eigen::Vector3d> pts_p3d(N,Eigen::Vector3d(-1,-1,-1)), pts_vel(N,Eigen::Vector3d(-1,-1,-1));

    const cv::Mat Rcw = mCurrentFrame.mTcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tcw = mCurrentFrame.mTcw.rowRange(0,3).col(3);

    // Main loop
    for (int i = 0; i < N; ++i)
    {
        // // filter
        // if(mCurrentFrame.mvObjDepth[i]>max_depth  || mLastFrame.mvObjDepth[i]>max_depth)
        // {
        //     mCurrentFrame.vObjLabel[i]=-1;
        //     continue;
        // }
        if (mCurrentFrame.vSemObjLabel[i]<=0 || mLastFrame.vSemObjLabel[i]<=0)
        {
            mCurrentFrame.vObjLabel[i]=-1;
            continue;
        }

        // get the 3d flow
        cv::Mat x3D_p = mLastFrame.UnprojectStereoObject(i,0);
        cv::Mat x3D_c = mCurrentFrame.UnprojectStereoObject(i,0);

        pts_p3d[i] << x3D_p.at<float>(0), x3D_p.at<float>(1), x3D_p.at<float>(2);

        // cout << "3d points: " << x3D_p << " " << x3D_c << endl;

        cv::Point3f flow3d;
        flow3d.x = x3D_c.at<float>(0) - x3D_p.at<float>(0);
        flow3d.y = x3D_c.at<float>(1) - x3D_p.at<float>(1);
        flow3d.z = x3D_c.at<float>(2) - x3D_p.at<float>(2);

        pts_vel[i] << flow3d.x, flow3d.y, flow3d.z;

        // cout << "3d points: " << mCurrentFrame.vFlow_3d[i] << endl;

        // // threshold the velocity
        // if(cv::norm(flow3d)*fps > max_velocity_ms)
        // {
        //     mCurrentFrame.vObjLabel[i]=-1;
        //     continue;
        // }

        mCurrentFrame.vFlow_3d[i] = flow3d;

        // // get the 2D re-projection error vector
        // // (1) transfer 3d from world to current frame.
        // cv::Mat x3D_pc = Rcw*x3D_p+tcw;
        // // (2) project 3d into current image plane
        // float xc = x3D_pc.at<float>(0);
        // float yc = x3D_pc.at<float>(1);
        // float invzc = 1.0/x3D_pc.at<float>(2);
        // float u = mCurrentFrame.fx*xc*invzc+mCurrentFrame.cx;
        // float v = mCurrentFrame.fy*yc*invzc+mCurrentFrame.cy;

        // mCurrentFrame.vFlow_2d[i].x = mCurrentFrame.mvObjKeys[i].pt.x - u;
        // mCurrentFrame.vFlow_2d[i].y = mCurrentFrame.mvObjKeys[i].pt.y - v;

        // // cout << "2d errors: " << mCurrentFrame.vFlow_2d[i] << endl;

    }

    // // // ===== show scene flow from bird eye view =====
    // cv::Mat img_sparse_flow_3d;
    // BirdEyeVizProperties viz_props;
    // viz_props.birdeye_scale_factor_ = 20.0;
    // viz_props.birdeye_left_plane_ = -15.0;
    // viz_props.birdeye_right_plane_ = 15.0;
    // viz_props.birdeye_far_plane_ = 30.0;

    // Tracking::DrawSparseFlowBirdeye(pts_p3d, pts_vel, Converter::toInvMatrix(mLastFrame.mTcw), viz_props, img_sparse_flow_3d);
    // cv::imshow("SparseFlowBirdeye", img_sparse_flow_3d*255);
    // cv::waitKey(0);
}

void Tracking::GetSceneFlow(const vector<int> &TemperalMatch)
{
    // Threshold
    int max_dist = 90, max_lat = 30;
    double fps = 10, max_velocity_ms = 40;

    // Initialization
    int N = mCurrentFrame.N;
    mCurrentFrame.vFlow_3d.resize(N);
    mCurrentFrame.vFlow_2d.resize(N);

    const cv::Mat Rcw = mCurrentFrame.mTcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tcw = mCurrentFrame.mTcw.rowRange(0,3).col(3);

    // Main loop
    // cout << "depth threshold: " << mThDepth << endl;
    for (int i = 0; i < N; ++i)
    {
        // filter
        if(mCurrentFrame.mvDepth[i]>mThDepth || mCurrentFrame.mvDepth[i]<=0 || mLastFrame.mvDepth[TemperalMatch[i]]>mThDepth
           || mLastFrame.mvDepth[TemperalMatch[i]]<=0 || TemperalMatch[i]==-1)
        {
            mCurrentFrame.vObjLabel[i]=-1;
            continue;
        }

        // get the 3d flow
        cv::Mat x3D_p = mLastFrame.UnprojectStereo(TemperalMatch[i]);
        cv::Mat x3D_c = mCurrentFrame.UnprojectStereo(i);

        // cout << "3d points: " << x3D_p << " " << x3D_c << endl;

        // threshold latitude and distance
        if (std::fabs(x3D_c.at<float>(0)) > max_lat || std::fabs(x3D_c.at<float>(2)) > max_dist
            || std::fabs(x3D_p.at<float>(0)) > max_lat || std::fabs(x3D_p.at<float>(2)) > max_dist)
        {
            // cout << "latitude and distance (ppppp): " << std::fabs(x3D_c.at<float>(0)) << " " << std::fabs(x3D_c.at<float>(2)) << endl;
            // cout << "latitude and distance (ccccc): " << std::fabs(x3D_p.at<float>(0)) << " " << std::fabs(x3D_p.at<float>(2)) << endl;
            mCurrentFrame.vObjLabel[i]=-1;
            continue;
        }

        mCurrentFrame.vFlow_3d[i].x = x3D_c.at<float>(0) - x3D_p.at<float>(0);
        mCurrentFrame.vFlow_3d[i].y = x3D_c.at<float>(1) - x3D_p.at<float>(1);
        mCurrentFrame.vFlow_3d[i].z = x3D_c.at<float>(2) - x3D_p.at<float>(2);

        // cout << "3d points: " << mCurrentFrame.vFlow_3d[i] << endl;

        // threshold the velocity
        if(cv::norm(mCurrentFrame.vFlow_3d[i])*fps > max_velocity_ms)
        {
            mCurrentFrame.vObjLabel[i]=-1;
            continue;
        }

        // get the 2D re-projection error vector
        // (1) transfer 3d from world to current frame.
        cv::Mat x3D_pc = Rcw*x3D_p+tcw;
        // (2) project 3d into current image plane
        float xc = x3D_pc.at<float>(0);
        float yc = x3D_pc.at<float>(1);
        float invzc = 1.0/x3D_pc.at<float>(2);
        float u = mCurrentFrame.fx*xc*invzc+mCurrentFrame.cx;
        float v = mCurrentFrame.fy*yc*invzc+mCurrentFrame.cy;

        mCurrentFrame.vFlow_2d[i].x = mCurrentFrame.mvKeys[i].pt.x - u;
        mCurrentFrame.vFlow_2d[i].y = mCurrentFrame.mvKeys[i].pt.y - v;

        // cout << "2d errors: " << mCurrentFrame.vFlow_2d[i] << endl;

    }

}

cv::Mat Tracking::GetObjMod(const vector<int> &TemperalMatch, const vector<int> &ObjId)
{
    cv::Mat Mod = cv::Mat::eye(4,4,CV_32F);

    // construct input
    std::vector<cv::Point2f> cur_2d;
    std::vector<cv::Point3f> pre_3d;
    for (int i = 0; i < ObjId.size(); ++i)
    {
        cv::Point2f tmp_2d;
        tmp_2d.x = mCurrentFrame.mvSiftKeys[ObjId[i]].pt.x;
        tmp_2d.y = mCurrentFrame.mvSiftKeys[ObjId[i]].pt.y;
        cur_2d.push_back(tmp_2d);
        cv::Point3f tmp_3d;
        cv::Mat x3D_p = mLastFrame.UnprojectStereoSift(TemperalMatch[ObjId[i]],0);
        tmp_3d.x = x3D_p.at<float>(0);
        tmp_3d.y = x3D_p.at<float>(1);
        tmp_3d.z = x3D_p.at<float>(2);
        pre_3d.push_back(tmp_3d);
    }

    // camera matrix & distortion coefficients
    cv::Mat camera_mat(3, 3, CV_64FC1);
    cv::Mat distCoeffs = cv::Mat::zeros(1, 4, CV_64FC1);
    camera_mat.at<double>(0, 0) = mK.at<float>(0,0);
    camera_mat.at<double>(1, 1) = mK.at<float>(1,1);
    camera_mat.at<double>(0, 2) = mK.at<float>(0,2);
    camera_mat.at<double>(1, 2) = mK.at<float>(1,2);
    camera_mat.at<double>(2, 2) = 1.0;

    // output
    cv::Mat Rvec(3, 1, CV_64FC1);
    cv::Mat Tvec(3, 1, CV_64FC1);
    cv::Mat d(3, 3, CV_64FC1);
    cv::Mat inliers;

    // solve
    int iter_num = 500;
    double reprojectionError = 2.0, confidence = 0.98;
    cv::solvePnPRansac(pre_3d, cur_2d, camera_mat, distCoeffs, Rvec, Tvec, false,
               iter_num, reprojectionError, confidence, inliers, cv::SOLVEPNP_P3P);

    cout << "inliers/total number: " << inliers.rows << "/" << ObjId.size() << endl;

    cv::Rodrigues(Rvec, d);

    // assign the result to current pose
    Mod.at<float>(0,0) = d.at<double>(0,0); Mod.at<float>(0,1) = d.at<double>(0,1); Mod.at<float>(0,2) = d.at<double>(0,2); Mod.at<float>(0,3) = Tvec.at<double>(0,0);
    Mod.at<float>(1,0) = d.at<double>(1,0); Mod.at<float>(1,1) = d.at<double>(1,1); Mod.at<float>(1,2) = d.at<double>(1,2); Mod.at<float>(1,3) = Tvec.at<double>(1,0);
    Mod.at<float>(2,0) = d.at<double>(2,0); Mod.at<float>(2,1) = d.at<double>(2,1); Mod.at<float>(2,2) = d.at<double>(2,2); Mod.at<float>(2,3) = Tvec.at<double>(2,0);

    Mod = mCurrentFrame.mTcw*Mod;

    cout << "Object Motion: " << endl << Mod << endl;

    return Mod;
}

cv::Mat Tracking::GetInitModelCam(const std::vector<int> &MatchId, std::vector<int> &MatchId_sub)
{
    cv::Mat Mod = cv::Mat::eye(4,4,CV_32F);
    int N = MatchId.size();

    // construct input
    std::vector<cv::Point2f> cur_2d(N);
    std::vector<cv::Point3f> pre_3d(N);
    for (int i = 0; i < N; ++i)
    {
        cv::Point2f tmp_2d;
        tmp_2d.x = mCurrentFrame.mvSiftKeys[MatchId[i]].pt.x;
        tmp_2d.y = mCurrentFrame.mvSiftKeys[MatchId[i]].pt.y;
        cur_2d[i] = tmp_2d;
        cv::Point3f tmp_3d;
        cv::Mat x3D_p = mLastFrame.UnprojectStereoSift(MatchId[i],0);
        tmp_3d.x = x3D_p.at<float>(0);
        tmp_3d.y = x3D_p.at<float>(1);
        tmp_3d.z = x3D_p.at<float>(2);
        pre_3d[i] = tmp_3d;
    }

    // camera matrix & distortion coefficients
    cv::Mat camera_mat(3, 3, CV_64FC1);
    cv::Mat distCoeffs = cv::Mat::zeros(1, 4, CV_64FC1);
    camera_mat.at<double>(0, 0) = mK.at<float>(0,0);
    camera_mat.at<double>(1, 1) = mK.at<float>(1,1);
    camera_mat.at<double>(0, 2) = mK.at<float>(0,2);
    camera_mat.at<double>(1, 2) = mK.at<float>(1,2);
    camera_mat.at<double>(2, 2) = 1.0;

    // output
    cv::Mat Rvec(3, 1, CV_64FC1);
    cv::Mat Tvec(3, 1, CV_64FC1);
    cv::Mat d(3, 3, CV_64FC1);
    cv::Mat inliers;

    // solve
    int iter_num = 500;
    double reprojectionError = 0.3, confidence = 0.98; // 0.5 0.3
    cv::solvePnPRansac(pre_3d, cur_2d, camera_mat, distCoeffs, Rvec, Tvec, false,
               iter_num, reprojectionError, confidence, inliers, cv::SOLVEPNP_AP3P); // AP3P EPNP P3P ITERATIVE DLS

    cv::Rodrigues(Rvec, d);

    // assign the result to current pose
    Mod.at<float>(0,0) = d.at<double>(0,0); Mod.at<float>(0,1) = d.at<double>(0,1); Mod.at<float>(0,2) = d.at<double>(0,2); Mod.at<float>(0,3) = Tvec.at<double>(0,0);
    Mod.at<float>(1,0) = d.at<double>(1,0); Mod.at<float>(1,1) = d.at<double>(1,1); Mod.at<float>(1,2) = d.at<double>(1,2); Mod.at<float>(1,3) = Tvec.at<double>(1,0);
    Mod.at<float>(2,0) = d.at<double>(2,0); Mod.at<float>(2,1) = d.at<double>(2,1); Mod.at<float>(2,2) = d.at<double>(2,2); Mod.at<float>(2,3) = Tvec.at<double>(2,0);


    // calculate the re-projection error
    std::vector<int> MM_inlier;
    const cv::Mat MotionModel = mVelocity*mLastFrame.mTcw;
    for (int i = 0; i < N; ++i)
    {
        const cv::Mat x3D  = (cv::Mat_<float>(3,1) << pre_3d[i].x, pre_3d[i].y, pre_3d[i].z);
        const cv::Mat x3D_c = MotionModel.rowRange(0,3).colRange(0,3)*x3D+MotionModel.rowRange(0,3).col(3);

        const float xc = x3D_c.at<float>(0);
        const float yc = x3D_c.at<float>(1);
        const float invzc = 1.0/x3D_c.at<float>(2);
        const float u = mCurrentFrame.fx*xc*invzc+mCurrentFrame.cx;
        const float v = mCurrentFrame.fy*yc*invzc+mCurrentFrame.cy;
        const float u_ = cur_2d[i].x - u;
        const float v_ = cur_2d[i].y - v;
        const float Rpe = std::sqrt(u_*u_ + v_*v_);
        if (Rpe<reprojectionError){
            MM_inlier.push_back(i);
        }
    }

    cv::Mat output;

    if (inliers.rows>MM_inlier.size())
    {
        // save the inliers IDs
        output = Mod;
        MatchId_sub.resize(inliers.rows);
        for (int i = 0; i < MatchId_sub.size(); ++i){
            MatchId_sub[i] = MatchId[inliers.at<int>(i)];
        }
        cout << "(Camera) AP3P+RanSac inliers/total number: " << inliers.rows << "/" << MatchId.size() << endl;
    }
    else
    {
        output = MotionModel;
        MatchId_sub.resize(MM_inlier.size());
        for (int i = 0; i < MatchId_sub.size(); ++i){
            MatchId_sub[i] = MatchId[MM_inlier[i]];
        }
        cout << "(Camera) Motion Model inliers/total number: " << MM_inlier.size() << "/" << MatchId.size() << endl;
    }

    return output;
}

cv::Mat Tracking::GetInitModelObj(const std::vector<int> &ObjId, std::vector<int> &ObjId_sub, const int objid)
{
    cv::Mat Mod = cv::Mat::eye(4,4,CV_32F);
    int N = ObjId.size();

    // construct input
    std::vector<cv::Point2f> cur_2d(N);
    std::vector<cv::Point3f> pre_3d(N);
    for (int i = 0; i < N; ++i)
    {
        cv::Point2f tmp_2d;
        tmp_2d.x = mCurrentFrame.mvObjKeys[ObjId[i]].pt.x;
        tmp_2d.y = mCurrentFrame.mvObjKeys[ObjId[i]].pt.y;
        cur_2d[i] = tmp_2d;
        cv::Point3f tmp_3d;
        cv::Mat x3D_p = mLastFrame.UnprojectStereoObject(ObjId[i],0);
        tmp_3d.x = x3D_p.at<float>(0);
        tmp_3d.y = x3D_p.at<float>(1);
        tmp_3d.z = x3D_p.at<float>(2);
        pre_3d[i] = tmp_3d;
    }

    // camera matrix & distortion coefficients
    cv::Mat camera_mat(3, 3, CV_64FC1);
    cv::Mat distCoeffs = cv::Mat::zeros(1, 4, CV_64FC1);
    camera_mat.at<double>(0, 0) = mK.at<float>(0,0);
    camera_mat.at<double>(1, 1) = mK.at<float>(1,1);
    camera_mat.at<double>(0, 2) = mK.at<float>(0,2);
    camera_mat.at<double>(1, 2) = mK.at<float>(1,2);
    camera_mat.at<double>(2, 2) = 1.0;

    // output
    cv::Mat Rvec(3, 1, CV_64FC1);
    cv::Mat Tvec(3, 1, CV_64FC1);
    cv::Mat d(3, 3, CV_64FC1);
    cv::Mat inliers;

    // solve
    int iter_num = 500;
    double reprojectionError = 0.3, confidence = 0.98; // 0.3 0.5 1.0
    cv::solvePnPRansac(pre_3d, cur_2d, camera_mat, distCoeffs, Rvec, Tvec, false,
               iter_num, reprojectionError, confidence, inliers, cv::SOLVEPNP_AP3P); // AP3P EPNP P3P ITERATIVE DLS

    cv::Rodrigues(Rvec, d);

    // assign the result to current pose
    Mod.at<float>(0,0) = d.at<double>(0,0); Mod.at<float>(0,1) = d.at<double>(0,1); Mod.at<float>(0,2) = d.at<double>(0,2); Mod.at<float>(0,3) = Tvec.at<double>(0,0);
    Mod.at<float>(1,0) = d.at<double>(1,0); Mod.at<float>(1,1) = d.at<double>(1,1); Mod.at<float>(1,2) = d.at<double>(1,2); Mod.at<float>(1,3) = Tvec.at<double>(1,0);
    Mod.at<float>(2,0) = d.at<double>(2,0); Mod.at<float>(2,1) = d.at<double>(2,1); Mod.at<float>(2,2) = d.at<double>(2,2); Mod.at<float>(2,3) = Tvec.at<double>(2,0);

    // ******* Generate Motion Model if it does exist from previous frame *******
    int CurObjLab = mCurrentFrame.nModLabel[objid];
    int PreObjID = -1;
    for (int i = 0; i < mLastFrame.nModLabel.size(); ++i)
    {
        if (mLastFrame.nModLabel[i]==CurObjLab)
        {
            PreObjID = i;
            break;
        }
    }

    cv::Mat MotionModel, output;
    std::vector<int> ObjId_tmp(N,-1); // new added Nov 19, 2019
    if (PreObjID!=-1)
    {
        // calculate the re-projection error
        std::vector<int> MM_inlier;
        MotionModel = mCurrentFrame.mTcw*mLastFrame.vObjMod[PreObjID];
        for (int i = 0; i < N; ++i)
        {
            const cv::Mat x3D  = (cv::Mat_<float>(3,1) << pre_3d[i].x, pre_3d[i].y, pre_3d[i].z);
            const cv::Mat x3D_c = MotionModel.rowRange(0,3).colRange(0,3)*x3D+MotionModel.rowRange(0,3).col(3);

            const float xc = x3D_c.at<float>(0);
            const float yc = x3D_c.at<float>(1);
            const float invzc = 1.0/x3D_c.at<float>(2);
            const float u = mCurrentFrame.fx*xc*invzc+mCurrentFrame.cx;
            const float v = mCurrentFrame.fy*yc*invzc+mCurrentFrame.cy;
            const float u_ = cur_2d[i].x - u;
            const float v_ = cur_2d[i].y - v;
            const float Rpe = std::sqrt(u_*u_ + v_*v_);
            if (Rpe<reprojectionError){
                MM_inlier.push_back(i);
            }
        }

        // ===== decide which model is best now =====
        if (inliers.rows>MM_inlier.size())
        {
            // save the inliers IDs
            output = Mod;
            ObjId_sub.resize(inliers.rows);
            for (int i = 0; i < ObjId_sub.size(); ++i){
                ObjId_sub[i] = ObjId[inliers.at<int>(i)];
                ObjId_tmp[inliers.at<int>(i)] = ObjId[inliers.at<int>(i)];
            }
            cout << "(Object) AP3P+RanSac inliers/total number: " << inliers.rows << "/" << ObjId.size() << endl;
        }
        else
        {
            output = MotionModel;
            ObjId_sub.resize(MM_inlier.size());
            for (int i = 0; i < ObjId_sub.size(); ++i){
                ObjId_sub[i] = ObjId[MM_inlier[i]];
                ObjId_tmp[MM_inlier[i]] = ObjId[MM_inlier[i]];
            }
            cout << "(Object) Motion Model inliers/total number: " << MM_inlier.size() << "/" << ObjId.size() << endl;
        }
    }
    else
    {
        // save the inliers IDs
        output = Mod;
        ObjId_sub.resize(inliers.rows);
        for (int i = 0; i < ObjId_sub.size(); ++i){
            ObjId_sub[i] = ObjId[inliers.at<int>(i)];
            ObjId_tmp[inliers.at<int>(i)] = ObjId[inliers.at<int>(i)];
        }
        cout << "(Object) AP3P+RanSac [No MM] inliers/total number: " << inliers.rows << "/" << ObjId.size() << endl;
    }

    // update on vObjLabel (Nov 19 2019)
    for (int i = 0; i < ObjId_tmp.size(); ++i)
    {
        if (ObjId_tmp[i]==-1)
            mCurrentFrame.vObjLabel[ObjId[i]]=-1;
    }

    return output;
}

cv::Mat Tracking::Delaunay(const std::vector<int> &id_dynamic)
{
    /// Return the Delaunay triangulation, under the form of an adjacency matrix
    /// points is a Nx2 mat containing the coordinates (x, y) of the points
    const int r_min = 0, c_min = 0, r_max = mCurrentFrame.mnMaxY, c_max = mCurrentFrame.mnMaxX;
    std::map<cv::Point2f, int, LessPoint2f> mappts;
    cv::Mat1b adj(id_dynamic.size(), id_dynamic.size(), uchar(0));

    /// Create subdiv and insert the points to it
    // cv::Subdiv2D subdiv(cv::Rect(r_min, c_min, r_max+std::fabs(r_min), c_max+std::fabs(c_min)));
    cv::Subdiv2D subdiv(cv::Rect(r_min, c_min, c_max, r_max));
    for (int p = 0; p < id_dynamic.size(); p++)
    {
        float xp = mCurrentFrame.mvSiftKeys[id_dynamic[p]].pt.x;
        float yp = mCurrentFrame.mvSiftKeys[id_dynamic[p]].pt.y;

        if(xp<0)
            xp=0;
        if(yp<0)
            yp=0;

        cv::Point2f fp(xp, yp);

        // cout << "the subdiv dimensions: " << xp << " " << yp << endl;

        // Don't add duplicates
        if (mappts.count(fp) == 0)
        {
            // Save point and index
            mappts[fp] = p;
            subdiv.insert(fp);
        }
    }

    /// Get the number of edges
    std::vector<cv::Vec4f> edgeList;
    subdiv.getEdgeList(edgeList);
    int nE = edgeList.size();

    /// Check adjacency
    for (int i = 0; i < nE; i++)
    {
        cv::Vec4f e = edgeList[i];
        cv::Point2f pt0(e[0], e[1]);
        cv::Point2f pt1(e[2], e[3]);

        if (mappts.count(pt0) == 0 || mappts.count(pt1) == 0) {
            continue;  // Not a valid point
        }

        int idx0 = mappts[pt0];
        int idx1 = mappts[pt1];

        // Symmetric matrix
        adj(idx0, idx1) = 1;
        adj(idx1, idx0) = 1;

    }

    return adj;
}

std::vector<Eigen::Vector4i> Tracking::GetMSS(const std::vector<int> &id_dynamic, const std::vector<int> &id_inter, const std::vector<int> &id_unknown, const Eigen::MatrixXi &Sorted_ind)
{

    int n_kp = id_dynamic.size(), m = 6;  // consider the first 6 neighbors
    std::vector<Eigen::Vector4i> nMSS;
    std::vector<std::vector<int> > Ind_cover(id_unknown.size());

    // Main Loop
    for (int i = 0; i < n_kp; ++i)
    {
        int cc = 0;
        Eigen::Vector4i mss_tmp;
        for (int k1 = 1; k1 < m-2; ++k1)
        {
            for (int k2 = k1+1; k2 < m-1; ++k2)
            {
                for (int k3 = k2+1; k3 < m; ++k3)
                {
                    // check if it is dynamic point
                    if (mCurrentFrame.vObjLabel[id_unknown[Sorted_ind(id_inter[i],k1)]]!=1 || mCurrentFrame.vObjLabel[id_unknown[Sorted_ind(id_inter[i],k2)]]!=1 || mCurrentFrame.vObjLabel[id_unknown[Sorted_ind(id_inter[i],k3)]]!=1){
                        cc++;
                        continue;
                    }

                    // check if it has been used
                    if (Ind_cover[id_inter[i]].size()>0){
                        bool used_1 = false, used_2 = false, used_3 = false;
                        for (int j = 0; j < Ind_cover[id_inter[i]].size(); ++j)
                        {
                            if (id_unknown[Sorted_ind(id_inter[i],k1)] == Ind_cover[id_inter[i]][j]){
                                used_1 = true;
                                break;
                            }
                        }
                        for (int j = 0; j < Ind_cover[id_inter[i]].size(); ++j)
                        {
                            if (id_unknown[Sorted_ind(id_inter[i],k2)]  == Ind_cover[id_inter[i]][j]){
                                used_2 = true;
                                break;
                            }
                        }
                        for (int j = 0; j < Ind_cover[id_inter[i]].size(); ++j)
                        {
                            if (id_unknown[Sorted_ind(id_inter[i],k3)]  == Ind_cover[id_inter[i]][j]){
                                used_3 = true;
                                break;
                            }
                        }
                        if (used_1 == true && used_2 == true && used_3 == true){
                            cc++;
                            continue;
                        }
                    }

                    // add new MSS
                    mss_tmp[0]=id_dynamic[i];
                    mss_tmp[1]=id_unknown[Sorted_ind(id_inter[i],k1)];
                    mss_tmp[2]=id_unknown[Sorted_ind(id_inter[i],k2)];
                    mss_tmp[3]=id_unknown[Sorted_ind(id_inter[i],k3)];
                    nMSS.push_back(mss_tmp);

                    // std::cout << "minimal sample set: " << mss_tmp[0] << " " << mss_tmp[1] << " " << mss_tmp[2] << " " << mss_tmp[3] << std::endl;

                    // update the index cover array
                    Ind_cover[id_inter[i]].push_back(id_unknown[Sorted_ind(id_inter[i],k1)]);
                    Ind_cover[id_inter[i]].push_back(id_unknown[Sorted_ind(id_inter[i],k2)]);
                    Ind_cover[id_inter[i]].push_back(id_unknown[Sorted_ind(id_inter[i],k3)]);
                    Ind_cover[Sorted_ind(id_inter[i],k1)].push_back(id_dynamic[i]);
                    Ind_cover[Sorted_ind(id_inter[i],k1)].push_back(id_unknown[Sorted_ind(id_inter[i],k2)]);
                    Ind_cover[Sorted_ind(id_inter[i],k1)].push_back(id_unknown[Sorted_ind(id_inter[i],k3)]);
                    Ind_cover[Sorted_ind(id_inter[i],k2)].push_back(id_dynamic[i]);
                    Ind_cover[Sorted_ind(id_inter[i],k2)].push_back(id_unknown[Sorted_ind(id_inter[i],k1)]);
                    Ind_cover[Sorted_ind(id_inter[i],k2)].push_back(id_unknown[Sorted_ind(id_inter[i],k3)]);
                    Ind_cover[Sorted_ind(id_inter[i],k3)].push_back(id_dynamic[i]);
                    Ind_cover[Sorted_ind(id_inter[i],k3)].push_back(id_unknown[Sorted_ind(id_inter[i],k1)]);
                    Ind_cover[Sorted_ind(id_inter[i],k3)].push_back(id_unknown[Sorted_ind(id_inter[i],k2)]);
                }
            }
        }
    }

    return nMSS;
}

cv::Mat Tracking::GetModel(const Eigen::Vector4i &mss, const vector<int> &TemperalMatch)
{
    cv::Mat Mod(6, 1, CV_64FC1);

    // construct input
    std::vector<cv::Point2f> cur_2d(4);
    std::vector<cv::Point3f> pre_3d(4);
    for (int i = 0; i < 4; ++i)
    {
        cur_2d[i].x = mCurrentFrame.mvSiftKeys[mss[i]].pt.x;
        cur_2d[i].y = mCurrentFrame.mvSiftKeys[mss[i]].pt.y;
        cv::Mat x3D_p = mLastFrame.UnprojectStereoSift(TemperalMatch[mss[i]],0); // big bug ! ! !
        pre_3d[i].x = x3D_p.at<float>(0);
        pre_3d[i].y = x3D_p.at<float>(1);
        pre_3d[i].z = x3D_p.at<float>(2);
    }


    // camera matrix & distortion coefficients
    cv::Mat camera_mat(3, 3, CV_64FC1);
    cv::Mat distCoeffs = cv::Mat::zeros(1, 4, CV_64FC1);
    camera_mat.at<double>(0, 0) = mK.at<float>(0,0);
    camera_mat.at<double>(1, 1) = mK.at<float>(1,1);
    camera_mat.at<double>(0, 2) = mK.at<float>(0,2);
    camera_mat.at<double>(1, 2) = mK.at<float>(1,2);
    camera_mat.at<double>(2, 2) = 1.0;

    // output
    cv::Mat Rvec;
    cv::Mat Tvec;

    // solve
    cv::solvePnP(pre_3d,cur_2d,camera_mat,distCoeffs,Rvec,Tvec,false,cv::SOLVEPNP_AP3P); // SOLVEPNP_ITERATIVE; SOLVEPNP_AP3P; SOLVEPNP_P3P;

    // assign the result to output
    Mod.at<double>(0,0) = Rvec.at<double>(0,0); Mod.at<double>(1,0) = Rvec.at<double>(1,0); Mod.at<double>(2,0) = Rvec.at<double>(2,0);
    Mod.at<double>(3,0) = Tvec.at<double>(0,0); Mod.at<double>(4,0) = Tvec.at<double>(1,0); Mod.at<double>(5,0) = Tvec.at<double>(2,0);

    // std::cout << "model: " << std::endl << Mod << std::endl;

    return Mod;
}

void Tracking::DrawLine(cv::KeyPoint &keys, cv::Point2f &flow, cv::Mat &ref_image, const cv::Scalar &color, int thickness, int line_type, const cv::Point2i &offset)
{

    auto cv_p1 = cv::Point2i(keys.pt.x,keys.pt.y);
    auto cv_p2 = cv::Point2i(keys.pt.x+flow.x,keys.pt.y+flow.y);
    //cout << "p1: " << cv_p1 << endl;
    //cout << "p2: " << cv_p2 << endl;

    bool p1_in_bounds = true;
    bool p2_in_bounds = true;
    if ((cv_p1.x < 0) && (cv_p1.y < 0) && (cv_p1.x > ref_image.cols) && (cv_p1.y > ref_image.rows) )
        p1_in_bounds = false;

    if ((cv_p2.x < 0) && (cv_p2.y < 0) && (cv_p2.x > ref_image.cols) && (cv_p2.y > ref_image.rows) )
        p2_in_bounds = false;

    // Draw line, but only if both end-points project into the image!
    if (p1_in_bounds || p2_in_bounds) { // This is correct. Won't draw only if both lines are out of bounds.
        // Draw line
        auto p1_offs = offset+cv_p1;
        auto p2_offs = offset+cv_p2;
        if (cv::clipLine(cv::Size(ref_image.cols, ref_image.rows), p1_offs, p2_offs)) {
            //cv::line(ref_image, p1_offs, p2_offs, color, thickness, line_type);
            cv::arrowedLine(ref_image, p1_offs, p2_offs, color, thickness, line_type);
        }
    }
}

void Tracking::DrawTransparentSquare(cv::Point center, cv::Vec3b color, int radius, double alpha, cv::Mat &ref_image)
{
    for (int i=-radius; i<radius; i++) {
        for (int j=-radius; j<radius; j++) {
            int coord_y = center.y + i;
            int coord_x = center.x + j;

            if (coord_x>0 && coord_y>0 && coord_x<ref_image.cols && coord_y < ref_image.rows) {
                ref_image.at<cv::Vec3b>(cv::Point(coord_x,coord_y)) = (1.0-alpha)*ref_image.at<cv::Vec3b>(cv::Point(coord_x,coord_y)) + alpha*color;
            }
        }
    }
}

void Tracking::DrawGridBirdeye(double res_x, double res_z, const BirdEyeVizProperties &viz_props, cv::Mat &ref_image)
{

    auto color = cv::Scalar(0.0, 0.0, 0.0);
    // Draw horizontal lines
    for (double i=0; i<viz_props.birdeye_far_plane_; i+=res_z) {
        double x_1 = viz_props.birdeye_left_plane_;
        double y_1 = i;
        double x_2 = viz_props.birdeye_right_plane_;
        double y_2 = i;
        TransformPointToScaledFrustum(x_1, y_1, viz_props);
        TransformPointToScaledFrustum(x_2, y_2, viz_props);
        auto p1 = cv::Point(x_1, y_1), p2=cv::Point(x_2,y_2);
        cv::line(ref_image, p1, p2, color);
    }

    // Draw vertical lines
    for (double i=viz_props.birdeye_left_plane_; i<viz_props.birdeye_right_plane_; i+=res_x) {
        double x_1 = i;
        double y_1 = 0;
        double x_2 = i;
        double y_2 = viz_props.birdeye_far_plane_;
        TransformPointToScaledFrustum(x_1, y_1, viz_props);
        TransformPointToScaledFrustum(x_2, y_2, viz_props);
        auto p1 = cv::Point(x_1, y_1), p2=cv::Point(x_2,y_2);
        cv::line(ref_image, p1, p2, color);
    }
}

void Tracking::DrawSparseFlowBirdeye(
        const std::vector<Eigen::Vector3d> &pts, const std::vector<Eigen::Vector3d> &vel,
        const cv::Mat &camera, const BirdEyeVizProperties &viz_props, cv::Mat &ref_image)
{

    // For scaling / flipping cov. matrices
    Eigen::Matrix2d flip_mat;
    flip_mat << viz_props.birdeye_scale_factor_*1.0, 0, 0, viz_props.birdeye_scale_factor_*1.0;
    Eigen::Matrix2d world_to_cam_mat;
    const Eigen::Matrix4d &ref_to_rt_inv = Converter::toMatrix4d(camera);
    world_to_cam_mat << ref_to_rt_inv(0,0), ref_to_rt_inv(2,0), ref_to_rt_inv(0,2), ref_to_rt_inv(2,2);
    flip_mat = flip_mat*world_to_cam_mat;

    // Parameters
    // const int line_width = 2;

    ref_image = cv::Mat(viz_props.birdeye_scale_factor_*viz_props.birdeye_far_plane_,
                        (-viz_props.birdeye_left_plane_+viz_props.birdeye_right_plane_)*viz_props.birdeye_scale_factor_, CV_32FC3);
    ref_image.setTo(cv::Scalar(1.0, 1.0, 1.0));
    Tracking::DrawGridBirdeye(1.0, 1.0, viz_props, ref_image);


    for (int i=0; i<pts.size(); i++) {

        Eigen::Vector3d p_3d = pts[i];
        Eigen::Vector3d p_vel = vel[i];

        if (p_3d[0]==-1 || p_3d[1]==-1 || p_3d[2]<0)
            continue;
        if (p_vel[0]>0.1 || p_vel[2]>0.1)
            continue;

        // float xc = p_3d[0];
        // float yc = p_3d[1];
        // float invzc = 1.0/p_3d[2];
        // float u = mCurrentFrame.fx*xc*invzc+mCurrentFrame.cx;
        // float v = mCurrentFrame.fy*yc*invzc+mCurrentFrame.cy;
        // Eigen::Vector3i p_proj = Eigen::Vector3i(round(u), round(v), 1);
        const Eigen::Vector2d velocity = Eigen::Vector2d(p_vel[0], p_vel[2]); // !!!
        Eigen::Vector3d dir(velocity[0], 0.0, velocity[1]);

        double x_1 = p_3d[0];
        double z_1 = p_3d[2];

        double x_2 = x_1 + dir[0];
        double z_2 = z_1 + dir[2];

        // cout << dir[0] << " " << dir[2] << endl;

        if (x_1 > viz_props.birdeye_left_plane_ && x_2 > viz_props.birdeye_left_plane_ &&
            x_1 < viz_props.birdeye_right_plane_ && x_2 < viz_props.birdeye_right_plane_ &&
            z_1 > 0 && z_2 > 0 &&
            z_1 < viz_props.birdeye_far_plane_ && z_2 < viz_props.birdeye_far_plane_) {

            TransformPointToScaledFrustum(x_1, z_1, viz_props); //velocity[0], velocity[1]);
            TransformPointToScaledFrustum(x_2, z_2, viz_props); //velocity[0], velocity[1]);

            cv::arrowedLine(ref_image, cv::Point(x_1, z_1), cv::Point(x_2, z_2), cv::Scalar(1.0, 0.0, 0.0), 1);
            cv::circle(ref_image, cv::Point(x_1, z_1), 3.0, cv::Scalar(0.0, 0.0, 1.0), -1.0);
        }
    }

    // Coord. sys.
    int arrow_len = 60;
    int offset_y = 10;
    cv::arrowedLine(ref_image, cv::Point(ref_image.cols/2, offset_y),
                    cv::Point(ref_image.cols/2+arrow_len, offset_y),
                    cv::Scalar(1.0, 0, 0), 2);
    cv::arrowedLine(ref_image, cv::Point(ref_image.cols/2, offset_y),
                    cv::Point(ref_image.cols/2, offset_y+arrow_len),
                    cv::Scalar(0.0, 1.0, 0), 2);

    //cv::putText(ref_image, "X", cv::Point(ref_image.cols/2+arrow_len+10, offset_y+10), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(1.0, 0, 0));
    //cv::putText(ref_image, "Z", cv::Point(ref_image.cols/2+10, offset_y+arrow_len), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0.0, 1.0, 0));

    // Flip image, because it is more intuitive to have ref. point at the bottom of the image
    cv::Mat dst;
    cv::flip(ref_image, dst, 0);
    ref_image = dst;
}

void Tracking::TransformPointToScaledFrustum(double &pose_x, double &pose_z, const BirdEyeVizProperties &viz_props)
{
    pose_x += (-viz_props.birdeye_left_plane_);
    pose_x *= viz_props.birdeye_scale_factor_;
    pose_z *= viz_props.birdeye_scale_factor_;
}

cv::Mat Tracking::ObjPoseParsing(const std::vector<float> &vObjPose_gt)
{
    // assign t vector
    cv::Mat t(3, 1, CV_32FC1);
    t.at<float>(0) = vObjPose_gt[6];
    t.at<float>(1) = vObjPose_gt[7];
    t.at<float>(2) = vObjPose_gt[8];

    // from Euler to Rotation Matrix
    cv::Mat R(3, 3, CV_32FC1);

    // assign r vector
    float y = vObjPose_gt[9]+(3.1415926/2); // +(3.1415926/2)
    float x = 0.0;
    float z = 0.0;

    // the angles are in radians.
    float cy = cos(y);
    float sy = sin(y);
    float cx = cos(x);
    float sx = sin(x);
    float cz = cos(z);
    float sz = sin(z);

    float m00, m01, m02, m10, m11, m12, m20, m21, m22;

    // ====== R = Ry*Rx*Rz =======

    // m00 = cy;
    // m01 = -sy;
    // m02 = 0;
    // m10 = sy;
    // m11 = cy;
    // m12 = 0;
    // m20 = 0;
    // m21 = 0;
    // m22 = 1;

    m00 = cy*cz+sy*sx*sz;
    m01 = -cy*sz+sy*sx*cz;
    m02 = sy*cx;
    m10 = cx*sz;
    m11 = cx*cz;
    m12 = -sx;
    m20 = -sy*cz+cy*sx*sz;
    m21 = sy*sz+cy*sx*cz;
    m22 = cy*cx;

    // ***************** old **************************

    // float alpha = vObjPose_gt[7]; // 7
    // float beta = vObjPose_gt[5]+(3.1415926/2);  // 5
    // float gamma = vObjPose_gt[6]; // 6

    // the angles are in radians.
    // float ca = cos(alpha);
    // float sa = sin(alpha);
    // float cb = cos(beta);
    // float sb = sin(beta);
    // float cg = cos(gamma);
    // float sg = sin(gamma);

    // float m00, m01, m02, m10, m11, m12, m20, m21, m22;

    // default
    // m00 = cb*ca;
    // m01 = cb*sa;
    // m02 = -sb;
    // m10 = sb*sg*ca-sa*cg;
    // m11 = sb*sg*sa+ca*cg;
    // m12 = cb*sg;
    // m20 = sb*cg*ca+sa*sg;
    // m21 = sb*cg*sa-ca*sg;
    // m22 = cb*cg;

    // m00 = ca*cb;
    // m01 = ca*sb*sg - sa*cg;
    // m02 = ca*sb*cg + sa*sg;
    // m10 = sa*cb;
    // m11 = sa*sb*sg + ca*cg;
    // m12 = sa*sb*cg - ca*sg;
    // m20 = -sb;
    // m21 = cb*sg;
    // m22 = cb*cg;

    // **************************************************

    R.at<float>(0,0) = m00;
    R.at<float>(0,1) = m01;
    R.at<float>(0,2) = m02;
    R.at<float>(1,0) = m10;
    R.at<float>(1,1) = m11;
    R.at<float>(1,2) = m12;
    R.at<float>(2,0) = m20;
    R.at<float>(2,1) = m21;
    R.at<float>(2,2) = m22;

    // construct 4x4 transformation matrix
    cv::Mat Pose = cv::Mat::eye(4,4,CV_32F);
    Pose.at<float>(0,0) = R.at<float>(0,0); Pose.at<float>(0,1) = R.at<float>(0,1); Pose.at<float>(0,2) = R.at<float>(0,2); Pose.at<float>(0,3) = t.at<float>(0);
    Pose.at<float>(1,0) = R.at<float>(1,0); Pose.at<float>(1,1) = R.at<float>(1,1); Pose.at<float>(1,2) = R.at<float>(1,2); Pose.at<float>(1,3) = t.at<float>(1);
    Pose.at<float>(2,0) = R.at<float>(2,0); Pose.at<float>(2,1) = R.at<float>(2,1); Pose.at<float>(2,2) = R.at<float>(2,2); Pose.at<float>(2,3) = t.at<float>(2);

    // cout << "OBJ Pose: " << endl << Pose << endl;

    return Pose;

}

cv::Mat Tracking::InvMatrix(const cv::Mat &T)
{
    cv::Mat T_inv = cv::Mat::eye(4,4,CV_32F);

    const cv::Mat R = T.rowRange(0,3).colRange(0,3);
    const cv::Mat t = T.rowRange(0,3).col(3);
    cv::Mat t_inv = -R.t()*t;
    cv::Mat R_inv = R.t();

    cv::Mat tmp_R = T_inv.rowRange(0,3).colRange(0,3);
    R_inv.copyTo(tmp_R);
    cv::Mat tmp_t = T_inv.rowRange(0,3).col(3);
    t_inv.copyTo(tmp_t);

    return T_inv;
}

cv::Mat Tracking::RanSacHorn(const vector<int> &TemperalMatch, const vector<int> &ObjId)
{
    cv::RNG rng((unsigned)time(NULL));

    // initialize the parameters for ransac
    double thres = 0.15;  // p = 0.98, e = 0.7,
    int s = 3, iter_num = 500;
    //int iter_num = (int)log(1-p)/log(1-pow(1-e,s));
    cout << "iterations times: " << iter_num << endl;

    // get the corresponding 3d points
    int nPointNum = ObjId.size();
    Eigen::Matrix3Xd T_in(3,nPointNum), M_out(3,nPointNum);

    for(int i=0; i< nPointNum; i++){

        cv::Mat x3D_p = mLastFrame.UnprojectStereoSift(TemperalMatch[ObjId[i]],0);
        cv::Mat x3D_c = mCurrentFrame.UnprojectStereoSift(ObjId[i],0);

        T_in(0,i) = x3D_p.at<float>(0); T_in(1,i) = x3D_p.at<float>(1); T_in(2,i) = x3D_p.at<float>(2);
        M_out(0,i) = x3D_c.at<float>(0); M_out(1,i) = x3D_c.at<float>(1); M_out(2,i) = x3D_c.at<float>(2);

    }

    cout << "features in frame k-1" << endl;
    for (int i = 0; i < nPointNum; ++i)
    {
        float xp = mLastFrame.mvSiftKeys[TemperalMatch[ObjId[i]]].pt.x;
        float yp = mLastFrame.mvSiftKeys[TemperalMatch[ObjId[i]]].pt.y;
        cout << xp << " " << yp << endl;
    }
    cout << "correspondences in frame k" << endl;
    for (int i = 0; i < nPointNum; ++i)
    {
        float x = mCurrentFrame.mvSiftKeys[ObjId[i]].pt.x;
        float y = mCurrentFrame.mvSiftKeys[ObjId[i]].pt.y;
        cout << x << " " << y << endl;
    }


    // main loop for the ransac
    int count = 0, candidate, inlier_num = 0;
    vector<int> inlier_idx;
    Eigen::Matrix3Xd T_in_s(3,s), M_out_s(3,s);
    while (count<iter_num){

        // random sample s candidate points
        for(int i=0; i< s; i++){
            candidate = rng.uniform(0, nPointNum-1);
            T_in_s(0,i) = T_in(0,candidate); T_in_s(1,i) = T_in(1,candidate); T_in_s(2,i) = T_in(2,candidate);
            M_out_s(0,i) = M_out(0,candidate); M_out_s(1,i) = M_out(1,candidate); M_out_s(2,i) = M_out(2,candidate);
        }

        // fit a model to the candidate points
        Eigen::Matrix4d A = Eigen::umeyama (T_in_s, M_out_s, false);


        // find the model with most inliers
        vector<int> inlier_tmp;
        for(int i=0; i< nPointNum; i++){
            if((A.topLeftCorner(3, 3)*T_in.col(i)+A.block(0, 3, 3, 1)-M_out.col(i)).norm()<thres){
                inlier_tmp.push_back(i);
            }
        }
        if((int)inlier_tmp.size()>inlier_num){
            //inlier_idx.assign(inlier_tmp.begin(),inlier_tmp.end());
            inlier_idx = inlier_tmp;
            inlier_num = (int)inlier_tmp.size();
        }

        count++;

    }

    // estimate the model using the most inliers found
    cout << "inliers number: " << inlier_num << "/" << nPointNum << endl;

    Eigen::Matrix3Xd T_in_m(3,inlier_num), M_out_m(3,inlier_num);
    for(int i=0; i< inlier_num; i++){
        T_in_m(0,i) = T_in(0,inlier_idx[i]);   T_in_m(1,i) = T_in(1,inlier_idx[i]);   T_in_m(2,i) = T_in(2,inlier_idx[i]);
        M_out_m(0,i) = M_out(0,inlier_idx[i]); M_out_m(1,i) = M_out(1,inlier_idx[i]); M_out_m(2,i) = M_out(2,inlier_idx[i]);
    }
    Eigen::Matrix4d A = Eigen::umeyama (T_in_m, M_out_m, false);

    cv::Mat Pose = cv::Mat::eye(4,4,CV_32F);

    Pose.at<float>(0,0) = A(0,0); Pose.at<float>(0,1) = A(0,1); Pose.at<float>(0,2) = A(0,2); Pose.at<float>(0,3) = A(0,3);
    Pose.at<float>(1,0) = A(1,0); Pose.at<float>(1,1) = A(1,1); Pose.at<float>(1,2) = A(1,2); Pose.at<float>(1,3) = A(1,3);
    Pose.at<float>(2,0) = A(2,0); Pose.at<float>(1,2) = A(2,1); Pose.at<float>(2,2) = A(2,2); Pose.at<float>(2,3) = A(2,3);

    return Pose;

}
// Given two sets of 3D points, find the rotation + translation + scale
// which best maps the first set to the second.
// Source: http://en.wikipedia.org/wiki/Kabsch_algorithm
cv::Mat Tracking::Find3DAffineTransform(const vector<int> &TemperalMatch, const vector<int> &ObjId)
{

    // Default output
    Eigen::Affine3d A;
    A.linear() = Eigen::Matrix3d::Identity(3, 3);
    A.translation() = Eigen::Vector3d::Zero();

    // Get input
    int nPointNum = ObjId.size();
    Eigen::Matrix3Xd in(3,nPointNum), out(3,nPointNum);

    for(int i=0; i< nPointNum; i++){

        cv::Mat x3D_p = mLastFrame.UnprojectStereoSift(TemperalMatch[ObjId[i]],0);
        cv::Mat x3D_c = mCurrentFrame.UnprojectStereoSift(ObjId[i],0);

        in(0,i) = x3D_p.at<float>(0); in(1,i) = x3D_p.at<float>(1); in(2,i) = x3D_p.at<float>(2);
        out(0,i) = x3D_c.at<float>(0); out(1,i) = x3D_c.at<float>(1); out(2,i) = x3D_c.at<float>(2);

    }

    cout << "points in k-1 frame: " << endl;
    for(int i=0; i< nPointNum; i++)
        cout << in(0,i) << " " << in(1,i) << " " << in(2,i) << endl;
    cout << "points in k frame: " << endl;
    for(int i=0; i< nPointNum; i++)
        cout << out(0,i) << " " << out(1,i) << " " << out(2,i) << endl;


    if (in.cols() != out.cols())
        throw "Find3DAffineTransform(): input data mis-match";

    // First find the scale, by finding the ratio of sums of some distances,
    // then bring the datasets to the same scale.
    double scale = 0, dist_in = 0, dist_out = 0;
    for (int col = 0; col < in.cols()-1; col++) {
        dist_in  += (in.col(col+1) - in.col(col)).norm();
        dist_out += (out.col(col+1) - out.col(col)).norm();
    }

    //double scale = dist_out/dist_in;
    scale = dist_out/dist_in;
    out /= scale;

    // Find the centroids then shift to the origin
    Eigen::Vector3d in_ctr = Eigen::Vector3d::Zero();
    Eigen::Vector3d out_ctr = Eigen::Vector3d::Zero();
    for (int col = 0; col < in.cols(); col++) {
        in_ctr  += in.col(col);
        out_ctr += out.col(col);
    }
    in_ctr /= in.cols();
    out_ctr /= out.cols();
    for (int col = 0; col < in.cols(); col++) {
        in.col(col)  -= in_ctr;
        out.col(col) -= out_ctr;
    }

    // SVD
    Eigen::MatrixXd Cov = in * out.transpose();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Cov, Eigen::ComputeThinU | Eigen::ComputeThinV);

    // Find the rotation
    double d = (svd.matrixV() * svd.matrixU().transpose()).determinant();
    if (d > 0)
        d = 1.0;
    else
        d = -1.0;
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3, 3);
    I(2, 2) = d;
    Eigen::Matrix3d R = svd.matrixV() * I * svd.matrixU().transpose();

    // The final transform
    A.linear() = scale * R;
    A.translation() = scale*(out_ctr - R*in_ctr);

    cv::Mat Pose = cv::Mat::eye(4,4,CV_32F);

    Pose.at<float>(0,0) = A.linear()(0,0); Pose.at<float>(0,1) = A.linear()(0,1); Pose.at<float>(0,2) = A.linear()(0,2);
    Pose.at<float>(1,0) = A.linear()(1,0); Pose.at<float>(1,1) = A.linear()(1,1); Pose.at<float>(1,2) = A.linear()(1,2);
    Pose.at<float>(2,0) = A.linear()(2,0); Pose.at<float>(2,1) = A.linear()(2,1); Pose.at<float>(2,2) = A.linear()(2,2);
    Pose.at<float>(0,3) = A.translation()(0,0); Pose.at<float>(1,3) = A.translation()(1,0); Pose.at<float>(2,3) = A.translation()(2,0);

    return Pose;
}

void Tracking::StackObjInfo(std::vector<cv::KeyPoint> &FeatDynObj, std::vector<float> &DepDynObj,
                  std::vector<int> &FeatLabObj)
{
    for (int i = 0; i < mCurrentFrame.vnObjID.size(); ++i)
    {
        for (int j = 0; j < mCurrentFrame.vnObjID[i].size(); ++j)
        {
            FeatDynObj.push_back(mLastFrame.mvObjKeys[mCurrentFrame.vnObjID[i][j]]);
            FeatDynObj.push_back(mCurrentFrame.mvObjKeys[mCurrentFrame.vnObjID[i][j]]);
            DepDynObj.push_back(mLastFrame.mvObjDepth[mCurrentFrame.vnObjID[i][j]]);
            DepDynObj.push_back(mCurrentFrame.mvObjDepth[mCurrentFrame.vnObjID[i][j]]);
            FeatLabObj.push_back(mCurrentFrame.vObjLabel[mCurrentFrame.vnObjID[i][j]]);
        }
    }
}

std::vector<std::vector<std::pair<int, int> > > Tracking::GetStaticTrack()
{
    // Get temporal match from Map
    std::vector<std::vector<int> > TemporalMatch = mpMap->vnAssoSta;
    int N = TemporalMatch.size();
    // save the track id in TrackLets for previous frame and current frame.
    std::vector<int> TrackCheck_pre;
    // pair.first = frameID; pair.second = featureID;
    std::vector<std::vector<std::pair<int, int> > > TrackLets;

    // main loop
    int IDsofar = 0;
    for (int i = 0; i < N; ++i)
    {
        // initialize TrackCheck
        std::vector<int> TrackCheck_cur(TemporalMatch[i].size(),-1);

        // check each feature
        for (int j = 0; j < TemporalMatch[i].size(); ++j)
        {
            // first pair of frames (frame 0 and 1)
            if(i==0)
            {
                // check if there's association
                if (TemporalMatch[i][j]!=-1)
                {
                    // first, save one tracklet consisting of two featureID
                    // pair.first = frameID; pair.second = featureID
                    std::vector<std::pair<int, int> > TraLet(2);
                    TraLet[0] = std::make_pair(i,TemporalMatch[i][j]);
                    TraLet[1] = std::make_pair(i+1,j);
                    // then, save to the main tracklets list
                    TrackLets.push_back(TraLet);

                    // save tracklet ID
                    TrackCheck_cur[j] = IDsofar;
                    IDsofar = IDsofar + 1;
                }
                else
                    continue;
            }
            // frame i and i+1 (i>0)
            else
            {
                // check if there's association
                if (TemporalMatch[i][j]!=-1)
                {
                    // check the TrackID in previous frame
                    // if it is associated before, then add to existing tracklets.
                    if (TrackCheck_pre[TemporalMatch[i][j]]!=-1)
                    {
                        TrackLets[TrackCheck_pre[TemporalMatch[i][j]]].push_back(std::make_pair(i+1,j));
                        TrackCheck_cur[j] = TrackCheck_pre[TemporalMatch[i][j]];
                    }
                    // if not, insert new tracklets.
                    else
                    {
                        // first, save one tracklet consisting of two featureID
                        std::vector<std::pair<int, int> > TraLet(2);
                        TraLet[0] = std::make_pair(i,TemporalMatch[i][j]);
                        TraLet[1] = std::make_pair(i+1,j);
                        // then, save to the main tracklets list
                        TrackLets.push_back(TraLet);

                        // save tracklet ID
                        TrackCheck_cur[j] = IDsofar;
                        IDsofar = IDsofar + 1;
                    }
                }
                else
                    continue;
            }
        }

        TrackCheck_pre = TrackCheck_cur;
    }

    // // display info
    // cout << endl;
    // cout << "==============================================" << endl;
    // cout << "the number of static feature tracklets: " << TrackLets.size() << endl;
    // cout << "==============================================" << endl;
    // cout << endl;

    // std::vector<int> TrackLength(N,0);
    // for (int i = 0; i < TrackLets.size(); ++i)
    //     TrackLength[TrackLets[i].size()-2]++;

    // for (int i = 0; i < N; ++i)
    //     cout << "The length of " << i+2 << " tracklets is found with the amount of " << TrackLength[i] << " ..." << endl;
    // cout << endl;

    return TrackLets;
}

std::vector<std::vector<std::pair<int, int> > > Tracking::GetDynamicTrackNew()
{
    // Get temporal match from Map
    std::vector<std::vector<int> > TemporalMatch = mpMap->vnAssoDyn;
    std::vector<std::vector<int> > ObjLab = mpMap->vnFeatLabel;
    int N = TemporalMatch.size();
    // save the track id in TrackLets for previous frame and current frame.
    std::vector<int> TrackCheck_pre;
    // pair.first = frameID; pair.second = featureID;
    std::vector<std::vector<std::pair<int, int> > > TrackLets;
    // save object id of each tracklets
    std::vector<int> ObjectID;

    // main loop
    int IDsofar = 0;
    for (int i = 0; i < N; ++i)
    {
        // initialize TrackCheck
        std::vector<int> TrackCheck_cur(TemporalMatch[i].size(),-1);

        // check each feature
        for (int j = 0; j < TemporalMatch[i].size(); ++j)
        {
            // first pair of frames (frame 0 and 1)
            if(i==0)
            {
                // check if there's association
                if (TemporalMatch[i][j]!=-1)
                {
                    // first, save one tracklet consisting of two featureID
                    // pair.first = frameID, pair.second = featureID
                    std::vector<std::pair<int, int> > TraLet(2);
                    TraLet[0] = std::make_pair(i,TemporalMatch[i][j]);
                    TraLet[1] = std::make_pair(i+1,j);
                    // then, save to the main tracklets list
                    TrackLets.push_back(TraLet);
                    ObjectID.push_back(ObjLab[i][j]);

                    // save tracklet ID
                    TrackCheck_cur[j] = IDsofar;
                    IDsofar = IDsofar + 1;
                }
            }
            // frame i and i+1 (i>0)
            else
            {
                // check if there's association
                if (TemporalMatch[i][j]!=-1)
                {
                    // check the TrackID in previous frame
                    // if it is associated before, then add to existing tracklets.
                    if (TrackCheck_pre[TemporalMatch[i][j]]!=-1)
                    {
                        TrackLets[TrackCheck_pre[TemporalMatch[i][j]]].push_back(std::make_pair(i+1,j));
                        TrackCheck_cur[j] = TrackCheck_pre[TemporalMatch[i][j]];
                    }
                    // if not, insert new tracklets.
                    else
                    {
                        // first, save one tracklet consisting of two featureID
                        std::vector<std::pair<int, int> > TraLet(2);
                        TraLet[0] = std::make_pair(i,TemporalMatch[i][j]);
                        TraLet[1] = std::make_pair(i+1,j);
                        // then, save to the main tracklets list
                        TrackLets.push_back(TraLet);
                        ObjectID.push_back(ObjLab[i][j]);

                        // save tracklet ID
                        TrackCheck_cur[j] = IDsofar;
                        IDsofar = IDsofar + 1;
                    }
                }
            }
        }

        TrackCheck_pre = TrackCheck_cur;
    }

    // update object ID list
    mpMap->nObjID = ObjectID;

    // // display info
    // cout << endl;
    // cout << "==============================================" << endl;
    // cout << "the number of dynamic feature tracklets: " << TrackLets.size() << endl;
    // cout << "==============================================" << endl;
    // cout << endl;

    // std::vector<int> TrackLength(N,0);
    // for (int i = 0; i < TrackLets.size(); ++i)
    //     TrackLength[TrackLets[i].size()-2]++;

    // for (int i = 0; i < N; ++i)
    //     cout << "The length of " << i+2 << " tracklets is found with the amount of " << TrackLength[i] << " ..." << endl;
    // cout << endl;

    return TrackLets;
}

std::vector<std::vector<std::pair<int, int> > > Tracking::GetDynamicTrack()
{
    std::vector<std::vector<cv::KeyPoint> > Feats = mpMap->vpFeatDyn;
    std::vector<std::vector<int> > ObjLab = mpMap->vnFeatLabel;
    int N = Feats.size();

    // pair.first = frameID; pair.second = featureID;
    std::vector<std::vector<std::pair<int, int> > > TrackLets;
    // save object id of each tracklets
    std::vector<int> ObjectID;
    // save the track id in TrackLets for previous frame and current frame.
    std::vector<int> TrackCheck_pre;


    // main loop
    int IDsofar = 0;
    for (int i = 0; i < N; ++i)
    {
        // initialize TrackCheck
        std::vector<int> TrackCheck_cur(Feats[i].size(),-1);

        // Check empty
        if (Feats[i].empty())
        {
           TrackCheck_pre = TrackCheck_cur;
           continue;
        }

        // first pair of frames (frame 0 and 1)
        if (i==0)
        {
            int M = Feats[i].size();
            for (int j = 0; j < M; j=j+2)
            {
                // first, save one tracklet consisting of two featureID
                std::vector<std::pair<int, int> > TraLet(2);
                TraLet[0] = std::make_pair(i,j);
                TraLet[1] = std::make_pair(i,j+1); // used to be i+1
                // then, save to the main tracklets list
                TrackLets.push_back(TraLet);
                ObjectID.push_back(ObjLab[i][j/2]);

                // finally, save tracklet ID
                TrackCheck_cur[j+1] = IDsofar;
                IDsofar = IDsofar + 1;
            }
        }
        // frame i and i+1 (i>0)
        else
        {
            int M_pre = TrackCheck_pre.size();
            int M_cur = Feats[i].size();

            if (M_pre==0)
            {
                for (int j = 0; j < M_cur; j=j+2)
                {
                    // first, save one tracklet consisting of two featureID
                    std::vector<std::pair<int, int> > TraLet(2);
                    TraLet[0] = std::make_pair(i,j);
                    TraLet[1] = std::make_pair(i,j+1); // used to be i+1
                    // then, save to the main tracklets list
                    TrackLets.push_back(TraLet);
                    ObjectID.push_back(ObjLab[i][j/2]);

                    // finally, save tracklet ID
                    TrackCheck_cur[j+1] = IDsofar;
                    IDsofar = IDsofar + 1;
                }
            }
            else
            {
                // (1) find the temporal matching list (TM) between
                // previous flow locations and current sampled locations
                vector<int> TM(M_cur,-1);
                std::vector<float> MinDist(M_cur,-1);
                int nmatches = 0;
                for (int k = 1; k < M_pre; k=k+2)
                {
                    float x_ = Feats[i-1][k].pt.x;
                    float y_ = Feats[i-1][k].pt.y;
                    float min_dist = 10;
                    int candi = -1;
                    for (int j = 0; j < M_cur; j=j+2)
                    {
                        if (ObjLab[i-1][(k-1)/2]!=ObjLab[i][j/2])
                            continue;

                        float x  = Feats[i][j].pt.x;
                        float y  = Feats[i][j].pt.y;
                        float dist = std::sqrt( (x_-x)*(x_-x) + (y_-y)*(y_-y) );

                        if (dist<min_dist){
                            min_dist = dist;
                            candi = j;
                        }
                    }
                    // threshold
                    if (min_dist<1.0)
                    {
                        // current feature not occupied -or- occupied but new distance is smaller
                        // then label current match
                        if (TM[candi]==-1 || (TM[candi]!=-1 && min_dist<MinDist[candi]))
                        {
                            TM[candi] = k;
                            MinDist[candi] = min_dist;
                            nmatches = nmatches + 1;
                        }
                    }
                }

                // (2) save tracklets according to TM
                for (int j = 0; j < M_cur; j=j+2)
                {
                    // check the TM. if it is associated with last frame, then add to existing tracklets.
                    if (TM[j]!=-1)
                    {
                        TrackLets[TrackCheck_pre[TM[j]]].push_back(std::make_pair(i,j+1)); // used to be i+1
                        TrackCheck_cur[j+1] = TrackCheck_pre[TM[j]];
                    }
                    else
                    {
                        std::vector<std::pair<int, int> > TraLet(2);
                        TraLet[0] = std::make_pair(i,j);
                        TraLet[1] = std::make_pair(i,j+1); // used to be i+1
                        // then, save to the main tracklets list
                        TrackLets.push_back(TraLet);
                        ObjectID.push_back(ObjLab[i][j/2]);

                        // save tracklet ID
                        TrackCheck_cur[j+1] = IDsofar;
                        IDsofar = IDsofar + 1;
                    }
                }
            }
        }

        TrackCheck_pre = TrackCheck_cur;
    }

    // update object ID list
    mpMap->nObjID = ObjectID;


    // display info
    cout << endl;
    cout << "==============================================" << endl;
    cout << "the number of object feature tracklets: " << TrackLets.size() << endl;
    cout << "==============================================" << endl;
    cout << endl;

    std::vector<int> TrackLength(N,0);
    for (int i = 0; i < TrackLets.size(); ++i)
        TrackLength[TrackLets[i].size()-2]++;

    for (int i = 0; i < N; ++i)
        cout << "The length of " << i+2 << " tracklets is found with the amount of " << TrackLength[i] << " ..." << endl;
    cout << endl;


    return TrackLets;
}

void Tracking::RenewFrameInfo(const std::vector<int> &TM_sta)
{
    // cout << endl << "START UPDATE FRAME INFORMATION......" << endl;

    // ---------------------------------------------------------------------------------------
    // ++++++++++++++++++++++++++++ Update for static features +++++++++++++++++++++++++++++++
    // ---------------------------------------------------------------------------------------

    std::vector<cv::KeyPoint> mvKeysTmp;
    std::vector<cv::KeyPoint> mvCorresTmp;
    std::vector<cv::Point2f> mvFlowNextTmp;
    std::vector<int> StaInlierIDTmp;

    // (1) Save the inliers from last frame
    for (int i = 0; i < TM_sta.size(); ++i)
    {
        if (TM_sta[i]==-1)
            continue;

        int x = mCurrentFrame.mvSiftKeys[TM_sta[i]].pt.x;
        int y = mCurrentFrame.mvSiftKeys[TM_sta[i]].pt.y;

        if (x>=mImGrayLast.cols || y>=mImGrayLast.rows || x<=0 || y<=0)
            continue;

        if (mSegMap.at<int>(y,x)!=0)
            continue;

        if (mDepthMap.at<float>(y,x)>40 || mDepthMap.at<float>(y,x)<=0)
            continue;

        float flow_xe = mFlowMap.at<cv::Vec2f>(y,x)[0];
        float flow_ye = mFlowMap.at<cv::Vec2f>(y,x)[1];

        if(flow_xe!=0 && flow_ye!=0)
        {
            if(mCurrentFrame.mvSiftKeys[TM_sta[i]].pt.x+flow_xe < mImGrayLast.cols && mCurrentFrame.mvSiftKeys[TM_sta[i]].pt.y+flow_ye < mImGrayLast.rows && mCurrentFrame.mvSiftKeys[TM_sta[i]].pt.x < mImGrayLast.cols && mCurrentFrame.mvSiftKeys[TM_sta[i]].pt.y < mImGrayLast.rows)
            {
                mvKeysTmp.push_back(mCurrentFrame.mvSiftKeys[TM_sta[i]]);
                mvCorresTmp.push_back(cv::KeyPoint(mCurrentFrame.mvSiftKeys[TM_sta[i]].pt.x+flow_xe,mCurrentFrame.mvSiftKeys[TM_sta[i]].pt.y+flow_ye,0,0,0,-1));
                mvFlowNextTmp.push_back(cv::Point2f(flow_xe,flow_ye));
                StaInlierIDTmp.push_back(TM_sta[i]);
            }
        }
    }

    // cout << "accumulate static inlier number in: " << mvKeysTmp.size() << endl;

    // (2) Save extra key points to make it a fixed number (max = 1000)
    int tot_num = mvKeysTmp.size(), max_num_sta = 800, start_id = 0, step = 20;
    while (tot_num<max_num_sta)
    {
        // start id > step number, then stop
        if (start_id==step)
            break;

        for (int i = start_id; i < mCurrentFrame.mvKeys.size(); i=i+step)
        {
            // check if this key point is already been used
            float min_dist = 100;
            bool used = false;
            for (int j = 0; j < mvKeysTmp.size(); ++j)
            {
                float cur_dist = std::sqrt( (mvKeysTmp[j].pt.x-mCurrentFrame.mvKeys[i].pt.x)*(mvKeysTmp[j].pt.x-mCurrentFrame.mvKeys[i].pt.x) + (mvKeysTmp[j].pt.y-mCurrentFrame.mvKeys[i].pt.y)*(mvKeysTmp[j].pt.y-mCurrentFrame.mvKeys[i].pt.y) );
                if (cur_dist<min_dist)
                    min_dist = cur_dist;
                if (min_dist<1.0)
                {
                    used = true;
                    break;
                }
            }
            if (used)
                continue;

            int x = mCurrentFrame.mvKeys[i].pt.x;
            int y = mCurrentFrame.mvKeys[i].pt.y;

            if (x>=mImGrayLast.cols || y>=mImGrayLast.rows || x<=0 || y<=0)
                continue;

            if (mSegMap.at<int>(y,x)!=0)
                continue;

            if (mDepthMap.at<float>(y,x)>40 || mDepthMap.at<float>(y,x)<=0)
                continue;

            float flow_xe = mFlowMap.at<cv::Vec2f>(y,x)[0];
            float flow_ye = mFlowMap.at<cv::Vec2f>(y,x)[1];

            if(flow_xe!=0 && flow_ye!=0)
            {
                if(mCurrentFrame.mvKeys[i].pt.x+flow_xe < mImGrayLast.cols && mCurrentFrame.mvKeys[i].pt.y+flow_ye < mImGrayLast.rows && mCurrentFrame.mvKeys[i].pt.x < mImGrayLast.cols && mCurrentFrame.mvKeys[i].pt.y < mImGrayLast.rows)
                {
                    mvKeysTmp.push_back(mCurrentFrame.mvKeys[i]);
                    mvCorresTmp.push_back(cv::KeyPoint(mCurrentFrame.mvKeys[i].pt.x+flow_xe,mCurrentFrame.mvKeys[i].pt.y+flow_ye,0,0,0,-1));
                    mvFlowNextTmp.push_back(cv::Point2f(flow_xe,flow_ye));
                    StaInlierIDTmp.push_back(-1);
                    tot_num = tot_num + 1;
                }
            }

            if (tot_num>=max_num_sta)
                break;
        }
        start_id = start_id + 1;
    }

    mCurrentFrame.N_s_tmp = mvKeysTmp.size();

    // (3) assign the depth value to each key point
    std::vector<float> mvDepthTmp(mCurrentFrame.N_s_tmp,-1);
    for(int i=0; i<mCurrentFrame.N_s_tmp; i++)
    {
        const cv::KeyPoint &kp = mvKeysTmp[i];

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        float d = mDepthMap.at<float>(v,u); // be careful with the order  !!!

        if(d>0)
            mvDepthTmp[i] = d;
    }

    // (4) create 3d point based on key point, depth and pose
    std::vector<cv::Mat> mv3DPointTmp(mCurrentFrame.N_s_tmp);
    for (int i = 0; i < mCurrentFrame.N_s_tmp; ++i)
    {
        mv3DPointTmp[i] = Optimizer::Get3DinWorld(mvKeysTmp[i], mvDepthTmp[i], mK, InvMatrix(mCurrentFrame.mTcw));
    }

    // Obtain inlier ID
    mCurrentFrame.nStaInlierID = StaInlierIDTmp;

    // Update
    mCurrentFrame.mvSiftKeysTmp = mvKeysTmp;
    mCurrentFrame.mvSiftDepthTmp = mvDepthTmp;
    mCurrentFrame.mvSift3DPointTmp = mv3DPointTmp;
    mCurrentFrame.mvFlowNext = mvFlowNextTmp;
    mCurrentFrame.mvCorres = mvCorresTmp;

    // cout << "updating STATIC features finished...... " << mvKeysTmp.size() << endl;

    // ---------------------------------------------------------------------------------------
    // ++++++++++++++++++++++++++++ Update for Dynamic Object Features +++++++++++++++++++++++
    // ---------------------------------------------------------------------------------------

    std::vector<cv::KeyPoint> mvObjKeysTmp;
    std::vector<float> mvObjDepthTmp;
    std::vector<cv::KeyPoint> mvObjCorresTmp;
    std::vector<cv::Point2f> mvObjFlowNextTmp;
    std::vector<int> vSemObjLabelTmp;
    std::vector<int> DynInlierIDTmp;
    std::vector<int> vObjLabelTmp;

    // (1) Again, save the inliers from last frame
    std::vector<std::vector<int> > ObjInlierSet = mCurrentFrame.vnObjInlierID;
    std::vector<int> ObjFeaCount(ObjInlierSet.size());
    for (int i = 0; i < ObjInlierSet.size(); ++i)
    {
        int count = 0;
        for (int j = 0; j < ObjInlierSet[i].size(); ++j)
        {
            const int x = mCurrentFrame.mvObjKeys[ObjInlierSet[i][j]].pt.x;
            const int y = mCurrentFrame.mvObjKeys[ObjInlierSet[i][j]].pt.y;

            if (x>=mImGrayLast.cols || y>=mImGrayLast.rows || x<=0 || y<=0)
                continue;

            if (mSegMap.at<int>(y,x)!=0 && mDepthMap.at<float>(y,x)<25 && mDepthMap.at<float>(y,x)>0)
            {
                const float flow_x = mFlowMap.at<cv::Vec2f>(y,x)[0];
                const float flow_y = mFlowMap.at<cv::Vec2f>(y,x)[1];

                if (x+flow_x < mImGrayLast.cols && y+flow_y < mImGrayLast.rows)
                {
                    mvObjKeysTmp.push_back(cv::KeyPoint(x,y,0,0,0,-1));
                    mvObjDepthTmp.push_back(mDepthMap.at<float>(y,x));
                    vSemObjLabelTmp.push_back(mSegMap.at<int>(y,x));
                    mvObjFlowNextTmp.push_back(cv::Point2f(flow_x,flow_y));
                    mvObjCorresTmp.push_back(cv::KeyPoint(x+flow_x,y+flow_y,0,0,0,-1));
                    DynInlierIDTmp.push_back(ObjInlierSet[i][j]);
                    vObjLabelTmp.push_back(mCurrentFrame.vObjLabel[ObjInlierSet[i][j]]);
                    count = count + 1;
                }
            }
        }
        ObjFeaCount[i] = count;
        // cout << "accumulate dynamic inlier number: " << ObjFeaCount[i] << endl;
    }


    // (2) Save extra key points to make each object having a fixed number (max = 1000)
    std::vector<std::vector<int> > ObjSet = mCurrentFrame.vnObjID;
    for (int i = 0; i < ObjSet.size(); ++i)
    {
        int SemLabel = mCurrentFrame.nSemPosition[i];
        int tot_num = ObjFeaCount[i];
        int start_id = 0, step = 15, max_num_obj = 400;
        while (tot_num<max_num_obj)
        {
            // start id > step number, then stop
            if (start_id==step){
                // cout << "run on all the original objset... tot_num: " << tot_num << endl;
                break;
            }

            for (int j = start_id; j < mvTmpSemObjLabel.size(); j=j+step)
            {
                // check the semantic label if it is the same
                if (mvTmpSemObjLabel[j]!=SemLabel)
                    continue;

                // check if this key point is already been used
                float min_dist = 100;
                bool used = false;
                for (int k = 0; k < mvObjKeysTmp.size(); ++k)
                {
                    float cur_dist = std::sqrt( (mvObjKeysTmp[k].pt.x-mvTmpObjKeys[j].pt.x)*(mvObjKeysTmp[k].pt.x-mvTmpObjKeys[j].pt.x) + (mvObjKeysTmp[k].pt.y-mvTmpObjKeys[j].pt.y)*(mvObjKeysTmp[k].pt.y-mvTmpObjKeys[j].pt.y) );
                    if (cur_dist<min_dist)
                        min_dist = cur_dist;
                    if (min_dist<1.0)
                    {
                        used = true;
                        break;
                    }
                }
                if (used)
                    continue;

                // save the found one
                mvObjKeysTmp.push_back(mvTmpObjKeys[j]);
                mvObjDepthTmp.push_back(mvTmpObjDepth[j]);
                vSemObjLabelTmp.push_back(mvTmpSemObjLabel[j]);
                mvObjFlowNextTmp.push_back(mvTmpObjFlowNext[j]);
                mvObjCorresTmp.push_back(mvTmpObjCorres[j]);
                DynInlierIDTmp.push_back(-1);
                vObjLabelTmp.push_back(mCurrentFrame.nModLabel[i]);
                tot_num = tot_num + 1;

                if (tot_num>=max_num_obj){
                    // cout << "reach max_num_obj... tot_num: " << tot_num << endl;
                    break;
                }
            }
            start_id = start_id + 1;
        }

    }

    // (3) Update new appearing objects
    // (3.1) find the unique labels in semantic label
    auto UniLab = mvTmpSemObjLabel;
    std::sort(UniLab.begin(), UniLab.end());
    UniLab.erase(std::unique( UniLab.begin(), UniLab.end() ), UniLab.end() );
    // (3.2) find new appearing label
    std::vector<bool> NewLab(UniLab.size(),false);
    for (int i = 0; i < mCurrentFrame.nSemPosition.size(); ++i)
    {
        int CurSemLabel = mCurrentFrame.nSemPosition[i];
        for (int j = 0; j < UniLab.size(); ++j)
        {
            if (UniLab[j]==CurSemLabel)
            {
                NewLab[j] = true;
                break;
            }
        }

    }
    // (3.3) add the new object key points
    for (int i = 0; i < NewLab.size(); ++i)
    {
        if (NewLab[i]==false)
        {
            for (int j = 0; j < mvTmpSemObjLabel.size(); j++)
            {
                if (UniLab[i]==mvTmpSemObjLabel[j])
                {
                    // save the found one
                    mvObjKeysTmp.push_back(mvTmpObjKeys[j]);
                    mvObjDepthTmp.push_back(mvTmpObjDepth[j]);
                    vSemObjLabelTmp.push_back(mvTmpSemObjLabel[j]);
                    mvObjFlowNextTmp.push_back(mvTmpObjFlowNext[j]);
                    mvObjCorresTmp.push_back(mvTmpObjCorres[j]);
                    DynInlierIDTmp.push_back(-1);
                    vObjLabelTmp.push_back(-2);
                }
            }
        }
    }

    // (4) create 3d point based on key point, depth and pose
    std::vector<cv::Mat> mvObj3DPointTmp(mvObjKeysTmp.size());
    for (int i = 0; i < mvObjKeysTmp.size(); ++i)
    {
        mvObj3DPointTmp[i] = Optimizer::Get3DinWorld(mvObjKeysTmp[i], mvObjDepthTmp[i], mK, InvMatrix(mCurrentFrame.mTcw));
    }


    // update
    mCurrentFrame.mvObjKeys = mvObjKeysTmp;
    mCurrentFrame.mvObjDepth = mvObjDepthTmp;
    mCurrentFrame.mvObj3DPoint = mvObj3DPointTmp;
    mCurrentFrame.mvObjCorres = mvObjCorresTmp;
    mCurrentFrame.mvObjFlowNext = mvObjFlowNextTmp;
    mCurrentFrame.vSemObjLabel = vSemObjLabelTmp;
    mCurrentFrame.nDynInlierID = DynInlierIDTmp;
    mCurrentFrame.vObjLabel = vObjLabelTmp;

    // cout << "updating DYNAMIC features finished...... " << mvObjKeysTmp.size() << endl;

}

void Tracking::UpdateMask()
{
    // find the unique labels in semantic label
    auto UniLab = mLastFrame.vSemObjLabel;
    std::sort(UniLab.begin(), UniLab.end());
    UniLab.erase(std::unique( UniLab.begin(), UniLab.end() ), UniLab.end() );

    // collect the predicted labels and semantic labels in vector
    std::vector<std::vector<int> > ObjID(UniLab.size());
    for (int i = 0; i < mLastFrame.vSemObjLabel.size(); ++i)
    {
        // save object label
        for (int j = 0; j < UniLab.size(); ++j)
        {
            if(mLastFrame.vSemObjLabel[i]==UniLab[j]){
                ObjID[j].push_back(i);
                break;
            }
        }
    }

    // check each object label distribution in the coming frame
    for (int i = 0; i < ObjID.size(); ++i)
    {
        // collect labels
        std::vector<int> LabTmp;
        for (int j = 0; j < ObjID[i].size(); ++j)
        {
            const int u = mLastFrame.mvObjCorres[ObjID[i][j]].pt.x;
            const int v = mLastFrame.mvObjCorres[ObjID[i][j]].pt.y;
            if (u<mImGray.cols && u>0 && v<mImGray.rows && v>0)
            {
                LabTmp.push_back(mSegMap.at<int>(v,u));
            }
        }

        // find label that appears most in LabTmp()
        // (1) count duplicates
        std::map<int, int> dups;
        for(int k : LabTmp)
            ++dups[k];
        // (2) and sort them by descending order
        std::vector<std::pair<int, int> > sorted;
        for (auto k : dups)
            sorted.push_back(std::make_pair(k.first,k.second));
        std::sort(sorted.begin(), sorted.end(), sort_pair_int);

        // recover the missing mask (time consuming!)
        if (sorted[0].first==0) // && sorted[0].second==LabTmp.size()
        {
            for (int j = 0; j < mImGrayLast.rows; j++)
            {
                for (int k = 0; k < mImGrayLast.cols; k++)
                {
                    if (mSegMapLast.at<int>(j,k)==UniLab[i])
                    {
                        const int flow_x = mFlowMapLast.at<cv::Vec2f>(j,k)[0];
                        const int flow_y = mFlowMapLast.at<cv::Vec2f>(j,k)[1];

                        if(k+flow_x < mImGrayLast.cols && k+flow_x > 0 && j+flow_y < mImGrayLast.cols && j+flow_y > 0)
                            mSegMap.at<int>(j+flow_y,k+flow_x) = UniLab[i];
                    }
                }
            }
        }
        // end of recovery
    }
}

void Tracking::GetMetricError(const std::vector<cv::Mat> &CamPose, const std::vector<std::vector<cv::Mat> > &RigMot,
                    const std::vector<cv::Mat> &CamPose_gt, const std::vector<std::vector<cv::Mat> > &RigMot_gt)
{
    // absolute trajectory error for CAMERA (RMSE)
    cout << "=================================================" << endl;

    cout << "CAMERA:" << endl;
    float t_sum = 0, r_sum = 0;
    for (int i = 1; i < CamPose.size(); ++i)
    {
        // cv::Mat T_lc_inv = CamPose[i]*Converter::toInvMatrix(CamPose[i-1]);
        // cv::Mat T_lc_gt = CamPose_gt[i-1]*Converter::toInvMatrix(CamPose_gt[i]);
        // cv::Mat ate_cam = T_lc_inv*T_lc_gt;
        cv::Mat ate_cam = CamPose[i]*Converter::toInvMatrix(CamPose_gt[i]);

        // translation
        float t_ate_cam = std::sqrt(ate_cam.at<float>(0,3)*ate_cam.at<float>(0,3) + ate_cam.at<float>(1,3)*ate_cam.at<float>(1,3) + ate_cam.at<float>(2,3)*ate_cam.at<float>(2,3));
        t_sum = t_sum + t_ate_cam;

        // rotation
        float trace_ate = 0;
        for (int j = 0; j < 3; ++j)
        {
            if (ate_cam.at<float>(j,j)>1.0)
                trace_ate = trace_ate + 1.0-(ate_cam.at<float>(j,j)-1.0);
            else
                trace_ate = trace_ate + ate_cam.at<float>(j,j);
        }
        float r_ate_cam = acos( (trace_ate -1.0)/2.0 )*180.0/3.1415926;
        r_sum = r_sum + r_ate_cam;
        // cout << " t: " << t_ate_cam << " R: " << r_ate_cam << endl;
    }
    // t_mean = std::sqrt(t_sum/(CamPose.size()-1));
    t_sum = t_sum/(CamPose.size()-1);
    r_sum = r_sum/(CamPose.size()-1);
    cout << "average error (Camera):" << " t: " << t_sum << " R: " << r_sum << endl;

    cout << "OBJECTS:" << endl;
    // all motion error for objects (mean error)
    float r_rpe_sum = 0, t_rpe_sum = 0, obj_count = 0;
    for (int i = 0; i < RigMot.size(); ++i)
    {
        if (RigMot[i].size()>1)
        {
            for (int j = 1; j < RigMot[i].size(); ++j)
            {
                cv::Mat rpe_obj = Converter::toInvMatrix(RigMot[i][j])*RigMot_gt[i][j];

                // translation error
                float t_rpe_obj = std::sqrt( rpe_obj.at<float>(0,3)*rpe_obj.at<float>(0,3) + rpe_obj.at<float>(1,3)*rpe_obj.at<float>(1,3) + rpe_obj.at<float>(2,3)*rpe_obj.at<float>(2,3) );
                t_rpe_sum = t_rpe_sum + t_rpe_obj;

                // rotation error
                float trace_rpe = 0;
                for (int k = 0; k < 3; ++k)
                {
                    if (rpe_obj.at<float>(k,k)>1.0)
                        trace_rpe = trace_rpe + 1.0-(rpe_obj.at<float>(k,k)-1.0);
                    else
                        trace_rpe = trace_rpe + rpe_obj.at<float>(k,k);
                }
                float r_rpe_obj = acos( ( trace_rpe -1.0 )/2.0 )*180.0/3.1415926;
                r_rpe_sum = r_rpe_sum + r_rpe_obj;

                // cout << "(" << j-1 << ")" << " t: " << t_rpe_obj << " R: " << r_rpe_obj << endl;
                obj_count++;
            }
        }
    }
    t_rpe_sum = t_rpe_sum/obj_count;
    r_rpe_sum = r_rpe_sum/obj_count;
    cout << "average error (Objects):" << " t: " << t_rpe_sum << " R: " << r_rpe_sum << endl;

    cout << "=================================================" << endl << endl;

}

// ---------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------

} //namespace ORB_SLAM
