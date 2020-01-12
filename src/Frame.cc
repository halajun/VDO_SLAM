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

#include <opencv2/xfeatures2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include "cudasift/cudaImage.h"
// #include "cudasift/cudaSift.h"

#include <opencv2/flann.hpp>

#include "Frame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include <thread>
#include<time.h>
#include<chrono>

namespace ORB_SLAM2
{

long unsigned int Frame::nNextId=0;
bool Frame::mbInitialComputations=true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

// cv::RNG rng;

Frame::Frame()
{}

//Copy Constructor
Frame::Frame(const Frame &frame)
    :mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
     mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
     mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
     mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn),  mvuRight(frame.mvuRight),
     mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
     mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
     mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId),
     mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
     mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
     mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
     mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2),
     // new added
     mTcw_gt(frame.mTcw_gt), vObjPose_gt(frame.vObjPose_gt), nSemPosi_gt(frame.nSemPosi_gt), vObjBox_gt(frame.vObjBox_gt),
     vObjLabel(frame.vObjLabel),
     nModLabel(frame.nModLabel), nSemPosition(frame.nSemPosition), vObjMod(frame.vObjMod),
     mvCorres(frame.mvCorres), mvObjCorres(frame.mvObjCorres),
     mvFlowNext(frame.mvFlowNext), mvObjFlowNext(frame.mvObjFlowNext)
{
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j]=frame.mGrid[i][j];

    if(!frame.mTcw.empty())
        SetPose(frame.mTcw);
}


Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const cv::Mat &imMask, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
     mpReferenceKF(static_cast<KeyFrame*>(NULL))
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    thread threadLeft(&Frame::ExtractORB,this,0,imLeft);
    thread threadRight(&Frame::ExtractORB,this,1,imRight);
    threadLeft.join();
    threadRight.join();

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    // cv::Mat img_show;
    // cv::drawKeypoints(imLeft, mvKeys, img_show, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT); // cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
    // cv::imshow("ORB Detection", img_show);
    // cv::waitKey(0);

    // ---------------------------------------------------------------------------------------
    // ++++++++++++++++++++++++++++ New added ++++++++++++++++++++++++++++
    // ---------------------------------------------------------------------------------------

    // Assign the semantic label to each extracted feature
    vSemLabel.resize(N);
    int u, v;
    for (int i = 0; i < N; ++i)
    {
        u = mvKeys[i].pt.x;
        v = mvKeys[i].pt.y;
        // cout << "mvKeys: " << u << " " << v << endl;
        vSemLabel[i] = imMask.at<int>(v,u);
    }

    // ---------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------

    UndistortKeyPoints();

    ComputeStereoMatches();


    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);


    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imLeft);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}

Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const cv::Mat &imFlow, const cv::Mat &maskSEM,
    const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{

    cout << "Start Constructing Frame......" << endl;

    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    ExtractORB(0,imGray);

    // used for adding noise
    cv::RNG rng((unsigned)time(NULL));

    // ---------------------------------------------------------------------------------------
    // ++++++++++++++++++++++++++++ New added for dense object features ++++++++++++++++++++++
    // ---------------------------------------------------------------------------------------

    // semi-dense features on objects
    int step = 4; // 3
    for (int i = 0; i < imGray.rows; i=i+step)
    {
        for (int j = 0; j < imGray.cols; j=j+step)
        {

            // check ground truth motion mask
            if (maskSEM.at<int>(i,j)!=0 && imDepth.at<float>(i,j)<25 && imDepth.at<float>(i,j)>0)
            {
                // get flow
                const float flow_x = imFlow.at<cv::Vec2f>(i,j)[0];
                const float flow_y = imFlow.at<cv::Vec2f>(i,j)[1];

                if(j+flow_x < imGray.cols && j+flow_x > 0 && i+flow_y < imGray.rows && i+flow_y > 0)
                {
                    // save correspondences
                    mvObjFlowNext.push_back(cv::Point2f(flow_x,flow_y));
                    mvObjCorres.push_back(cv::KeyPoint(j+flow_x,i+flow_y,0,0,0,-1));
                    // save pixel location
                    mvObjKeys.push_back(cv::KeyPoint(j,i,0,0,0,-1));
                    // save depth
                    mvObjDepth.push_back(imDepth.at<float>(i,j));
                    // save label
                    vSemObjLabel.push_back(maskSEM.at<int>(i,j));
                }
            }
        }
    }

    // ---------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------

    // ---------------------------------------------------------------------------------------
    // ++++++++++++++++++++++++++++ New added for sampled features ++++++++++++++++++++++++++++
    // ---------------------------------------------------------------------------------------

    // int fal_ma = 0, pos_ma = 0;
    // float e_sum = 0;
    for (int i = 0; i < mvKeys.size(); ++i)
    {
        // inliers
        if (1) // i%2==0
        {
            int x = mvKeys[i].pt.x;
            int y = mvKeys[i].pt.y;

            if (maskSEM.at<int>(y,x)!=0)  // new added in Jun 13 2019
                continue;

            if (imDepth.at<float>(y,x)>40 || imDepth.at<float>(y,x)<=0)  // new added in Aug 21 2019
                continue;

            // float flow_x = imFlow.at<cv::Vec2f>(y,x)[0];
            // float flow_y = imFlow.at<cv::Vec2f>(y,x)[1];
            float flow_xe = imFlow.at<cv::Vec2f>(y,x)[0];
            float flow_ye = imFlow.at<cv::Vec2f>(y,x)[1];
            // float x_ = flow_x-flow_xe;
            // float y_ = flow_y-flow_ye;
            // e_sum = e_sum + std::sqrt(x_*x_ + y_*y_);


            if(flow_xe!=0 && flow_ye!=0)
            {
                if(mvKeys[i].pt.x+flow_xe < imGray.cols && mvKeys[i].pt.y+flow_ye < imGray.rows && mvKeys[i].pt.x < imGray.cols && mvKeys[i].pt.y < imGray.rows)
                {
                    mvSiftKeysTmp.push_back(mvKeys[i]);
                    mvCorres.push_back(cv::KeyPoint(mvKeys[i].pt.x+flow_xe,mvKeys[i].pt.y+flow_ye,0,0,0,mvKeys[i].octave,-1));
                    mvFlowNext.push_back(cv::Point2f(flow_xe,flow_ye));
                    // vCorSta.push_back(1);
                    // pos_ma = pos_ma + 1;
                }
                // cout << "flow vector: " << flow_x << " " << flow_y << endl;
                // cout << "key point: " << mvKeys[i].pt.x << " " << mvKeys[i].pt.y<< endl;
                // cout << "new key: " << mvKeys[i].pt.x+flow_x << " " << mvKeys[i].pt.y+flow_y << endl;
            }
        }
    }
    // cout << "the inliers and outliers number: " << pos_ma << " " << fal_ma << endl;

    // cout << "AVERAGE FLOW ERROR: " << e_sum/mvCorres.size() << endl;

    N_s_tmp = mvCorres.size();
    // cout << "number of random sample points: " << mvCorres.size() << endl;

    // assign the depth value to each sift keypoint
    mvSiftDepthTmp = vector<float>(N_s_tmp,-1);
    for(int i=0; i<N_s_tmp; i++)
    {
        const cv::KeyPoint &kp = mvSiftKeysTmp[i];

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        float d = imDepth.at<float>(v,u); // be careful with the order  !!!

        if(d>0)
            mvSiftDepthTmp[i] = d;
    }

    // Assign the semantic label to each extracted feature
    // vSemLabelTmp.resize(N_s_tmp,-1);
    // vObjLabel_gtTmp.resize(N_s_tmp,-1);
    // int u, v;
    // // cout << maskSEM.cols << " " << maskSEM.rows << endl;
    // for (int i = 0; i < N_s_tmp; ++i)
    // {
    //     u = mvSiftKeysTmp[i].pt.x;
    //     v = mvSiftKeysTmp[i].pt.y;

    //     if (u==maskSEM.cols || v==maskSEM.rows)
    //     {
    //         cout << "boundary warning ! ! !" << endl;
    //         continue;
    //     }

    //     // cout << maskSEM.at<int>(v,u) << " ";

    //     vSemLabelTmp[i] = maskSEM.at<int>(v,u);  // be careful with the order  !!!
    // }
    // // cout << endl;

    // ---------------------------------------------------------------------------------------
    // ---------------------------------------------------------------------------------------

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    ComputeStereoFromRGBD(imDepth);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();

    cout << "Done!" << endl;
}

// Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const cv::Mat &imFlow, const cv::Mat &maskSEM,
//     const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth,
//     Frame &mLastFrame)
//     :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
//      mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
// {
//     // Frame ID
//     mnId=nNextId++;

//     // Scale Level Info
//     mnScaleLevels = mpORBextractorLeft->GetLevels();
//     mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
//     mfLogScaleFactor = log(mfScaleFactor);
//     mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
//     mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
//     mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
//     mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

//     // ORB extraction
//     ExtractORB(0,imGray);

//     // used for adding noise
//     cv::RNG rng((unsigned)time(NULL));

//     // ---------------------------------------------------------------------------------------
//     // ++++++++++++++++++++++++++++ New added for dense object features ++++++++++++++++++++++
//     // ---------------------------------------------------------------------------------------

//     // semi-dense features on objects
//     int step = 3;
//     for (int i = 0; i < imGray.rows; i=i+step)
//     {
//         for (int j = 0; j < imGray.cols; j=j+step)
//         {

//             // check ground truth motion mask
//             if (maskSEM.at<int>(i,j)!=0 && imDepth.at<float>(i,j)<25 && imDepth.at<float>(i,j)>0)
//             {
//                 // get flow
//                 const float flow_x = imFlow.at<cv::Vec2f>(i,j)[0];
//                 const float flow_y = imFlow.at<cv::Vec2f>(i,j)[1];

//                 if(j+flow_x < imGray.cols && j+flow_x > 0 && i+flow_y < imGray.rows && i+flow_y > 0)
//                 {
//                     // save correspondences
//                     mvObjFlowNext.push_back(cv::Point2f(flow_x,flow_y));
//                     mvObjCorres.push_back(cv::KeyPoint(j+flow_x,i+flow_y,0,0,0,-1));
//                     // save pixel location
//                     mvObjKeys.push_back(cv::KeyPoint(j,i,0,0,0,-1));
//                     // save depth
//                     mvObjDepth.push_back(imDepth.at<float>(i,j));
//                     // save label
//                     vSemObjLabel.push_back(maskSEM.at<int>(i,j));
//                 }

//             }
//         }
//     }

//     // ---------------------------------------------------------------------------------------
//     // ---------------------------------------------------------------------------------------

//     // ---------------------------------------------------------------------------------------
//     // ++++++++++++++++++++++++++++ New added for sampled features ++++++++++++++++++++++++++++
//     // ---------------------------------------------------------------------------------------

//     // (1) Save the inliers from last frame
//     int inl_num = InlierLastFrame.size();
//     for (int i = 0; i < inl_num; ++i)
//     {
//         int x = InlierLastFrame[i].pt.x;
//         int y = InlierLastFrame[i].pt.y;

//         if (maskSEM.at<int>(y,x)!=0)
//             continue;

//         if (imDepth.at<float>(y,x)>40 || imDepth.at<float>(y,x)<=0)
//             continue;

//         float flow_xe = imFlow.at<cv::Vec2f>(y,x)[0];
//         float flow_ye = imFlow.at<cv::Vec2f>(y,x)[1];

//         if(flow_xe!=0 && flow_ye!=0)
//         {
//             if(InlierLastFrame[i].pt.x+flow_xe < imGray.cols && InlierLastFrame[i].pt.y+flow_ye < imGray.rows && InlierLastFrame[i].pt.x < imGray.cols && InlierLastFrame[i].pt.y < imGray.rows)
//             {
//                 mvSiftKeysTmp.push_back(InlierLastFrame[i]);
//                 mvCorres.push_back(cv::KeyPoint(InlierLastFrame[i].pt.x+flow_xe,InlierLastFrame[i].pt.y+flow_ye,0,0,0,-1));
//                 mvFlowNext.push_back(cv::Point2f(flow_xe,flow_ye));

//             }
//         }
//     }

//     // (2) Save extra key points to make it a fixed number (max = 1000)
//     int tot_num = mvSiftKeysTmp.size();
//     while (tot_num<1000)
//     {
//         int i = rng.uniform((int)0,(int)mvKeys.size());

//         // check if this key point is already been used
//         float min_dist = 100;
//         bool used = false;
//         for (int j = 0; j < mvSiftKeysTmp.size(); ++j)
//         {
//             float cur_dist = std::sqrt( (mvSiftKeysTmp[j].pt.x-mvKeys[i].pt.x)*(mvSiftKeysTmp[j].pt.x-mvKeys[i].pt.x) + (mvSiftKeysTmp[j].pt.y-mvKeys[i].pt.y)*(mvSiftKeysTmp[j].pt.y-mvKeys[i].pt.y) )
//             if (cur_dist<min_dist)
//                 min_dist = cur_dist;
//             if (min_dist<1.0)
//             {
//                 used = true;
//                 break;
//             }
//         }
//         if (used)
//             continue;

//         int x = mvKeys[i].pt.x;
//         int y = mvKeys[i].pt.y;

//         if (maskSEM.at<int>(y,x)!=0)
//             continue;

//         if (imDepth.at<float>(y,x)>40 || imDepth.at<float>(y,x)<=0)
//             continue;

//         float flow_xe = imFlow.at<cv::Vec2f>(y,x)[0];
//         float flow_ye = imFlow.at<cv::Vec2f>(y,x)[1];


//         if(flow_xe!=0 && flow_ye!=0)
//         {
//             if(mvKeys[i].pt.x+flow_xe < imGray.cols && mvKeys[i].pt.y+flow_ye < imGray.rows && mvKeys[i].pt.x < imGray.cols && mvKeys[i].pt.y < imGray.rows)
//             {
//                 mvSiftKeysTmp.push_back(mvKeys[i]);
//                 mvCorres.push_back(cv::KeyPoint(mvKeys[i].pt.x+flow_xe,mvKeys[i].pt.y+flow_ye,0,0,0,mvKeys[i].octave,-1));
//                 mvFlowNext.push_back(cv::Point2f(flow_xe,flow_ye));

//             }
//         }

//         tot_num = tot_num + 1;
//     }

//     N_s_tmp = mvCorres.size();

//     // assign the depth value to each sift keypoint
//     mvSiftDepthTmp = vector<float>(N_s_tmp,-1);
//     for(int i=0; i<N_s_tmp; i++)
//     {
//         const cv::KeyPoint &kp = mvSiftKeysTmp[i];

//         const float &v = kp.pt.y;
//         const float &u = kp.pt.x;

//         float d = imDepth.at<float>(v,u); // be careful with the order  !!!

//         if(d>0)
//             mvSiftDepthTmp[i] = d;
//     }

//     // ---------------------------------------------------------------------------------------
//     // ---------------------------------------------------------------------------------------

//     N = mvKeys.size();

//     if(mvKeys.empty())
//         return;

//     UndistortKeyPoints();

//     ComputeStereoFromRGBD(imDepth);

//     mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
//     mvbOutlier = vector<bool>(N,false);

//     // This is done only for the first Frame (or after a change in the calibration)
//     if(mbInitialComputations)
//     {
//         ComputeImageBounds(imGray);

//         mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
//         mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

//         fx = K.at<float>(0,0);
//         fy = K.at<float>(1,1);
//         cx = K.at<float>(0,2);
//         cy = K.at<float>(1,2);
//         invfx = 1.0f/fx;
//         invfy = 1.0f/fy;

//         mbInitialComputations=false;
//     }

//     mb = mbf/fx;

//     AssignFeaturesToGrid();
// }

Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    ExtractORB(0,imGray);

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    // Set no stereo information
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}

void Frame::AssignFeaturesToGrid()
{
    int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);

    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
}

void Frame::ExtractORB(int flag, const cv::Mat &im)
{
    if(flag==0){
        (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors);
        // cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SURF::create(400);
        // cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SIFT::create();
        // f2d->compute(im, mvKeys, mSift);
    }
    else{
        (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight);
        // cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SURF::create(400);
        // cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SIFT::create();
        // f2d->compute(im, mvKeysRight, mSiftRight);
    }
}

void Frame::SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
    // mTcw.at<float>(0,0)=Tcw.at<float>(0,0);mTcw.at<float>(0,1)=Tcw.at<float>(0,1);mTcw.at<float>(0,2)=Tcw.at<float>(0,2);mTcw.at<float>(0,3)=Tcw.at<float>(0,3);
    // mTcw.at<float>(1,0)=Tcw.at<float>(1,0);mTcw.at<float>(1,1)=Tcw.at<float>(1,1);mTcw.at<float>(1,2)=Tcw.at<float>(1,2);mTcw.at<float>(1,3)=Tcw.at<float>(1,3);
    // mTcw.at<float>(2,0)=Tcw.at<float>(2,0);mTcw.at<float>(2,1)=Tcw.at<float>(2,1);mTcw.at<float>(2,2)=Tcw.at<float>(2,2);mTcw.at<float>(2,3)=Tcw.at<float>(2,3);
    // mTcw.at<float>(3,0)=Tcw.at<float>(3,0);mTcw.at<float>(3,1)=Tcw.at<float>(3,1);mTcw.at<float>(3,2)=Tcw.at<float>(3,2);mTcw.at<float>(2,3)=Tcw.at<float>(3,3);
    UpdatePoseMatrices();
}

void Frame::UpdatePoseMatrices()
{
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0,3).col(3);
    mOw = -mRcw.t()*mtcw;
}

bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    pMP->mbTrackInView = false;

    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos();

    // 3D in camera coordinates
    const cv::Mat Pc = mRcw*P+mtcw;
    const float &PcX = Pc.at<float>(0);
    const float &PcY= Pc.at<float>(1);
    const float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0f)
        return false;

    // Project in image and check it is not outside
    const float invz = 1.0f/PcZ;
    const float u=fx*PcX*invz+cx;
    const float v=fy*PcY*invz+cy;

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const cv::Mat PO = P-mOw;
    const float dist = cv::norm(PO);

    if(dist<minDistance || dist>maxDistance)
        return false;

   // Check viewing angle
    cv::Mat Pn = pMP->GetNormal();

    const float viewCos = PO.dot(Pn)/dist;

    if(viewCos<viewingCosLimit)
        return false;

    // Predict scale in the image
    const int nPredictedLevel = pMP->PredictScale(dist,this);

    // Data used by the tracking
    pMP->mbTrackInView = true;
    pMP->mTrackProjX = u;
    pMP->mTrackProjXR = u - mbf*invz;
    pMP->mTrackProjY = v;
    pMP->mnTrackScaleLevel= nPredictedLevel;
    pMP->mTrackViewCos = viewCos;

    return true;
}

vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}


void Frame::ComputeBoW()
{
    if(mBowVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void Frame::UndistortKeyPoints()
{
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        return;
    }

    // Fill matrix with points
    cv::Mat mat(N,2,CV_32F);
    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);
    cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
    mvKeysUn.resize(N);
    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;
    }
}

void Frame::ComputeImageBounds(const cv::Mat &imLeft)
{
    if(mDistCoef.at<float>(0)!=0.0)
    {
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;
        mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows;

        // Undistort corners
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
        mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
        mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
        mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));

    }
    else
    {
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }
}

void Frame::ComputeStereoMatches()
{
    // output
    mvuRight = vector<float>(N,-1.0f);
    mvDepth = vector<float>(N,-1.0f);
    vDescIndex = vector<int>(N,-1);

    const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;
    // const double thOrbDist = 200; // for sift 100  for surf 0.4

    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

    // Assign keypoints to row table
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);

    const int Nr = mvKeysRight.size();

    for(int iR=0; iR<Nr; iR++)
    {
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY = kp.pt.y;
        const float r = 1.2f*mvScaleFactors[mvKeysRight[iR].octave];  // 2.98 3.58 4.29  // 4.97 5.97 7.16
        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);

        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }

    // Set limits for search
    // const float minZ = mb;
    const float minD = 0;   // 0
    const float maxD = 200;   // 120  mbf/minZ

    // For each left keypoint search a match in the right image
    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(N);

    for(int iL=0; iL<N; iL++)
    {
        const cv::KeyPoint &kpL = mvKeys[iL];


        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        const vector<size_t> &vCandidates = vRowIndices[vL];

        if(vCandidates.empty())
            continue;

        const float minU = uL-maxD;
        const float maxU = uL-minD;

        if(maxU<0)
            continue;

        int bestDist = ORBmatcher::TH_HIGH;
        // double bestDist = 200; // for sift 100 , for surf 0.4
        size_t bestIdxR = 0;

        const cv::Mat &dL = mDescriptors.row(iL);
        // const cv::Mat &dL = mSift.row(iL);

        // Compare descriptor to right keypoints
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            const float &uR = kpR.pt.x;

            if(uR>=minU && uR<=maxU)
            {
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);
                // const cv::Mat &dR = mSiftRight.row(iR);
                // const double dist = cv::norm(dL,dR,cv::NORM_L2);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        if(bestIdxR!=0)
            vDescIndex[iL] = bestIdxR;

        // cout << "best distance: " << bestDist << endl;

        // Subpixel match by correlation
        if(bestDist<thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            const float uR0 = mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);

            // sliding window search
            const int w = 5;
            cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
            IL.convertTo(IL,CV_32F);
            IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2*L+1);

            const float iniu = scaleduR0+L-w;
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            for(int incR=-L; incR<=+L; incR++)
            {
                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                IR.convertTo(IR,CV_32F);
                IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);

                float dist = cv::norm(IL,IR,cv::NORM_L1);
                if(dist<bestDist)
                {
                    bestDist =  dist;
                    bestincR = incR;
                }

                vDists[L+incR] = dist;
            }

            if(bestincR==-L || bestincR==L)
                continue;

            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

            float disparity = (uL-bestuR);

            if(disparity>=minD && disparity<maxD)
            {
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                mvDepth[iL]=mbf/disparity;
                mvuRight[iL] = bestuR;
                vDistIdx.push_back(pair<int,int>(bestDist,iL));
            }
        }
    }

    sort(vDistIdx.begin(),vDistIdx.end());
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median;

    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            mvuRight[vDistIdx[i].second]=-1;
            mvDepth[vDistIdx[i].second]=-1;
            vDescIndex[vDistIdx[i].second]=-1;  // new added
        }
    }
}


void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
{
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    for(int i=0; i<N; i++)
    {
        const cv::KeyPoint &kp = mvKeys[i];
        const cv::KeyPoint &kpU = mvKeysUn[i];

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        const float d = imDepth.at<float>(v,u); // be careful with the order

        if(d>0)
        {
            mvDepth[i] = d;
            mvuRight[i] = kpU.pt.x-mbf/d;
        }
    }
}

cv::Mat Frame::UnprojectStereo(const int &i)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeysUn[i].pt.x;
        const float v = mvKeysUn[i].pt.y;
        // cout << "xyz: " << u << " " << v << " " << z << endl;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
        return mRwc*x3Dc+mOw;
    }
    else
        return cv::Mat();
}

cv::Mat Frame::UnprojectStereoSift(const int &i, const bool &addnoise)
{
    float z = mvSiftDepth[i];

    // used for adding noise
    cv::RNG rng((unsigned)time(NULL));

    if(addnoise){
        z = z + rng.gaussian(z*z/(725*0.5)*0.15);  // sigma = z*0.01
        // z = z + 0.0;
    }

    if(z>0)
    {
        const float u = mvSiftKeys[i].pt.x;
        const float v = mvSiftKeys[i].pt.y;
        // cout << "xyz: " << u << " " << v << " " << z << endl;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

        // using ground truth
        const cv::Mat Rlw = mTcw.rowRange(0,3).colRange(0,3);
        const cv::Mat Rwl = Rlw.t();
        const cv::Mat tlw = mTcw.rowRange(0,3).col(3);
        const cv::Mat twl = -Rlw.t()*tlw;

        return Rwl*x3Dc+twl;
        // return mRwc*x3Dc+mOw;
    }
    else
    {
        cout << "found a depth value < 0 ..." << endl;
        return cv::Mat();
    }
}

cv::Mat Frame::UnprojectStereoObject(const int &i, const bool &addnoise)
{
    float z = mvObjDepth[i];

    // used for adding noise
    cv::RNG rng((unsigned)time(NULL));

    if(addnoise){
        float noise = rng.gaussian(z*z/(725*0.5)*0.15);
        z = z + noise;  // sigma = z*0.01
        // z = z + 0.0;
        // cout << "noise: " << noise << endl;
    }

    if(z>0)
    {
        const float u = mvObjKeys[i].pt.x;
        const float v = mvObjKeys[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

        // using ground truth
        const cv::Mat Rlw = mTcw.rowRange(0,3).colRange(0,3);
        const cv::Mat Rwl = Rlw.t();
        const cv::Mat tlw = mTcw.rowRange(0,3).col(3);
        const cv::Mat twl = -Rlw.t()*tlw;

        return Rwl*x3Dc+twl;
    }
    else{
        cout << "found a depth value < 0 ..." << endl;
        return cv::Mat();
    }
}

cv::Mat Frame::UnprojectStereoObjectCamera(const int &i, const bool &addnoise)
{
    float z = mvObjDepth[i];
    // cout << "depth check: " << z << endl;

    // used for adding noise
    cv::RNG rng((unsigned)time(NULL));

    if(addnoise){
        // sigma = z*0.01
        z = z + rng.gaussian(z*z/(725*0.5)*0.15);  // sigma = z*0.01
        // z = z + 0.0;
    }

    if(z>0)
    {
        const float u = mvObjKeys[i].pt.x;
        const float v = mvObjKeys[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

        return x3Dc;
    }
    else{
        cout << "found a depth value < 0 ..." << endl;
        return cv::Mat();
    }
}

cv::Mat Frame::UnprojectStereoObjectNoise(const int &i, const cv::Point2f of_error)
{
    float z = mvObjDepth[i];

    // if(addnoise){
    //     z = z + rng.gaussian(z*0.01);  // sigma = z*0.01
    // }

    if(z>0)
    {
        const float u = mvObjKeys[i].pt.x + of_error.x;
        const float v = mvObjKeys[i].pt.y + of_error.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

        // using ground truth
        const cv::Mat Rlw = mTcw.rowRange(0,3).colRange(0,3);
        const cv::Mat Rwl = Rlw.t();
        const cv::Mat tlw = mTcw.rowRange(0,3).col(3);
        const cv::Mat twl = -Rlw.t()*tlw;

        return Rwl*x3Dc+twl;
    }
    else{
        cout << "found a depth value < 0 ..." << endl;
        return cv::Mat();
    }
}

cv::Mat Frame::ObtainFlowDepthObject(const int &i, const bool &addnoise)
{
    float z = mvObjDepth[i];

    // used for adding noise
    cv::RNG rng((unsigned)time(NULL));

    if(addnoise){
        z = z + rng.gaussian(z*z/(725*0.5)*0.15);  // sigma = z*0.01 or z*z/(725*0.5)*0.12
        // z = z + 0.0;
    }

    if(z>0)
    {
        const float flow_u = mvObjFlowNext[i].x;
        const float flow_v = mvObjFlowNext[i].y;

        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << flow_u, flow_v, z);

        return x3Dc;
    }
    else{
        cout << "found a depth value < 0 ..." << endl;
        return cv::Mat();
    }
}

cv::Mat Frame::ObtainFlowDepthCamera(const int &i, const bool &addnoise)
{
    float z = mvSiftDepth[i];

    // used for adding noise
    cv::RNG rng((unsigned)time(NULL));

    if(addnoise){
        z = z + rng.gaussian(z*z/(725*0.5)*0.15);  // sigma = z*0.01 or z*z/(725*0.5)*0.12
        // z = z + 0.0;
    }

    if(z>0)
    {
        const float flow_u = mvFlowNext[i].x;
        const float flow_v = mvFlowNext[i].y;

        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << flow_u, flow_v, z);

        return x3Dc;
    }
    else
    {
        cout << "found a depth value < 0 ..." << endl;
        return cv::Mat();
    }
}


} //namespace ORB_SLAM
