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



#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>

#include <unistd.h>

namespace ORB_SLAM2
{

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer):mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false),mbActivateLocalizationMode(false),
               mbDeactivateLocalizationMode(false)
{
    // Output welcome message
    // cout << endl <<
    // "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    // "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    // "This is free software, and you are welcome to redistribute it" << endl <<
    // "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }


    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Map
    mpMap = new Map();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    //Initialize the Viewer thread and launch
    if(false) // bUseViewer
    {
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
    }

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const cv::Mat &imMask, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,imMask,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackRGBD(const cv::Mat &im, cv::Mat &depthmap, const cv::Mat &flowmap, const cv::Mat &masksem,
                          const cv::Mat &mTcw_gt, const vector<vector<float> > &vObjPose_gt,
                          const double &timestamp, cv::Mat &imTraj)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,flowmap,masksem,mTcw_gt,vObjPose_gt,timestamp,imTraj);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
        usleep(5000);
    }

    if(mpViewer)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");
}

void System::SaveResultsICRA2020(const string &filename)
{
    cout << endl << "Saving Results into TXT File..." << endl;

    // ************************************************************************************************
    // ***************************************** SAVE OBJ SPEED ***************************************
    // ************************************************************************************************

    // // object
    // std::vector<std::vector<int> >         ObjMotID    = mpMap->vvObjMotID;
    // std::vector<std::vector<cv::Point2f> > ObjMotErr_1 = mpMap->vvObjMotErr_1;
    // std::vector<std::vector<cv::Point2f> > ObjMotErr_2 = mpMap->vvObjMotErr_2;
    // std::vector<std::vector<cv::Point2f> > ObjMotErr_3 = mpMap->vvObjMotErr_3;

    // ofstream save_obj_1;
    // string strObj1 = filename + "obj_mot_err.txt";
    // save_obj_1.open(strObj1.c_str(),ios::trunc);
    // ofstream save_obj_2;
    // string strObj2 = filename + "obj_mot_err_convert.txt";
    // save_obj_2.open(strObj2.c_str(),ios::trunc);
    // ofstream save_obj_3;
    // string strObj3 = filename + "obj_speed_err.txt";
    // save_obj_3.open(strObj3.c_str(),ios::trunc);

    // // save object
    // for (int i = 0; i < ObjMotErr_3.size(); ++i)
    // {
    //     for (int j = 0; j < ObjMotErr_3[i].size(); ++j)
    //     {
    //         save_obj_1 << fixed << ObjMotID[i][j] << " " << setprecision(6) << ObjMotErr_1[i][j].x << " " << ObjMotErr_1[i][j].y << endl;
    //         save_obj_2 << fixed << ObjMotID[i][j] << " " << setprecision(6) << ObjMotErr_2[i][j].x << " " << ObjMotErr_2[i][j].y << endl;
    //         save_obj_3 << fixed << ObjMotID[i][j] << " " << setprecision(6) << ObjMotErr_3[i][j].x << " " << ObjMotErr_3[i][j].y << endl;
    //     }
    // }

    // // vectorize object information
    // std::vector<int> UniLab, vObjID;
    // std::vector<cv::Point2f> vObjSpeedErr;
    // for (int i = 0; i < ObjMotID.size(); ++i)
    // {
    //     for (int j = 0; j < ObjMotID[i].size(); ++j)
    //     {
    //         UniLab.push_back(ObjMotID[i][j]);
    //         vObjID.push_back(ObjMotID[i][j]);
    //         vObjSpeedErr.push_back(ObjMotErr_3[i][j]);
    //     }
    // }
    // std::sort(UniLab.begin(), UniLab.end());
    // UniLab.erase(std::unique( UniLab.begin(), UniLab.end() ), UniLab.end() );

    // // collect in object instance order
    // std::vector<std::vector<cv::Point2f> > vvObjSpeedErr(UniLab.size());
    // for (int i = 0; i < vObjID.size(); ++i)
    // {
    //     for (int j = 0; j < UniLab.size(); ++j)
    //     {
    //         if (vObjID[i]==UniLab[j])
    //         {
    //             vvObjSpeedErr[j].push_back(vObjSpeedErr[i]);
    //             break;
    //         }
    //     }
    // }

    // // show object
    // float obj_se_avg_all = 0;
    // for (int i = 0; i < vvObjSpeedErr.size(); ++i)
    // {
    //     float obj_se_avg = 0, obj_sp_avg = 0;
    //     for (int j = 0; j < vvObjSpeedErr[i].size(); ++j)
    //     {
    //         obj_se_avg   = obj_se_avg   + vvObjSpeedErr[i][j].x;
    //         obj_sp_avg   = obj_sp_avg   + vvObjSpeedErr[i][j].y;
    //         obj_se_avg_all = obj_se_avg_all + vvObjSpeedErr[i][j].x;
    //     }
    //     cout << "object " << UniLab[i] << " has " << vvObjSpeedErr[i].size() << " times of tracking" << endl;
    //     cout << "avg speed: " << obj_sp_avg/vvObjSpeedErr[i].size() << "km/h " << " avg speed error: " << obj_se_avg/vvObjSpeedErr[i].size()*100 << "%" << endl;
    // }
    // cout << "average speed error of all objects: " << obj_se_avg_all/vObjID.size()*100 << "% " << vObjID.size() << endl;

    // save_obj_1.close();
    // save_obj_2.close();
    // save_obj_3.close();

    // // dynamic object number
    // std::vector<int> TotObjNum = mpMap->vTotObjNum;

    // // dynamic object number
    // int TON = 0;
    // for (int i = 0; i < TotObjNum.size(); ++i)
    //     TON = TON + TotObjNum[i];
    // cout << "total object number: " << TON << endl;

    // ************************************************************************************************
    // ************************************************************************************************

    // // camera
    // std::vector<cv::Point2f> CamMotErr_1 = mpMap->vvCamMotErr_1;
    // std::vector<cv::Point2f> CamMotErr_2 = mpMap->vvCamMotErr_2;

    // // create txt file
    // ofstream save_cam_1;
    // string strCam1 = filename + "cam_1.txt";
    // save_cam_1.open(strCam1.c_str(),ios::trunc);
    // ofstream save_cam_2;
    // string strCam2 = filename + "cam_2.txt";
    // save_cam_2.open(strCam2.c_str(),ios::trunc);

    // // save camera
    // float cam_t_avg_1 = 0, cam_r_avg_1 = 0, cam_t_avg_2 = 0, cam_r_avg_2 = 0;
    // for (int i = 0; i < CamMotErr_1.size(); ++i)
    // {
    //     save_cam_1 << fixed << setprecision(6) << CamMotErr_1[i].x << " " << CamMotErr_1[i].y << endl;
    //     save_cam_2 << fixed << setprecision(6) << CamMotErr_2[i].x << " " << CamMotErr_2[i].y << endl;
    //     cam_t_avg_1 = cam_t_avg_1 + CamMotErr_1[i].x;
    //     cam_r_avg_1 = cam_r_avg_1 + CamMotErr_1[i].y;
    //     cam_t_avg_2 = cam_t_avg_2 + CamMotErr_2[i].x;
    //     cam_r_avg_2 = cam_r_avg_2 + CamMotErr_2[i].y;
    // }
    // cout << "camera motion error: " << cam_t_avg_1/CamMotErr_1.size() << " " << cam_r_avg_1/CamMotErr_1.size() << " " << cam_t_avg_2/CamMotErr_1.size()*100 << "% " << cam_r_avg_2/CamMotErr_1.size() << endl;

    // save_cam_1.close();
    // save_cam_2.close();

    // ************************************************************************************************
    // ***************************************** SAVE TRAJETORY ***************************************
    // ************************************************************************************************

    std::vector<cv::Mat> CamPose_main = mpMap->vmCameraPose_main;

    ofstream save_traj_main;
    string path_main = filename + "main_2.txt";
    save_traj_main.open(path_main.c_str(),ios::trunc);

    for (int i = 0; i < CamPose_main.size(); ++i)
    {
        save_traj_main << fixed << setprecision(9) << CamPose_main[i].at<float>(0,0) << " " << CamPose_main[i].at<float>(0,1)  << " " << CamPose_main[i].at<float>(0,2) << " "  << CamPose_main[i].at<float>(0,3) << " " <<
                          CamPose_main[i].at<float>(1,0) << " " << CamPose_main[i].at<float>(1,1)  << " " << CamPose_main[i].at<float>(1,2) << " "  << CamPose_main[i].at<float>(1,3) << " " <<
                          CamPose_main[i].at<float>(2,0) << " " << CamPose_main[i].at<float>(2,1)  << " " << CamPose_main[i].at<float>(2,2) << " "  << CamPose_main[i].at<float>(2,3) << " " <<
                          0.0 << " " << 0.0 << " " << 0.0 << " " << 1.0 << endl;
    }

    save_traj_main.close();

    // -------------------------------------------------------------------------------------------------

    // std::vector<cv::Mat> CamPose_orb = mpMap->vmCameraPose_orb;

    // ofstream save_traj_orb;
    // string path_orb = filename + "orb.txt";
    // save_traj_orb.open(path_orb.c_str(),ios::trunc);


    // for (int i = 0; i < CamPose_orb.size(); ++i)
    // {
    //     save_traj_orb  << fixed << setprecision(9) << CamPose_orb[i].at<float>(0,0) << " " << CamPose_orb[i].at<float>(0,1)  << " " << CamPose_orb[i].at<float>(0,2) << " "  << CamPose_orb[i].at<float>(0,3) << " " <<
    //                       CamPose_orb[i].at<float>(1,0) << " " << CamPose_orb[i].at<float>(1,1)  << " " << CamPose_orb[i].at<float>(1,2) << " "  << CamPose_orb[i].at<float>(1,3) << " " <<
    //                       CamPose_orb[i].at<float>(2,0) << " " << CamPose_orb[i].at<float>(2,1)  << " " << CamPose_orb[i].at<float>(2,2) << " "  << CamPose_orb[i].at<float>(2,3) << " " <<
    //                       0.0 << " " << 0.0 << " " << 0.0 << " " << 1.0 << endl;
    // }

    // save_traj_orb.close();

    // ************************************************************************************************
    // ************************************************************************************************

}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    // Save all the mappoints
    ofstream save_map;
    save_map.open("map_ScottReef25.txt",ios::trunc);
    vector<MapPoint*> mapPoints = mpMap->GetAllMapPoints();
    for(int i=0; i<mapPoints.size(); i++){
        cv::Mat position = mapPoints[i]->GetWorldPos();
        save_map << fixed << i << " " << setprecision(6) << position.at<float>(0) << " " << position.at<float>(1) << " "<< position.at<float>(2) << "\n";
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    // Save all the mappoints
    ofstream save_map;
    save_map.open("map_multibody_slam.txt",ios::trunc);
    vector<MapPoint*> mapPoints = mpMap->GetAllMapPoints();
    for(int i=0; i<mapPoints.size(); i++){
        cv::Mat position = mapPoints[i]->GetWorldPos();
        save_map << fixed << i << " " << setprecision(6) << position.at<float>(0) << " " << position.at<float>(1) << " "<< position.at<float>(2) << "\n";
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        //cout << "Current Pose: " << endl << twc << endl;

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

void System::StartViewer()
{
    if (mpViewer)
        mpViewer->Run();
}

} //namespace ORB_SLAM
