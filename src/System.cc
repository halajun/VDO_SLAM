/**
* This file is part of VDO-SLAM.
*
* Copyright (C) 2019-2020 Jun Zhang <jun doc zhang2 at anu dot edu doc au> (The Australian National University)
* For more information see <https://github.com/halajun/DynamicObjectSLAM>
*
**/



#include "System.h"
#include "Converter.h"
#include <thread>
#include <iomanip>


#include <unistd.h>

namespace VDO_SLAM
{

System::System(const string &strSettingsFile, const eSensor sensor):mSensor(sensor)
{
    // // ===== output welcome message ======
    // cout << endl <<
    // "VDO-SLAM Copyright (C) 2019-2020 Jun Zhang, Australian National University." << endl <<
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

    // Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }

    //Create the Map
    mpMap = new Map();

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpMap, strSettingsFile, mSensor);

}


cv::Mat System::TrackRGBD(const cv::Mat &im, cv::Mat &depthmap, const cv::Mat &flowmap, const cv::Mat &masksem,
                          const cv::Mat &mTcw_gt, const vector<vector<float> > &vObjPose_gt,
                          const double &timestamp, cv::Mat &imTraj, const int &nImage)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }

    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,flowmap,masksem,mTcw_gt,vObjPose_gt,timestamp,imTraj,nImage);

    return Tcw;
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

void System::SaveResultsIJRR2020(const string &filename)
{
    cout << endl << "Saving Results into TXT File..." << endl;

    // *******************************************************************************************************
    // ***************************************** SAVE OBJ SPEED **********************************************
    // *******************************************************************************************************

    ofstream save_objmot, save_objmot_gt;
    string path_objmot = filename + "obj_mot_rgbd_new.txt";
    string path_objmot_gt = filename + "obj_mot_gt.txt";
    save_objmot.open(path_objmot.c_str(),ios::trunc);
    save_objmot_gt.open(path_objmot_gt.c_str(),ios::trunc);

    int start_frame = 0;
    // main loop
    for (int i = 0; i < mpMap->vmRigidMotion.size(); ++i)
    {
        if (mpMap->vmRigidMotion[i].size()>1)
        {
            for (int j = 1; j < mpMap->vmRigidMotion[i].size(); ++j)
            {
                save_objmot << start_frame+i+1 << " " <<  mpMap->vnRMLabel[i][j] << " " << fixed << setprecision(9) <<
                                 mpMap->vmRigidMotion[i][j].at<float>(0,0) << " " << mpMap->vmRigidMotion[i][j].at<float>(0,1)  << " " << mpMap->vmRigidMotion[i][j].at<float>(0,2) << " "  << mpMap->vmRigidMotion[i][j].at<float>(0,3) << " " <<
                                 mpMap->vmRigidMotion[i][j].at<float>(1,0) << " " << mpMap->vmRigidMotion[i][j].at<float>(1,1)  << " " << mpMap->vmRigidMotion[i][j].at<float>(1,2) << " "  << mpMap->vmRigidMotion[i][j].at<float>(1,3) << " " <<
                                 mpMap->vmRigidMotion[i][j].at<float>(2,0) << " " << mpMap->vmRigidMotion[i][j].at<float>(2,1)  << " " << mpMap->vmRigidMotion[i][j].at<float>(2,2) << " "  << mpMap->vmRigidMotion[i][j].at<float>(2,3) << " " <<
                                 0.0 << " " << 0.0 << " " << 0.0 << " " << 1.0 << endl;

                save_objmot_gt << start_frame+i+1 << " " <<  mpMap->vnRMLabel[i][j] << " " << fixed << setprecision(9) <<
                                  mpMap->vmRigidMotion_GT[i][j].at<float>(0,0) << " " << mpMap->vmRigidMotion_GT[i][j].at<float>(0,1)  << " " << mpMap->vmRigidMotion_GT[i][j].at<float>(0,2) << " "  << mpMap->vmRigidMotion_GT[i][j].at<float>(0,3) << " " <<
                                  mpMap->vmRigidMotion_GT[i][j].at<float>(1,0) << " " << mpMap->vmRigidMotion_GT[i][j].at<float>(1,1)  << " " << mpMap->vmRigidMotion_GT[i][j].at<float>(1,2) << " "  << mpMap->vmRigidMotion_GT[i][j].at<float>(1,3) << " " <<
                                  mpMap->vmRigidMotion_GT[i][j].at<float>(2,0) << " " << mpMap->vmRigidMotion_GT[i][j].at<float>(2,1)  << " " << mpMap->vmRigidMotion_GT[i][j].at<float>(2,2) << " "  << mpMap->vmRigidMotion_GT[i][j].at<float>(2,3) << " " <<
                                  0.0 << " " << 0.0 << " " << 0.0 << " " << 1.0 << endl;
            }
        }
    }

    save_objmot.close();
    save_objmot_gt.close();

    // *******************************************************************************************************
    // ***************************************** SAVE CAMERA TRAJETORY ***************************************
    // *******************************************************************************************************

    std::vector<cv::Mat> CamPose_ini = mpMap->vmCameraPose;

    ofstream save_traj_ini;
    string path_ini = filename + "initial_rgbd_new.txt";
    // cout << path_ini << endl;
    save_traj_ini.open(path_ini.c_str(),ios::trunc);

    for (int i = 0; i < CamPose_ini.size(); ++i)
    {
        save_traj_ini << start_frame+i << " " << fixed << setprecision(9) << CamPose_ini[i].at<float>(0,0) << " " << CamPose_ini[i].at<float>(0,1)  << " " << CamPose_ini[i].at<float>(0,2) << " "  << CamPose_ini[i].at<float>(0,3) << " " <<
                          CamPose_ini[i].at<float>(1,0) << " " << CamPose_ini[i].at<float>(1,1)  << " " << CamPose_ini[i].at<float>(1,2) << " "  << CamPose_ini[i].at<float>(1,3) << " " <<
                          CamPose_ini[i].at<float>(2,0) << " " << CamPose_ini[i].at<float>(2,1)  << " " << CamPose_ini[i].at<float>(2,2) << " "  << CamPose_ini[i].at<float>(2,3) << " " <<
                          0.0 << " " << 0.0 << " " << 0.0 << " " << 1.0 << endl;
    }

    save_traj_ini.close();

    // ------------------------------------------------------------------------------------------------------------

    std::vector<cv::Mat> CamPose_ref = mpMap->vmCameraPose_RF;

    ofstream save_traj_ref;
    string path_ref = filename + "refined_rgbd_new.txt";
    save_traj_ref.open(path_ref.c_str(),ios::trunc);

    for (int i = 0; i < CamPose_ref.size(); ++i)
    {
        save_traj_ref << start_frame+i << " " << fixed << setprecision(9) << CamPose_ref[i].at<float>(0,0) << " " << CamPose_ref[i].at<float>(0,1)  << " " << CamPose_ref[i].at<float>(0,2) << " "  << CamPose_ref[i].at<float>(0,3) << " " <<
                          CamPose_ref[i].at<float>(1,0) << " " << CamPose_ref[i].at<float>(1,1)  << " " << CamPose_ref[i].at<float>(1,2) << " "  << CamPose_ref[i].at<float>(1,3) << " " <<
                          CamPose_ref[i].at<float>(2,0) << " " << CamPose_ref[i].at<float>(2,1)  << " " << CamPose_ref[i].at<float>(2,2) << " "  << CamPose_ref[i].at<float>(2,3) << " " <<
                          0.0 << " " << 0.0 << " " << 0.0 << " " << 1.0 << endl;
    }

    save_traj_ref.close();

    // ------------------------------------------------------------------------------------------------------------

    std::vector<cv::Mat> CamPose_gt = mpMap->vmCameraPose_GT;

    ofstream save_traj_gt;
    string path_gt = filename + "cam_pose_gt.txt";
    save_traj_gt.open(path_gt.c_str(),ios::trunc);

    for (int i = 0; i < CamPose_gt.size(); ++i)
    {
        save_traj_gt << start_frame+i << " " << fixed << setprecision(9) << CamPose_gt[i].at<float>(0,0) << " " << CamPose_gt[i].at<float>(0,1)  << " " << CamPose_gt[i].at<float>(0,2) << " "  << CamPose_gt[i].at<float>(0,3) << " " <<
                          CamPose_gt[i].at<float>(1,0) << " " << CamPose_gt[i].at<float>(1,1)  << " " << CamPose_gt[i].at<float>(1,2) << " "  << CamPose_gt[i].at<float>(1,3) << " " <<
                          CamPose_gt[i].at<float>(2,0) << " " << CamPose_gt[i].at<float>(2,1)  << " " << CamPose_gt[i].at<float>(2,2) << " "  << CamPose_gt[i].at<float>(2,3) << " " <<
                          0.0 << " " << 0.0 << " " << 0.0 << " " << 1.0 << endl;
    }

    save_traj_gt.close();

    // ------------------------------------------------------------------------------------------------------------

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

    // *******************************************************************************************************
    // ***************************************** SAVE TIME ANALYSIS ******************************************
    // *******************************************************************************************************

    std::vector<std::vector<float> > All_timing = mpMap->vfAll_time;
    std::vector<float> LBA_timing = mpMap->fLBA_time;

    int obj_time_count=0;

    // all tracking components
    std::vector<float> avg_timing(All_timing[0].size(),0);
    for (int i = 0; i < All_timing.size(); ++i)
        for (int j = 0; j < All_timing[i].size(); ++j)
        {
            if (j==3 && All_timing[i][j]!=0)
            {
                avg_timing[j] = avg_timing[j] + All_timing[i][j];
                obj_time_count = obj_time_count + 1;
            }
            else
                avg_timing[j] = avg_timing[j] + All_timing[i][j];
        }

    cout << "Time of all components: " << endl;
    for (int j = 0; j < avg_timing.size(); ++j)
    {
        if (j==3)
            cout << "(" << j << "): " <<  avg_timing[j]/obj_time_count << " ";
        else
            cout << "(" << j << "): " <<  avg_timing[j]/All_timing.size() << " ";
    }
    cout << endl;

    // local bundle adjustment
    float avg_lba_timing = 0;
    for (int i = 0; i < LBA_timing.size(); ++i)
        avg_lba_timing = avg_lba_timing + LBA_timing[i];
    cout << "Time of local bundle adjustment: " << avg_lba_timing/LBA_timing.size() << endl;



    // ************************************************************************************************************
    // ************************************************************************************************************

}



} //namespace VDO_SLAM
