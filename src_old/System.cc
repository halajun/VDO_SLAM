/**
 * This file is part of VDO-SLAM.
 *
 * Copyright (C) 2019-2020 Jun Zhang <jun doc zhang2 at anu dot edu doc au> (The Australian National University)
 * For more information see <https://github.com/halajun/VDO_SLAM>
 *
 **/

#include "System.h"
#include "Converter.h"
#include <thread>
#include <iomanip>

#include "utils/timing.h"
#include <glog/logging.h>

#include <unistd.h>

namespace VDO_SLAM
{
System::System(const string& strSettingsFile, const eSensor sensor) : mSensor(sensor)
{
  // ===== output welcome message ======
  cout << endl
       << " ----------------------------------------------------------------------------" << endl
       << "| VDO-SLAM Copyright (C) 2019-2020 Jun Zhang, Australian National University.|" << endl
       << "| This program comes with ABSOLUTELY NO WARRANTY;                            |" << endl
       << "| This is free software, and you are welcome to redistribute it              |" << endl
       << "| under certain conditions; See LICENSE.txt.                                 |" << endl
       << " ----------------------------------------------------------------------------" << endl;

  // Check settings file
  cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
  if (!fsSettings.isOpened())
  {
    cerr << "Failed to open settings file at: " << strSettingsFile << endl;
    exit(-1);
  }

  // Create the Map
  mpMap = new Map();

  // Initialize the Tracking thread
  //(it will live in the main thread of execution, the one that called this constructor)
  mpTracker = new Tracking(this, mpMap, strSettingsFile, mSensor);
}

cv::Mat System::TrackRGBD(const cv::Mat& im, cv::Mat& depthmap, const cv::Mat& flowmap, const cv::Mat& masksem,
                          const cv::Mat& mTcw_gt, const vector<vector<float> >& vObjPose_gt, const double& timestamp,
                          cv::Mat& imTraj, const int& nImage)
{
  if (mSensor != RGBD)
  {
    cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
    exit(-1);
  }

  cv::Mat Tcw =
      mpTracker->GrabImageRGBD(im, depthmap, flowmap, masksem, mTcw_gt, vObjPose_gt, timestamp, imTraj, nImage);
  LOG_EVERY_N(INFO, 1) << timing::Timing::Print();
  return Tcw;
}

void System::SaveResults(const string& filename)
{
  cout << endl << "Saving Results into TXT File..." << endl;

  // *******************************************************************************************************
  // ***************************************** SAVE OBJ SPEED **********************************************
  // *******************************************************************************************************

  ofstream save_objmot, save_objmot_rf, save_objmot_gt, save_obj_centre;
  string path_objmot = filename + "obj_mot_stereo_new.txt";
  string path_objmot_rf = filename + "obj_mot_stereo_rf_new.txt";
  string path_objmot_gt = filename + "obj_mot_gt.txt";
  string path_obj_centre = filename + "obj_centre.txt";
  save_objmot.open(path_objmot.c_str(), ios::trunc);
  save_objmot_rf.open(path_objmot_rf.c_str(), ios::trunc);
  save_objmot_gt.open(path_objmot_gt.c_str(), ios::trunc);
  save_obj_centre.open(path_obj_centre.c_str(), ios::trunc);

  int start_frame = 0;
  // main loop
  for (int i = 0; i < mpMap->vmRigidMotion.size(); ++i)
  {
    if (mpMap->vmRigidMotion[i].size() > 1)
    {
      for (int j = 1; j < mpMap->vmRigidMotion[i].size(); ++j)
      {
        cv::Mat RigMot_body =
            Converter::toInvMatrix(mpMap->vmObjPosePre[i][j]) * mpMap->vmRigidMotion[i][j] * mpMap->vmObjPosePre[i][j];
        cv::Mat RigMot_body_rf = Converter::toInvMatrix(mpMap->vmObjPosePre[i][j]) * mpMap->vmRigidMotion_RF[i][j] *
                                 mpMap->vmObjPosePre[i][j];

        save_objmot << start_frame + i + 1 << " " << mpMap->vnRMLabel[i][j] << " " << fixed << setprecision(9)
                    << RigMot_body.at<float>(0, 0) << " " << RigMot_body.at<float>(0, 1) << " "
                    << RigMot_body.at<float>(0, 2) << " " << RigMot_body.at<float>(0, 3) << " "
                    << RigMot_body.at<float>(1, 0) << " " << RigMot_body.at<float>(1, 1) << " "
                    << RigMot_body.at<float>(1, 2) << " " << RigMot_body.at<float>(1, 3) << " "
                    << RigMot_body.at<float>(2, 0) << " " << RigMot_body.at<float>(2, 1) << " "
                    << RigMot_body.at<float>(2, 2) << " " << RigMot_body.at<float>(2, 3) << " " << 0.0 << " " << 0.0
                    << " " << 0.0 << " " << 1.0 << endl;

        save_objmot_rf << start_frame + i + 1 << " " << mpMap->vnRMLabel[i][j] << " " << fixed << setprecision(9)
                       << RigMot_body_rf.at<float>(0, 0) << " " << RigMot_body_rf.at<float>(0, 1) << " "
                       << RigMot_body_rf.at<float>(0, 2) << " " << RigMot_body_rf.at<float>(0, 3) << " "
                       << RigMot_body_rf.at<float>(1, 0) << " " << RigMot_body_rf.at<float>(1, 1) << " "
                       << RigMot_body_rf.at<float>(1, 2) << " " << RigMot_body_rf.at<float>(1, 3) << " "
                       << RigMot_body_rf.at<float>(2, 0) << " " << RigMot_body_rf.at<float>(2, 1) << " "
                       << RigMot_body_rf.at<float>(2, 2) << " " << RigMot_body_rf.at<float>(2, 3) << " " << 0.0 << " "
                       << 0.0 << " " << 0.0 << " " << 1.0 << endl;

        save_objmot_gt << start_frame + i + 1 << " " << mpMap->vnRMLabel[i][j] << " " << fixed << setprecision(9)
                       << mpMap->vmRigidMotion_GT[i][j].at<float>(0, 0) << " "
                       << mpMap->vmRigidMotion_GT[i][j].at<float>(0, 1) << " "
                       << mpMap->vmRigidMotion_GT[i][j].at<float>(0, 2) << " "
                       << mpMap->vmRigidMotion_GT[i][j].at<float>(0, 3) << " "
                       << mpMap->vmRigidMotion_GT[i][j].at<float>(1, 0) << " "
                       << mpMap->vmRigidMotion_GT[i][j].at<float>(1, 1) << " "
                       << mpMap->vmRigidMotion_GT[i][j].at<float>(1, 2) << " "
                       << mpMap->vmRigidMotion_GT[i][j].at<float>(1, 3) << " "
                       << mpMap->vmRigidMotion_GT[i][j].at<float>(2, 0) << " "
                       << mpMap->vmRigidMotion_GT[i][j].at<float>(2, 1) << " "
                       << mpMap->vmRigidMotion_GT[i][j].at<float>(2, 2) << " "
                       << mpMap->vmRigidMotion_GT[i][j].at<float>(2, 3) << " " << 0.0 << " " << 0.0 << " " << 0.0 << " "
                       << 1.0 << endl;

        save_obj_centre << start_frame + i + 1 << " " << mpMap->vnRMLabel[i][j] << " " << fixed << setprecision(9)
                        << mpMap->vmRigidCentre[i][j].at<float>(0, 0) << " "
                        << mpMap->vmRigidCentre[i][j].at<float>(0, 1) << " "
                        << mpMap->vmRigidCentre[i][j].at<float>(0, 2) << endl;
      }
    }
  }

  save_objmot.close();
  save_objmot_rf.close();
  save_objmot_gt.close();
  save_obj_centre.close();

  // *******************************************************************************************************
  // ***************************************** SAVE CAMERA TRAJETORY ***************************************
  // *******************************************************************************************************

  std::vector<cv::Mat> CamPose_ini = mpMap->vmCameraPose;

  ofstream save_traj_ini;
  string path_ini = filename + "initial_stereo_new.txt";
  // cout << path_ini << endl;
  save_traj_ini.open(path_ini.c_str(), ios::trunc);

  for (int i = 0; i < CamPose_ini.size(); ++i)
  {
    save_traj_ini << start_frame + i << " " << fixed << setprecision(9) << CamPose_ini[i].at<float>(0, 0) << " "
                  << CamPose_ini[i].at<float>(0, 1) << " " << CamPose_ini[i].at<float>(0, 2) << " "
                  << CamPose_ini[i].at<float>(0, 3) << " " << CamPose_ini[i].at<float>(1, 0) << " "
                  << CamPose_ini[i].at<float>(1, 1) << " " << CamPose_ini[i].at<float>(1, 2) << " "
                  << CamPose_ini[i].at<float>(1, 3) << " " << CamPose_ini[i].at<float>(2, 0) << " "
                  << CamPose_ini[i].at<float>(2, 1) << " " << CamPose_ini[i].at<float>(2, 2) << " "
                  << CamPose_ini[i].at<float>(2, 3) << " " << 0.0 << " " << 0.0 << " " << 0.0 << " " << 1.0 << endl;
  }

  save_traj_ini.close();

  // ------------------------------------------------------------------------------------------------------------

  std::vector<cv::Mat> CamPose_ref = mpMap->vmCameraPose_RF;

  ofstream save_traj_ref;
  string path_ref = filename + "refined_stereo_new.txt";
  save_traj_ref.open(path_ref.c_str(), ios::trunc);

  for (int i = 0; i < CamPose_ref.size(); ++i)
  {
    save_traj_ref << start_frame + i << " " << fixed << setprecision(9) << CamPose_ref[i].at<float>(0, 0) << " "
                  << CamPose_ref[i].at<float>(0, 1) << " " << CamPose_ref[i].at<float>(0, 2) << " "
                  << CamPose_ref[i].at<float>(0, 3) << " " << CamPose_ref[i].at<float>(1, 0) << " "
                  << CamPose_ref[i].at<float>(1, 1) << " " << CamPose_ref[i].at<float>(1, 2) << " "
                  << CamPose_ref[i].at<float>(1, 3) << " " << CamPose_ref[i].at<float>(2, 0) << " "
                  << CamPose_ref[i].at<float>(2, 1) << " " << CamPose_ref[i].at<float>(2, 2) << " "
                  << CamPose_ref[i].at<float>(2, 3) << " " << 0.0 << " " << 0.0 << " " << 0.0 << " " << 1.0 << endl;
  }

  save_traj_ref.close();

  // ------------------------------------------------------------------------------------------------------------

  std::vector<cv::Mat> CamPose_gt = mpMap->vmCameraPose_GT;

  ofstream save_traj_gt;
  string path_gt = filename + "cam_pose_gt_stereo.txt";
  save_traj_gt.open(path_gt.c_str(), ios::trunc);

  for (int i = 0; i < CamPose_gt.size(); ++i)
  {
    save_traj_gt << start_frame + i << " " << fixed << setprecision(9) << CamPose_gt[i].at<float>(0, 0) << " "
                 << CamPose_gt[i].at<float>(0, 1) << " " << CamPose_gt[i].at<float>(0, 2) << " "
                 << CamPose_gt[i].at<float>(0, 3) << " " << CamPose_gt[i].at<float>(1, 0) << " "
                 << CamPose_gt[i].at<float>(1, 1) << " " << CamPose_gt[i].at<float>(1, 2) << " "
                 << CamPose_gt[i].at<float>(1, 3) << " " << CamPose_gt[i].at<float>(2, 0) << " "
                 << CamPose_gt[i].at<float>(2, 1) << " " << CamPose_gt[i].at<float>(2, 2) << " "
                 << CamPose_gt[i].at<float>(2, 3) << " " << 0.0 << " " << 0.0 << " " << 0.0 << " " << 1.0 << endl;
  }

  save_traj_gt.close();

  // ------------------------------------------------------------------------------------------------------------

  // std::vector<cv::Mat> CamPose_orb = mpMap->vmCameraPose_orb;

  // ofstream save_traj_orb;
  // string path_orb = filename + "orb.txt";
  // save_traj_orb.open(path_orb.c_str(),ios::trunc);

  // for (int i = 0; i < CamPose_orb.size(); ++i)
  // {
  //     save_traj_orb  << fixed << setprecision(9) << CamPose_orb[i].at<float>(0,0) << " " <<
  //     CamPose_orb[i].at<float>(0,1)  << " " << CamPose_orb[i].at<float>(0,2) << " "  << CamPose_orb[i].at<float>(0,3)
  //     << " " <<
  //                       CamPose_orb[i].at<float>(1,0) << " " << CamPose_orb[i].at<float>(1,1)  << " " <<
  //                       CamPose_orb[i].at<float>(1,2) << " "  << CamPose_orb[i].at<float>(1,3) << " " <<
  //                       CamPose_orb[i].at<float>(2,0) << " " << CamPose_orb[i].at<float>(2,1)  << " " <<
  //                       CamPose_orb[i].at<float>(2,2) << " "  << CamPose_orb[i].at<float>(2,3) << " " << 0.0 << " "
  //                       << 0.0 << " " << 0.0 << " " << 1.0 << endl;
  // }

  // save_traj_orb.close();

  // *******************************************************************************************************
  // ***************************************** SAVE TIME ANALYSIS ******************************************
  // *******************************************************************************************************

  std::vector<std::vector<float> > All_timing = mpMap->vfAll_time;
  std::vector<float> LBA_timing = mpMap->fLBA_time;

  int obj_time_count = 0;

  // all tracking components
  std::vector<float> avg_timing(All_timing[0].size(), 0);
  for (int i = 0; i < All_timing.size(); ++i)
    for (int j = 0; j < All_timing[i].size(); ++j)
    {
      if (j == 3 && All_timing[i][j] != 0)
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
    if (j == 3)
      cout << "(" << j << "): " << avg_timing[j] / obj_time_count << " ";
    else
      cout << "(" << j << "): " << avg_timing[j] / All_timing.size() << " ";
  }
  cout << endl;

  // local bundle adjustment
  float avg_lba_timing = 0;
  for (int i = 0; i < LBA_timing.size(); ++i)
    avg_lba_timing = avg_lba_timing + LBA_timing[i];
  cout << "Time of local bundle adjustment: " << avg_lba_timing / LBA_timing.size() << endl;

  // ************************************************************************************************************
  // ************************************************************************************************************
}

}  // namespace VDO_SLAM
