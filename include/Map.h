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

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include <set>

#include <mutex>



namespace ORB_SLAM2
{

class MapPoint;
class KeyFrame;

class Map
{
public:
    Map();

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetMaxKFid();

    void clear();

    vector<KeyFrame*> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

    // ==========================================================
    // ============= output for evaluating results ==============

    // camera motion of each frame, starting from 1st frame. (k*1) // _1 for m and deg, _2 for % and deg/m
    std::vector<cv::Point2f> vvCamMotErr_1;
    std::vector<cv::Point2f> vvCamMotErr_2;

    // object motions in each frame
    std::vector<std::vector<int> > vvObjMotID;
    std::vector<std::vector<cv::Point2f> > vvObjMotErr_1;
    std::vector<std::vector<cv::Point2f> > vvObjMotErr_2;
    std::vector<std::vector<cv::Point2f> > vvObjMotErr_3;

    // dynamic object detection number
    std::vector<int> vTotObjNum;

    // save camera trajectory
    std::vector<cv::Mat> vmCameraPose_main;
    std::vector<cv::Mat> vmCameraPose_orb;

    // ==========================================================
    // ==========================================================

    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // <<<<<<<<<<<<<<<<<<<< output for graph structure >>>>>>>>>>>>>>>>>>>

    // static features and depths detected in image plane. (k*n)
    std::vector<std::vector<cv::KeyPoint> > vpFeatSta;
    std::vector<std::vector<float> > vfDepSta;
    std::vector<std::vector<cv::Mat> > vp3DPointSta;
    // index of temporal matching. (k-1)*n
    std::vector<std::vector<int> > vnAssoSta;
    // feature tracklets: pair.first = frameID; pair.second = featureID;
    std::vector<std::vector<std::pair<int, int> > > TrackletSta;


    // dynamic feature correspondences and depths detected in image plane. k*n
    std::vector<std::vector<cv::KeyPoint> > vpFeatDyn;
    std::vector<std::vector<float> > vfDepDyn;
    std::vector<std::vector<cv::Mat> > vp3DPointDyn;
    // index of temporal matching. (k-1)*n
    std::vector<std::vector<int> > vnAssoDyn;
    // label indicating which object the feature (3D point) belongs to. (k-1)*n
    std::vector<std::vector<int> > vnFeatLabel;
    // feature tracklets: pair.first = frameID; pair.second = featureID;
    std::vector<std::vector<std::pair<int, int> > > TrackletDyn;
    std::vector<int> nObjID;


    // absolute camera pose of each frame, starting from 1st frame. (k*1)
    std::vector<cv::Mat> vmCameraPose;
    std::vector<cv::Mat> vmCameraPose_RF;  // refine result
    std::vector<cv::Mat> vmCameraPose_GT;  // ground truth result
    // rigid motion of camera and dynamic points. (k-1)*m
    std::vector<std::vector<cv::Mat> > vmRigidCentre;  // ground truth object center
    std::vector<std::vector<cv::Mat> > vmRigidMotion;
    std::vector<std::vector<cv::Mat> > vmRigidMotion_RF;  // refine result
    std::vector<std::vector<cv::Mat> > vmRigidMotion_GT;  // ground truth result
    std::vector<std::vector<float> > vfAllSpeed_GT; // camera and object speeds
    // rigid motion label in each frame (k-1)*m
    // 0 stands for camera motion; 1,...,l stands for rigid motions.
    std::vector<std::vector<int> > vnRMLabel;
    // object status (10 Jan 2020)
    std::vector<std::vector<bool> > vbObjStat;
    // object tracking times (10 Jan 2020)
    std::vector<std::vector<int> > vnObjTraTime;


    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

protected:
    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    long unsigned int mnMaxKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    std::mutex mMutexMap;
};

} //namespace ORB_SLAM

#endif // MAP_H
