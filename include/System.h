/**
* This file is part of VDO-SLAM.
*
* Copyright (C) 2019-2020 Jun Zhang <jun doc zhang2 at anu dot edu doc au> (The Australian National University)
* For more information see <https://github.com/halajun/VDO_SLAM>
*
**/


#ifndef SYSTEM_H
#define SYSTEM_H

#include<string>
#include<thread>
#include<opencv2/core/core.hpp>

#include "Tracking.h"
#include "Map.h"

namespace VDO_SLAM
{

using namespace std;

class Map;
class Tracking;

class System
{
public:

    // Input sensor
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2
    };

public:

    // Initialize the SLAM system.
    System(const string &strSettingsFile, const eSensor sensor);


    // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
    // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Input depthmap: Float (CV_32F).
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackRGBD(const cv::Mat &im, cv::Mat &depthmap, const cv::Mat &flowmap, const cv::Mat &masksem,
                      const cv::Mat &mTcw_gt, const vector<vector<float> > &vObjPose_gt, const double &timestamp,
                      cv::Mat &imTraj, const int &nImage);

    void SaveResults(const string &filename);

private:

    // Input sensor
    eSensor mSensor;

    // Map structure.
    Map* mpMap;

    // Tracker. It receives a frame and computes the associated camera pose.
    Tracking* mpTracker;

};

}// namespace VDO_SLAM

#endif // SYSTEM_H
