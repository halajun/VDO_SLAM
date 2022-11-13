#include "DataProvider.h"
#include "UtilsGTSAM.h"
#include <glog/logging.h>
#include <iostream>


namespace vdo {

size_t DataProvider::next(Inputs& input) {
    return next(input.first, input.second);
}


KittiSequenceDataProvider::KittiSequenceDataProvider(const std::string& path_to_sequence_) {
    LOG(INFO) << "Constructing Kitti Sequence Data provider - " << path_to_sequence_;
    loadData(path_to_sequence_, data);
}

bool KittiSequenceDataProvider::loadData(const std::string& path_to_sequence, InputsVector& inputs_vector) {
    // +++ timestamps +++
    std::vector<std::string> vstrFilenamesRGB;
    std::vector<std::string> vstrFilenamesDEP;
    std::vector<std::string> vstrFilenamesSEM;
    std::vector<std::string> vstrFilenamesFLO;
    std::vector<gtsam::Pose3> vPoseGT;
    VectorsXx<gtsam::Pose3> vObjPoseGT;
    std::vector<Timestamp> vTimestamps;

    std::ifstream times_stream;
    std::string strPathTimeFile = path_to_sequence + "/times.txt";
    times_stream.open(strPathTimeFile.c_str());
    while(!times_stream.eof())
    {
        std::string s;
        getline(times_stream,s);
        if(!s.empty())
        {
            std::stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }
    times_stream.close();

    // +++ image, depth, semantic and moving object tracking mask +++
    std::string strPrefixImage = path_to_sequence + "/image_0/";         // image  image_0
    std::string strPrefixDepth = path_to_sequence + "/depth/";           // depth_gt  depth  depth_mono_stereo
    std::string strPrefixSemantic = path_to_sequence + "/semantic/";     // semantic_gt  semantic
    std::string strPrefixFlow = path_to_sequence + "/flow/";             // flow_gt  flow

    const int nTimes = vTimestamps.size();
    vstrFilenamesRGB.resize(nTimes);
    vstrFilenamesDEP.resize(nTimes);
    vstrFilenamesSEM.resize(nTimes);
    vstrFilenamesFLO.resize(nTimes);


    for(int i=0; i<nTimes; i++)
    {
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(6) << i;
        vstrFilenamesRGB[i] = strPrefixImage + ss.str() + ".png";
        vstrFilenamesDEP[i] = strPrefixDepth + ss.str() + ".png";
        vstrFilenamesSEM[i] = strPrefixSemantic + ss.str() + ".txt";
        vstrFilenamesFLO[i] = strPrefixFlow + ss.str() + ".flo";
    }


    // +++ ground truth pose +++
    std::string strFilenamePose = path_to_sequence + "/pose_gt.txt"; //  pose_gt.txt  kevin_extrinsics.txt
    // vPoseGT.resize(nTimes);
    std::ifstream fPose;
    fPose.open(strFilenamePose.c_str());
    while(!fPose.eof())
    {
        std::string s;
        getline(fPose,s);
        if(!s.empty())
        {
            std::stringstream ss;
            ss << s;
            int t;
            ss >> t;
            cv::Mat Pose_tmp = cv::Mat::eye(4,4,CV_64F);

            ss >> Pose_tmp.at<double>(0,0) >> Pose_tmp.at<double>(0,1) >> Pose_tmp.at<double>(0,2) >> Pose_tmp.at<double>(0,3)
               >> Pose_tmp.at<double>(1,0) >> Pose_tmp.at<double>(1,1) >> Pose_tmp.at<double>(1,2) >> Pose_tmp.at<double>(1,3)
               >> Pose_tmp.at<double>(2,0) >> Pose_tmp.at<double>(2,1) >> Pose_tmp.at<double>(2,2) >> Pose_tmp.at<double>(2,3)
               >> Pose_tmp.at<double>(3,0) >> Pose_tmp.at<double>(3,1) >> Pose_tmp.at<double>(3,2) >> Pose_tmp.at<double>(3,3);

            gtsam::Pose3 pose = utils::cvMatToGtsamPose3(Pose_tmp);
            vPoseGT.push_back(Pose_tmp);
            // if(t==410)
            //     cout << "ground truth pose 0 (for validation):" << endl << vPoseGT[t] << endl;
        }
    }
    fPose.close();


    // +++ ground truth object pose +++
    string strFilenameObjPose = strPathToSequence + "/object_pose.txt";
    ifstream fObjPose;
    fObjPose.open(strFilenameObjPose.c_str());

    while(!fObjPose.eof())
    {
        string s;
        getline(fObjPose,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;

            std::vector<float> ObjPose_tmp(10,0);
            ss >> ObjPose_tmp[0] >> ObjPose_tmp[1] >> ObjPose_tmp[2] >> ObjPose_tmp[3]
               >> ObjPose_tmp[4] >> ObjPose_tmp[5] >> ObjPose_tmp[6] >> ObjPose_tmp[7]
               >> ObjPose_tmp[8] >> ObjPose_tmp[9];

            vObjPoseGT.push_back(ObjPose_tmp);

        }
    }
    fObjPose.close();

}



}