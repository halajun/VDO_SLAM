// This file is used to align the semantic segmentation labels with the ground truth object labels, 
// so that one can compare the object motion estimation results with the ground truth object motion.
// The applied method is to assign the estimated object mask with the same label as the ground truth mask, 
// where both masks have overlapped most pixels.

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <random>
#include <algorithm>
#include <numeric>
#include <vector>

using namespace std;
using namespace cv;

// How To Run: ./kitti_mask_sem2gt PATH-TO-FOLDER

// // Format of .mask file: 
// (1) first row : Image_height Image_width Number_of_detected_objects;
// (2) following: category of the detected objects, with each taking one row;
// (3) finally: the mask matrix of image size, with -1 for background, and 1,...,n for objects;

cv::Mat LoadMask(const string &strMask);

void LoadData(const string &strPathToSequence, vector<double> &vTimestamps, vector<string> &vstrFilenamesSEM,
              vector<vector<float> > &vObjPoseGT, string &output_path);

int main(int argc, char **argv)
{
    // Retrieve paths to images
    string output_path;
    vector<string> vstrFilenamesSEM;
    vector<double> vTimestamps;
    vector<vector<float> > vObjPoseGT;

    LoadData(argv[1], vTimestamps, vstrFilenamesSEM, vObjPoseGT, output_path);

    // save the id of object pose in each frame
    vector<vector<int> > vObjPoseID(vstrFilenamesSEM.size());
    for (int i = 0; i < vObjPoseGT.size(); ++i)
    {
        int f_id = vObjPoseGT[i][0];
        vObjPoseID[f_id].push_back(i);
    }

    for(int ni=0; ni<vstrFilenamesSEM.size(); ni++)
    {
        cout << "Processing Frame: " << ni << endl;
        cv::Mat imSem(375, 1242, CV_32SC1);
        imSem = LoadMask(vstrFilenamesSEM[ni]);

        // object poses in current frame
        vector<vector<float> > vObjPose_gt(vObjPoseID[ni].size());
        for (int i = 0; i < vObjPoseID[ni].size(); ++i)
            vObjPose_gt[i] = vObjPoseGT[vObjPoseID[ni][i]];

        // collect uni_label in each mask
        std::vector<int> vSemLabel;
        std::vector<cv::Point2i> vKeyPoint;
        for (int i = 0; i < imSem.rows; i++)
        {
            for (int j = 0; j < imSem.cols; j++)
            {
              if (imSem.at<int>(i,j)!=0)
              {
                vKeyPoint.push_back(cv::Point2f(i,j));
                vSemLabel.push_back(imSem.at<int>(i,j));
              }
            }
        }

        // find the unique labels in semantic label
        std::vector<int> UniLab = vSemLabel;
        std::sort(UniLab.begin(), UniLab.end());
        UniLab.erase(std::unique( UniLab.begin(), UniLab.end() ), UniLab.end() );

        // collect key-points according to UniLab
        std::vector<std::vector<cv::Point2i> > vKeyPointNew(UniLab.size());
        for (int i = 0; i < vKeyPoint.size(); ++i)
        {
            // save object label
            for (int j = 0; j < UniLab.size(); ++j)
            {
                if(vSemLabel[i]==UniLab[j]){
                    vKeyPointNew[j].push_back(vKeyPoint[i]);
                    break;
                }
            }
        }

        std::vector<int> SemLabNew(UniLab.size(),0);
        for (int i = 0; i < vKeyPointNew.size(); ++i)
        {
            float max_po_num = 0;
            int obj_match = -1;
            for (int j = 0; j < vObjPose_gt.size(); ++j)
            {
              int count = 0;
              for (int k = 0; k < vKeyPointNew[i].size(); ++k)
              {
                  const float u = vKeyPointNew[i][k].x;
                  const float v = vKeyPointNew[i][k].y;
                  if (u>vObjPose_gt[j][3] && u<vObjPose_gt[j][5] && v>vObjPose_gt[j][2] && v<vObjPose_gt[j][4])
                  {
                    count = count + 1;
                  }
              }
              if (count>max_po_num)
              {
                obj_match = j;
                max_po_num = count;
              }
            }
            // get label !!!
            if (obj_match!=-1){
              cout << "get new label! " << vObjPose_gt[obj_match][1] << endl;
              SemLabNew[i] = vObjPose_gt[obj_match][1];
            }
        }

        // relabel the mask matrix
        for (int i = 0; i < vKeyPointNew.size(); ++i)
        {
            for (int j = 0; j < vKeyPointNew[i].size(); ++j)
            {
                imSem.at<int>(vKeyPointNew[i][j].x,vKeyPointNew[i][j].y) = SemLabNew[i];
            }
        }

        // create txt file
        ofstream save_mask;
        stringstream ss;
        ss << setfill('0') << setw(6) << ni;
        string save_path = output_path + ss.str() + ".txt";
        save_mask.open(save_path.c_str(),ios::trunc);
        for (int i = 0; i < imSem.rows; i++)
        {
            for (int j = 0; j < imSem.cols; j++)
            {
               save_mask << imSem.at<int>(i,j) << " ";
            }
            save_mask << endl;
        }
        save_mask.close();
    }

  return 0;
}

void LoadData(const string &strPathToSequence, vector<double> &vTimestamps, vector<string> &vstrFilenamesSEM,
              vector<vector<float> > &vObjPoseGT, string &output_path)
{

    output_path = strPathToSequence + "/semantic/";
    // +++ timestamps +++
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }
    fTimes.close();

    // +++ get mask path +++
    string strPrefixSemantic = strPathToSequence + "/mask/";

    const int nTimes = vTimestamps.size();
    vstrFilenamesSEM.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrFilenamesSEM[i] = strPrefixSemantic + ss.str() + ".png.mask";
    }

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

cv::Mat LoadMask(const string &strMask)
{
    ifstream file_mask;
    file_mask.open(strMask.c_str());

    int count = 0;
    std::vector<int> info_mask(3);

    // the first line
    string s;
    getline(file_mask,s);
    if(!s.empty()){
        stringstream ss;
        ss << s;
        ss >> info_mask[0] >> info_mask[1] >> info_mask[2];
        count++;
    }
    // cout << "img_mask: " << info_mask[0] << " " << info_mask[1] << " " << info_mask[2] << " " << endl;

    // the rest lines
    std::vector<bool> bRelabel(info_mask[2],false);
    cv::Mat img_mask(info_mask[0], info_mask[1], CV_32SC1);
    cv::Mat imgLabel(info_mask[0],info_mask[1],CV_8UC3); // for display
    while(!file_mask.eof())
    {
        string s;
        getline(file_mask,s);
        if(!s.empty())
        {
            stringstream ss;
            // skip until the data lines
            if(count<info_mask[2]+1){
                // cout << s << endl;
                if(s.compare("traffic light")==0 || s.compare("handbag")==0 || s.compare("bicycle")==0){
                    // cout << "traffic light ! ! ! " << count-1 << endl;
                    bRelabel[count-1] = true;
                }
                count++;
            }
            else{
                ss << s;
                int tmp;
                for(int i = 0; i < info_mask[1]; ++i){
                    ss >> tmp;
                    if (tmp!=-1){
                        if (bRelabel[tmp]){
                            img_mask.at<int>(count-info_mask[2]-1,i) = 0;  // -1 <-> 0
                            imgLabel.at<cv::Vec3b>(count-info_mask[2]-1,i) = cv::Vec3b(0,255,0);  // green
                        }
                        else{
                            img_mask.at<int>(count-info_mask[2]-1,i) = tmp + 1;  // tmp = tmp + 1
                            switch (tmp)
                            {
                                case 0:
                                    imgLabel.at<cv::Vec3b>(count-info_mask[2]-1,i) = cv::Vec3b(0,255,255);
                                    break;
                                case 1:
                                    imgLabel.at<cv::Vec3b>(count-info_mask[2]-1,i) = cv::Vec3b(0,0,255);
                                    break;
                                case 2:
                                    imgLabel.at<cv::Vec3b>(count-info_mask[2]-1,i) = cv::Vec3b(255,0,0);
                                    break;
                                case 3:
                                    imgLabel.at<cv::Vec3b>(count-info_mask[2]-1,i) = cv::Vec3b(255,255,0);
                                    break;
                                case 4:
                                    imgLabel.at<cv::Vec3b>(count-info_mask[2]-1,i) = cv::Vec3b(47,255,173); // greenyellow
                                    break;
                                case 5:
                                    imgLabel.at<cv::Vec3b>(count-info_mask[2]-1,i) = cv::Vec3b(128, 0, 128);
                                    break;
                                case 6:
                                    imgLabel.at<cv::Vec3b>(count-info_mask[2]-1,i) = cv::Vec3b(203,192,255);
                                    break;
                                case 7:
                                    imgLabel.at<cv::Vec3b>(count-info_mask[2]-1,i) = cv::Vec3b(196,228,255);
                                    break;
                                case 8:
                                    imgLabel.at<cv::Vec3b>(count-info_mask[2]-1,i) = cv::Vec3b(42,42,165);
                                    break;
                                case 9:
                                    imgLabel.at<cv::Vec3b>(count-info_mask[2]-1,i) = cv::Vec3b(255,255,255);
                                    break;
                                case 10:
                                    imgLabel.at<cv::Vec3b>(count-info_mask[2]-1,i) = cv::Vec3b(245,245,245); // whitesmoke
                                    break;
                                case 11:
                                    imgLabel.at<cv::Vec3b>(count-info_mask[2]-1,i) = cv::Vec3b(0,165,255); // orange
                                    break;
                                case 12:
                                    imgLabel.at<cv::Vec3b>(count-info_mask[2]-1,i) = cv::Vec3b(230,216,173); // lightblue
                                    break;
                                case 13:
                                    imgLabel.at<cv::Vec3b>(count-info_mask[2]-1,i) = cv::Vec3b(128,128,128); // grey
                                    break;
                                case 14:
                                    imgLabel.at<cv::Vec3b>(count-info_mask[2]-1,i) = cv::Vec3b(0,215,255); // gold
                                    break;
                                case 15:
                                    imgLabel.at<cv::Vec3b>(count-info_mask[2]-1,i) = cv::Vec3b(30,105,210); // chocolate
                                    break;
                            }
                        }
                    }
                    else{
                        img_mask.at<int>(count-info_mask[2]-1,i) = 0;  // -1 <-> 0
                        imgLabel.at<cv::Vec3b>(count-info_mask[2]-1,i) = cv::Vec3b(255,255,240); // azure
                    }
                    // cout << "img_mask: " << count-info_mask[2]-1 << " " << i << " " << img_mask.at<int>(count-info_mask[2]-1,i) << endl;
                }
                count++;
            }
        }
    }

    // Display the img_mask
    // cv::imshow("Semantic Mask", imgLabel);
    // cv::waitKey(0);

    return img_mask;

}
