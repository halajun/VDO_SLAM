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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>
#include<unistd.h>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps, vector<string> &vstrMask);

cv::Mat LoadMask(const string &strMask);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    vector<string> vstrMask;
    LoadImages(string(argv[3]), vstrImageLeft, vstrImageRight, vTimestamps, vstrMask);

    const int nImages = vstrImageLeft.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,false); // true

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imLeft, imRight, imMask;
    for(int ni=0; ni<50; ni++)
    {
        cout << "Processing Frame: " << ni << endl;
        // cout  << ni << " ";
        // cout << vstrMask[ni] << endl;

        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_UNCHANGED);
        imMask = LoadMask(vstrMask[ni]);

        // cout << "image size: " << imLeft.size() << endl;
        // cout << "mask size: " << imMask.size() << endl;

        double tframe = vTimestamps[ni];

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft,imRight,imMask,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryKITTI("multibody_slam.txt"); // /Users/steed/work/code/Evaluation/KITTI/KITTItest

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps, vector<string> &vstrMask)
{
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

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";
    string strMask = strPathToSequence + "/mask/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);
    vstrMask.resize(nTimes);


    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
        vstrMask[i] = strMask + ss.str() + ".png.mask";
    }
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
                if(s.compare("traffic light")==0){
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





