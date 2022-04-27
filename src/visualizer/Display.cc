#include "visualizer/Display.h"
#include <opencv2/opencv.hpp>
#include <glog/logging.h>

namespace VDO_SLAM {

cv::Scalar Display::getObjectColour(int label) {
    if (label > 25) {
        label = label/2;
    }

    switch(label) {
        case 0:
            return cv::Scalar(0,0,255, 255); // red
        case 1:
            return cv::Scalar(128, 0, 128, 255); // 255, 165, 0
        case 2:
            return cv::Scalar(255,255,0, 255);           
        case 3:
            return cv::Scalar(0, 255, 0, 255); // 255,255,0            
        case 4:
            return cv::Scalar(255,0,0, 255); // 255,192,203           
        case 5:
            return cv::Scalar(0,255,255, 255);            
        case 6:
            return cv::Scalar(128, 0, 128, 255);            
        case 7:
            return cv::Scalar(255,255,255, 255);           
        case 8:
            return cv::Scalar(255,228,196, 255);            
        case 9:
            return cv::Scalar(180, 105, 255, 255);            
        case 10:
            return cv::Scalar(165,42,42, 255);           
        case 11:
            return cv::Scalar(35, 142, 107, 255);           
        case 12:
            return cv::Scalar(45, 82, 160, 255);           
        case 13:
            return cv::Scalar(0,0,255, 255); // red            
        case 14:
            return cv::Scalar(255, 165, 0, 255);            
        case 15:
            return cv::Scalar(0,255,0, 255);           
        case 16:
            return cv::Scalar(255,255,0, 255);            
        case 17:
            return cv::Scalar(255,192,203, 255);            
        case 18:
            return cv::Scalar(0,255,255, 255);            
        case 19:
            return cv::Scalar(128, 0, 128, 255);            
        case 20:
            return cv::Scalar(255,255,255, 255);           
        case 21:
            return cv::Scalar(255,228,196, 255);            
        case 22:
            return cv::Scalar(180, 105, 255, 255);           
        case 23:
            return cv::Scalar(165,42,42, 255);           
        case 24:
            return cv::Scalar(35, 142, 107, 255);          
        case 25:
            return cv::Scalar(45, 82, 160, 255);          
        case 41:
            return cv::Scalar(60, 20, 220, 255);
            
    }
}


}