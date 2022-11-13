#pragma once

#include <opencv2/opencv.hpp>
#include <glog/logging.h>

#include "utils/ColorMap.h"

namespace VDO_SLAM {

struct Color {
    double a;
    double r;
    double g;
    double b;
};

//h is between 0 and 150
inline Color rainbowColorMap(int h) {
    //if > 150?
    CHECK(h < COLOURS_MAX) << "Rainbow colour map index must be less than the size of the color map itself";
    Color color;
    color.a = 255;
    color.r = colmap[h][0] * 255.0;
    color.g = colmap[h][1] * 255.0;
    color.b = colmap[h][2] * 255.0;
    return color;
}



inline cv::Scalar colorToCvScalar(const Color& color) {
    return cv::Scalar(color.r, color.g, color.b, color.a);
}

inline cv::Scalar rainbowColorMapCv(int h) {
    return colorToCvScalar(rainbowColorMap(h));
}



} //VDO_SLAM