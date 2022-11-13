#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <gtsam/geometry/Pose3.h>
#include <boost/optional.hpp>



namespace vdo {

using Timestamp = double;

template<typename T>
using VectorsXx = std::vector<std::vector<T>>;

//double spexalisation
using VectorsXd = VectorsXx<double>;
//int specalisation
using VectorsXi = VectorsXx<int>;

struct InputPacket {

    const Timestamp timestamp;
    const cv::Mat rgb; //RGB (CV_8UC3) or grayscale (CV_8U)
    const cv::Mat depth;
    const cv::Mat flow; //Float (CV_32F).
    const cv::Mat semantic_mask;

    InputPacket(const Timestamp timestamp_, const cv::Mat& rgb_, const cv::Mat& depth_, const cv::Mat& flow_, const cv::Mat& semantic_mask_)
    : timestamp(timestamp_), rgb(rgb_), depth(depth_), flow(flow_), semantic_mask(semantic_mask_) {}

};


struct GroundTruthInputPacket {

    using ConstOptional = boost::optional<const GroundTruthInputPacket&>;
    using Optional = boost::optional<GroundTruthInputPacket&>;

    gtsam::Pose3 X_wc; //pose of the cmaera in the world frame
    VectorsXd obj_poses;
    Timestamp timestamp;
};

}