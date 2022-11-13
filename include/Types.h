#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <gtsam/geometry/Pose3.h>
#include <boost/optional.hpp>



namespace vdo {

using Timestamp = double;
using Depth = double;
using InstanceLabel = int;
using Landmark = gtsam::Point3;

using KeypointsCV = std::vector<cv::KeyPoint>;
using Landmarks = std::vector<Landmark>;

template<typename T>
using VectorsXx = std::vector<std::vector<T>>;

//double spexalisation
using VectorsXd = VectorsXx<double>;
//int specalisation
using VectorsXi = VectorsXx<int>;


struct ImagePacket {
  cv::Mat rgb; //RGB (CV_8UC3) or grayscale (CV_8U)
  cv::Mat depth; 
  cv::Mat flow; //Float (CV_32F).
  cv::Mat semantic_mask;

  ImagePacket() {}
  ImagePacket(const cv::Mat& rgb_, const cv::Mat& depth_, const cv::Mat& flow_, const cv::Mat& semantic_mask_)
  : rgb(rgb_.clone()), depth(depth_.clone()), flow(flow_.clone()), semantic_mask(semantic_mask_.clone()) {}

};

struct InputPacket {

    Timestamp timestamp;
    size_t frame_id;

    ImagePacket images;

    InputPacket() {}

    InputPacket(const Timestamp timestamp_, const size_t frame_id_, const cv::Mat& rgb_, const cv::Mat& depth_, const cv::Mat& flow_, const cv::Mat& semantic_mask_)
    : timestamp(timestamp_), frame_id(frame_id_), images(rgb_, depth_, flow_, semantic_mask_) {}

    // InputPacket(const InputPacket &frame);

    // // assignment operator
    // Frame& operator=(const Frame &frame);


};

enum class DistortionModel
{
  NONE,
  RADTAN,
  EQUIDISTANT,
  FISH_EYE
};
inline std::string distortionToString(const DistortionModel& model)
{
  switch (model)
  {
    case DistortionModel::EQUIDISTANT:
      return "equidistant";
    case DistortionModel::FISH_EYE:
      return "fish_eye";
    case DistortionModel::NONE:
      return "none";
    case DistortionModel::RADTAN:
      return "radtan";
    default:
      break;
  }
}


struct ObjectPoseGT {
    size_t frame_id;
    size_t object_id;

    struct BoundingBox {
        int b1, b2, b3, b4;
    };

    BoundingBox bounding_box;
    gtsam::Point3 translation; //3d object location in camera coordinates
    double r1; //rotation around y-axis in camera coordinates
};


struct GroundTruthInputPacket {

    using ConstOptional = boost::optional<const GroundTruthInputPacket&>;
    using Optional = boost::optional<GroundTruthInputPacket&>;

    gtsam::Pose3 X_wc; //pose of the cmaera in the world frame
    std::vector<ObjectPoseGT> obj_poses;
    Timestamp timestamp;
    size_t frame_id;
};

}