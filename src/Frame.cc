#include "Frame.h"
#include "Types.h"
#include "utils/UtilsOpenCV.h"

#include <glog/logging.h>

namespace vdo
{
Frame::Frame(const ImagePacket& images_, Timestamp timestamp_, size_t frame_id_, const CameraParams& cam_params_)
  : images(images_), timestamp(timestamp_), frame_id(frame_id_), cam_params(cam_params_)
{
}

void Frame::detectFeatures(ORBextractor::UniquePtr& detector)
{
  cv::Mat mono;
  prepareRgbForDetection(images.rgb, mono);

  (*detector)(mono, cv::Mat(), keypoints, descriptors);
  LOG(INFO) << "Detected " << keypoints.size() << " kp";

  if (keypoints.size() == 0)
  {
    LOG(ERROR) << "zero features detected - frame " << frame_id;
  }

  undistortKeypoints(keypoints, keypoints);
}

void Frame::projectKeypoints(const Camera& camera)
{
  for (const Feature& feature : static_features)
  {
    CHECK(feature.depth != -1);
    Landmark lmk;
    camera.backProject(feature.keypoint, feature.depth, &lmk);
    static_landmarks.push_back(lmk);
  }

  for (const Feature& feature : dynamic_features)
  {
    CHECK(feature.depth != -1);
    Landmark lmk;
    camera.backProject(feature.keypoint, feature.depth, &lmk);
    dynamic_landmarks.push_back(lmk);
  }

  CHECK_EQ(static_features.size(), static_landmarks.size());
  CHECK_EQ(dynamic_features.size(), dynamic_landmarks.size());
}

void Frame::processStaticFeatures(double depth_background_thresh)
{
  if (keypoints.size() == 0)
  {
    LOG(ERROR) << "Cannot process flow correspondences with zero keypoints - frame " << frame_id;
  }

  const cv::Mat& ref_image = images.rgb;

  for (int i = 0; i < keypoints.size(); ++i)
  {
    int x = keypoints[i].pt.x;
    int y = keypoints[i].pt.y;

    if (images.semantic_mask.at<int>(y, x) != 0)  // new added in Jun 13 2019
      continue;

    if (images.depth.at<double>(y, x) > depth_background_thresh ||
        images.depth.at<double>(y, x) <= 0)  // new added in Aug 21 2019
      continue;

    double flow_xe = images.flow.at<cv::Vec2d>(y, x)[0];
    double flow_ye = images.flow.at<cv::Vec2d>(y, x)[1];

    if (flow_xe != 0 && flow_ye != 0)
    {
      if (x + flow_xe < ref_image.cols && y + flow_ye < ref_image.rows && x < ref_image.cols && y < ref_image.rows)
      {
        Feature feature;
        feature.keypoint = keypoints[i];
        feature.index = i;
        feature.frame_id = frame_id;
        feature.type = Feature::Type::STATIC;

        Depth d = images.depth.at<double>(y, x);  // be careful with the order  !!!
        if (d > 0)
        {
          feature.depth = d;
        }
        else
        {
          // log warning?
          feature.depth = -1;
        }

        feature.optical_flow = cv::Point2d(flow_xe, flow_ye);
        feature.predicted_keypoint =
            cv::KeyPoint(keypoints[i].pt.x + flow_xe, keypoints[i].pt.y + flow_ye, 0, 0, 0, keypoints[i].octave, -1);
        feature.instance_label = Feature::background;
        // // mvStatKeysTmp.push_back(mvKeys[i]);
        // predicted_keypoints_static.push_back(cv::KeyPoint(keypoints[i].pt.x+flow_xe,keypoints[i].pt.y+flow_ye,0,0,0,keypoints[i].octave,-1));
        // predicted_optical_flow.push_back(cv::Point2d(flow_xe,flow_ye));

        static_features.push_back(feature);
      }
    }
  }
}

void Frame::processDynamicFeatures(double depth_object_thresh)
{
  const cv::Mat& ref_image = images.rgb;
  // semi-dense features on objects
  int step = 4;  // 3
  for (int i = 0; i < ref_image.rows; i = i + step)
  {
    for (int j = 0; j < ref_image.cols; j = j + step)
    {
      InstanceLabel instance_label = images.semantic_mask.at<InstanceLabel>(i, j);
      Depth depth = images.depth.at<double>(i, j);
      // check ground truth motion mask
      if (instance_label != 0 && depth < depth_object_thresh && depth > 0)
      {
        double flow_x = images.flow.at<cv::Vec2d>(i, j)[0];
        double flow_y = images.flow.at<cv::Vec2d>(i, j)[1];

        // we are within the image bounds?
        if (j + flow_x < ref_image.cols && j + flow_x > 0 && i + flow_y < ref_image.rows && i + flow_y > 0)
        {
          Feature feature;
          feature.keypoint = cv::KeyPoint(j, i, 0, 0, 0, -1);
          feature.index = i;
          feature.frame_id = frame_id;
          feature.type = Feature::Type::DYNAMIC;
          feature.depth = depth;

          feature.optical_flow = cv::Point2d(flow_x, flow_y);
          feature.predicted_keypoint = cv::KeyPoint(j + flow_x, i + flow_y, 0, 0, 0, -1);
          feature.instance_label = instance_label;
          // // save correspondences
          // mvObjFlowNext.push_back(cv::Point2f(flow_x,flow_y));
          // //
          // mvObjCorres.push_back(cv::KeyPoint(j+flow_x,i+flow_y,0,0,0,-1));
          // // save pixel location
          // //and setting classid = -1
          // mvObjKeys.push_back(cv::KeyPoint(j,i,0,0,0,-1));
          // // save depth
          // mvObjDepth.push_back(imDepth.at<float>(i,j));
          // // save label
          // vSemObjLabel.push_back(maskSEM.at<int>(i,j));
          dynamic_features.push_back(feature);
        }
      }
    }
  }
}

void Frame::undistortKeypoints(const KeypointsCV& distorted, KeypointsCV& undistorted)
{
  if (cam_params.distortion_coeff[0] == 0.0)
  {
    undistorted = distorted;
    return;
  }
  const size_t N = distorted.size();
  // Fill matrix with points
  cv::Mat mat(N, 2, CV_32F);
  for (size_t i = 0; i < N; i++)
  {
    mat.at<float>(i, 0) = distorted[i].pt.x;
    mat.at<float>(i, 1) = distorted[i].pt.y;
  }

  // Undistort points
  mat = mat.reshape(2);
  cv::undistortPoints(mat, mat, cam_params.K, cam_params.D, cv::Mat());
  mat = mat.reshape(1);

  // Fill undistorted keypoint vector
  undistorted.resize(N);
  // TODO: issue going between float and double?
  for (size_t i = 0; i < N; i++)
  {
    cv::KeyPoint kp = distorted[i];
    kp.pt.x = mat.at<float>(i, 0);
    kp.pt.y = mat.at<float>(i, 1);
    undistorted[i] = kp;
  }
}

void Frame::drawStaticFeatures(cv::Mat& image) const
{
  // assumes image is sized appropiately
  for (const Feature& feature : static_features)
  {
    cv::Point2d point(feature.keypoint.pt.x, feature.keypoint.pt.y);
    utils::DrawCircleInPlace(image, point, cv::Scalar(0, 255, 0));
  }
}

void Frame::drawDynamicFeatures(cv::Mat& image) const
{
  for (const Feature& feature : dynamic_features)
  {
    cv::Point2d point(feature.keypoint.pt.x, feature.keypoint.pt.y);
    utils::DrawCircleInPlace(image, point, cv::Scalar(0, 0, 255));
  }
}

void Frame::prepareRgbForDetection(const cv::Mat& rgb, cv::Mat& mono)
{
  CHECK(!rgb.empty());
  PLOG_IF(ERROR, rgb.channels() == 1) << "Input image should be RGB (channels == 3), not 1";
  // Transfer color image to grey image
  rgb.copyTo(mono);

  if (mono.channels() == 3)
  {
    cv::cvtColor(mono, mono, CV_RGB2GRAY);
  }
  else if (rgb.channels() == 4)
  {
    cv::cvtColor(mono, mono, CV_RGBA2GRAY);
  }
}

}  // namespace vdo