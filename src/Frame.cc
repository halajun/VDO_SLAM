#include "Frame.h"
#include "Types.h"
#include "Frontend-Definitions.h"
#include "utils/UtilsOpenCV.h"

#include <glog/logging.h>

namespace vdo
{
std::size_t Frame::tracklet_id_count{0};

Frame::Frame(const ImagePacket& images_, Timestamp timestamp_, size_t frame_id_, const CameraParams& cam_params_)
  : images(images_), timestamp(timestamp_), frame_id(frame_id_), cam_params(cam_params_)
{
}

void Frame::refreshFeatures(ORBextractor::UniquePtr& detector, double depth_background_thresh) {
  observations.clear();
  static_features.clear();
  detectFeatures(detector);
  processStaticFeatures(depth_background_thresh);
}


void Frame::addStaticFeatures(const Observations& observations_) {
  observations = observations_;
  LOG(INFO) << "Added observations - " << observations.size();
  // undistortKeypoints(keypoints, keypoints);
}

Feature::Ptr Frame::getStaticFeature(std::size_t tracklet_id) const {
  if(static_features.find(tracklet_id) == static_features.end()) {
    return nullptr;
  }
  else {
    return static_features.at(tracklet_id);
  }
}


void Frame::detectFeatures(ORBextractor::UniquePtr& detector)
{
  cv::Mat mono;
  prepareRgbForDetection(images.rgb, mono);
  KeypointsCV keypoints;
  (*detector)(mono, cv::Mat(), keypoints, descriptors);
  LOG(INFO) << "Detected " << keypoints.size() << " kp";
  if (keypoints.size() == 0)
  {
    LOG(ERROR) << "zero features detected - frame " << frame_id;
    //what if we have zero features but some left over observations
  }
    LOG(INFO) << "before new detections " << observations.size();

  //TODO: make param
  //TODO: this will  add onto the exsiting obs -> we should try and spread the kp's around using something like NMS
  for(const KeypointCV& kp : keypoints) {
    Observation obs;
    obs.keypoint = kp;
    obs.tracklet_id = Frame::tracklet_id_count;
    Frame::tracklet_id_count++;
    obs.type = Observation::Type::DETECTION;
    observations.push_back(obs);
  }

  LOG(INFO) << "after new detections " << observations.size();

  // undistortKeypoints(keypoints, keypoints);
}

void Frame::projectKeypoints(const Camera& camera)
{
  for (const auto& feature_pair : static_features)
  {
    const Feature& feature = *feature_pair.second;
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
  CHECK_EQ(static_features.size(), 0u);
  if (observations.size() == 0)
  {
    LOG(ERROR) << "Cannot process flow correspondences with zero keypoints - frame " << frame_id;
    return;
  }

  const cv::Mat& ref_image = images.rgb;
  LOG(INFO) << "Processing static obs " << observations.size();
  for (int i = 0; i < observations.size(); ++i)
  {
    const Observation& obs = observations[i];
    
    int x = obs.keypoint.pt.x;
    int y = obs.keypoint.pt.y;

    //check is in image bounds
    if(x < 0 || y < 0 || x >= ref_image.cols || y >= ref_image.rows) {
      LOG(WARNING) << "Obs is outside image bounds";
      continue;
    }

    CHECK_EQ(images.semantic_mask.size(), ref_image.size());
    CHECK_EQ(images.depth.size(), ref_image.size());
    CHECK_EQ(images.flow.size(), ref_image.size());
    if (images.semantic_mask.at<int>(y, x) != 0)  // new added in Jun 13 2019
      continue;

    if (images.depth.at<double>(y, x) > depth_background_thresh ||
        images.depth.at<double>(y, x) <= 0)  // new added in Aug 21 2019
      continue;

    float flow_xe = images.flow.at<cv::Vec2f>(y, x)[0];
    float flow_ye = images.flow.at<cv::Vec2f>(y, x)[1];
    double flow_xe_d = static_cast<double>(flow_xe);
    double flow_ye_d = static_cast<double>(flow_ye);

    //WHY ONLY WHEN WE HAVE FLOW?
    if (flow_xe != 0 && flow_ye != 0)
    {
      if (x + flow_xe < ref_image.cols && y + flow_ye < ref_image.rows && x < ref_image.cols && y < ref_image.rows)
      {
        Feature::Ptr feature = std::make_shared<Feature>();
        feature->keypoint = obs.keypoint;
        feature->index = i;
        feature->frame_id = frame_id;
        feature->type = Feature::Type::STATIC;
        Depth d = images.depth.at<double>(y, x);  // be careful with the order  !!!
        if (d > 0)
        {
          feature->depth = d;
        }
        else
        {
          // log warning?
          feature->depth = -1;
        }

        feature->optical_flow = cv::Point2d(flow_xe_d, flow_ye_d);
        feature->predicted_keypoint =
            cv::KeyPoint(obs.keypoint.pt.x + flow_xe, obs.keypoint.pt.y + flow_ye, 0, 0, 0, obs.keypoint.octave, -1);
        feature->instance_label = Feature::background;
        feature->tracklet_id = obs.tracklet_id;
        CHECK(feature->tracklet_id != -1);
        // // mvStatKeysTmp.push_back(mvKeys[i]);
        // predicted_keypoints_static.push_back(cv::KeyPoint(keypoints[i].pt.x+flow_xe,keypoints[i].pt.y+flow_ye,0,0,0,keypoints[i].octave,-1));
        // predicted_optical_flow.push_back(cv::Point2d(flow_xe,flow_ye));

        static_features.insert({obs.tracklet_id, feature});
      }
    }
  }
  LOG(INFO) << "Processed static features " << static_features.size();
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
        float flow_x = images.flow.at<cv::Vec2f>(i, j)[0];
        float flow_y = images.flow.at<cv::Vec2f>(i, j)[1];
        

        double flow_x_d = static_cast<double>(flow_x);
        double flow_y_d = static_cast<double>(flow_y);

        // we are within the image bounds?
        if (j + flow_x < ref_image.cols && j + flow_x > 0 && i + flow_y < ref_image.rows && i + flow_y > 0)
        {
          Feature feature;
          feature.keypoint = cv::KeyPoint(j, i, 0, 0, 0, -1);
          feature.index = i;
          feature.frame_id = frame_id;
          feature.type = Feature::Type::DYNAMIC;
          feature.depth = depth;

          feature.optical_flow = cv::Point2d(flow_x_d, flow_y_d);
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
  for (const auto& feature_pair : static_features)
  {
    const Feature& feature = *feature_pair.second;
    cv::Point2d point(feature.keypoint.pt.x, feature.keypoint.pt.y);
    utils::DrawCircleInPlace(image, point, cv::Scalar(0, 255, 0));
    cv::arrowedLine(image, feature.keypoint.pt, feature.predicted_keypoint.pt, cv::Scalar(255, 0, 0));
    cv::putText(image, std::to_string(feature.tracklet_id), cv::Point2i(feature.keypoint.pt.x - 10, feature.keypoint.pt.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.3,
                      cv::Scalar(255, 0, 0));
  }
  
}

void Frame::drawDynamicFeatures(cv::Mat& image) const
{
  for (const Feature& feature : dynamic_features)
  {
    cv::Point2d point(feature.keypoint.pt.x, feature.keypoint.pt.y);
    utils::DrawCircleInPlace(image, point, cv::Scalar(0, 0, 255));

    
    cv::arrowedLine(image, feature.keypoint.pt, feature.predicted_keypoint.pt, cv::Scalar(0, 0, 255));
  }

  //TODO: refactor all this - for now just draw this point to predicted

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