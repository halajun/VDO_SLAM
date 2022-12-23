#include "Camera.h"
#include "DataProvider.h"
#include "utils/UtilsOpenCV.h"

#include "ORBextractor.h"
#include "Frontend-Definitions.h"  //for TrackingParams

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

#include <thread>

DECLARE_string(test_data_path);

struct Frame
{
  VDO_POINTER_TYPEDEFS(Frame);
  // vdo::TrackletIdFeatureMap static_features;
  vdo::FeaturePtrs features_;
  // dynamic and other stuff
};

class FeatureTracker
{
public:
  FeatureTracker(const vdo::TrackingParams& tracking_params, vdo::Camera camera)
    : tracking_params_(tracking_params), camera_(camera), initial_computation_(true), previous_frame_(nullptr)
  {
    feature_detector_ = vdo::make_unique<vdo::ORBextractor>(
        tracking_params_.n_features, static_cast<float>(tracking_params_.scale_factor), tracking_params_.n_levels,
        tracking_params_.init_threshold_fast, tracking_params_.min_threshold_fast);

    CHECK(initial_computation_);
  }

  Frame::Ptr track(const vdo::ImagePacket& image_packet, size_t& n_optical_flow, size_t& n_new_tracks)
  {
    if (initial_computation_)
    {
      CHECK(!previous_frame_);
      img_size_ = image_packet.rgb.size();
      computeImageBounds(img_size_, min_x_, max_x_, min_y_, max_y_);

      grid_elements_width_inv_ = static_cast<double>(FRAME_GRID_COLS) / static_cast<double>(max_x_ - min_x_);
      grid_elements_height_inv_ = static_cast<double>(FRAME_GRID_ROWS) / static_cast<double>(max_y_ - min_y_);

      LOG(INFO) << grid_elements_width_inv_ << " " << grid_elements_height_inv_;

      initial_computation_ = false;
    }

    // convert rgb image to mono for detection
    // TODO: make function
    const cv::Mat& rgb = image_packet.rgb;
    const cv::Mat& depth = image_packet.depth;
    const cv::Mat& flow = image_packet.flow;
    cv::Mat mono;
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

    // run detetions
    cv::Mat descriptors;
    vdo::KeypointsCV detected_keypoints;
    (*feature_detector_)(mono, cv::Mat(), detected_keypoints, descriptors);

    vdo::FeaturePtrs features;

    // appy tracking
    if (previous_frame_)
    {
      for (vdo::Feature::Ptr feature : previous_frame_->features_)
      {
        const size_t tracklet_id = feature->tracklet_id;
        const size_t age = feature->age;
        vdo::KeypointCV kp = feature->predicted_keypoint;

        // if camera contained
        if (camera_.isKeypointContained(kp, feature->depth) && feature->inlier)
        {
          size_t new_age = age + 1;
          vdo::Feature::Ptr new_feature = std::make_shared<vdo::Feature>();
          new_feature->age = new_age;
          new_feature->keypoint = kp;
          new_feature->tracklet_id = tracklet_id;

          new_feature->depth = 1;
          // new_feature->type = Feature::Type::STATIC;

          int x = kp.pt.x;
          int y = kp.pt.y;

          double flow_xe = static_cast<double>(flow.at<cv::Vec2f>(y, x)[0]);
          double flow_ye = static_cast<double>(flow.at<cv::Vec2f>(y, x)[1]);
          new_feature->optical_flow = cv::Point2d(flow_xe, flow_ye);
          new_feature->predicted_keypoint = cv::KeyPoint(x + flow_xe, y + flow_ye, 0, 0, 0, kp.octave, -1);

          // check still in image plane?>
          // TODO: all the others
          // and is still a static feature (ie not in a semantic mask)
          features.push_back(new_feature);
        }
      }
    }

    n_optical_flow = features.size();
    LOG(INFO) << "tracked with optical flow - " << n_optical_flow;

    const int& min_tracks = tracking_params_.n_features;
    // Assign Features to Grid Cells
    int n_reserve = (FRAME_GRID_COLS * FRAME_GRID_ROWS) / (0.5 * min_tracks);
    LOG(INFO) << "reserving - " << n_reserve;
    for (unsigned int i = 0; i < FRAME_GRID_COLS; i++)
      for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++)
        grid_[i][j].reserve(n_reserve);

    // assign tracked features to grid
    for (vdo::Feature::Ptr feature : features)
    {
      const cv::KeyPoint& kp = feature->keypoint;
      int grid_x, grid_y;
      if (posInGrid(kp, grid_x, grid_y))
      {
        grid_[grid_x][grid_y].push_back(feature->tracklet_id);
      }
    }

    // only add new features if tracks drop below min tracks
    if (features.size() < min_tracks)
    {
      // iterate over new observations
      for (cv::KeyPoint kp : detected_keypoints)
      {
        // TODO: if not object etc etc

        int grid_x, grid_y;
        if (posInGrid(kp, grid_x, grid_y))
        {
          // only add of we have less than n_reserve ammount
          if (grid_[grid_x][grid_y].size() < n_reserve)
          {
            vdo::Feature::Ptr new_feature = std::make_shared<vdo::Feature>();
            new_feature->age = 0;
            new_feature->keypoint = kp;
            new_feature->tracklet_id = tracklet_count;
            tracklet_count++;

            new_feature->depth = 1;
            // new_feature->type = Feature::Type::STATIC;

            int x = kp.pt.x;
            int y = kp.pt.y;

            double flow_xe = static_cast<double>(flow.at<cv::Vec2f>(y, x)[0]);
            double flow_ye = static_cast<double>(flow.at<cv::Vec2f>(y, x)[1]);
            new_feature->optical_flow = cv::Point2d(flow_xe, flow_ye);
            new_feature->predicted_keypoint = cv::KeyPoint(x + flow_xe, y + flow_ye, 0, 0, 0, kp.octave, -1);

            grid_[grid_x][grid_y].push_back(new_feature->tracklet_id);
            features.push_back(new_feature);
          }
        }
      }
    }

    size_t total_tracks = features.size();
    n_new_tracks = total_tracks - n_optical_flow;
    LOG(INFO) << "new tracks - " << n_optical_flow;
    LOG(INFO) << "total tracks - " << total_tracks;

    // for debug lets draw tracks
    cv::Mat viz;
    rgb.copyTo(viz);

    int width = viz.size().width;
    int height = viz.size().height;

    int x_dist = width / FRAME_GRID_COLS;
    int y_dist = height / FRAME_GRID_ROWS;

    for (int i = 0; i < height; i += x_dist)
      cv::line(viz, cv::Point(0, i), cv::Point(width, i), cv::Scalar(255, 255, 255));

    for (int i = 0; i < width; i += y_dist)
      cv::line(viz, cv::Point(i, 0), cv::Point(i, height), cv::Scalar(255, 255, 255));

    for (vdo::Feature::Ptr feature : features)
    {
      cv::Point2d point(feature->keypoint.pt.x, feature->keypoint.pt.y);
      if (feature->age == 0)
      {
        vdo::utils::DrawCircleInPlace(viz, point, cv::Scalar(0, 255, 0), 1);
      }
      else
      {
        vdo::utils::DrawCircleInPlace(viz, point, cv::Scalar(0, 0, 255), 1);
      }
    }

    Frame::Ptr frame = std::make_shared<Frame>();
    frame->features_ = features;
    previous_frame_ = frame;

    cv::imshow("disp", viz);
    cv::waitKey(0);

    return nullptr;
  }

public:
  vdo::TrackingParams tracking_params_;
  vdo::Camera camera_;
  vdo::ORBextractor::UniquePtr feature_detector_;

  size_t tracklet_count = 0;

  static constexpr int FRAME_GRID_ROWS = 48;
  static constexpr int FRAME_GRID_COLS = 64;

  // grid of trackled Id's
  std::vector<std::size_t> grid_[FRAME_GRID_COLS][FRAME_GRID_ROWS];

  bool initial_computation_;
  Frame::Ptr previous_frame_;

  // Compute the cell of a keypoint (return false if outside the grid)
  bool posInGrid(const cv::KeyPoint& kp, int& pos_x, int& pos_y)
  {
    pos_x = round((kp.pt.x - min_x_) * grid_elements_width_inv_);
    pos_y = round((kp.pt.y - min_y_) * grid_elements_height_inv_);

    // Keypoint's coordinates are undistorted, which could cause to go out of the image
    if (pos_x < 0 || pos_x >= FRAME_GRID_COLS || pos_y < 0 || pos_y >= FRAME_GRID_ROWS)
      return false;

    return true;
  }

  void computeImageBounds(const cv::Size& size, int& min_x, int& max_x, int& min_y, int& max_y) const
  {
    // TODO: distortion
    min_x = 0;
    max_x = size.width;
    min_y = 0;
    max_y = size.height;
  }

  int min_x_;
  int min_y_;
  int max_x_;
  int max_y_;
  cv::Size img_size_;  // set on first computation
  double grid_elements_width_inv_;
  double grid_elements_height_inv_;
};

using namespace vdo;

class TestTracker : public ::testing::Test
{
public:
  TestTracker() : kitti_test_data_path(FLAGS_test_data_path + std::string("/KITTI/"))
  {
    // Fix randomization seed (we use monte carlo runs here).
    srand(3);
    InitializeData();
  }

protected:
  void SetUp() override
  {
  }
  // TODO deallocate dynamic memory here!
  void TearDown() override
  {
  }

  void InitializeData()
  {
    loadFlow(kitti_test_data_path + "000000.flo", flow0);
    loadFlow(kitti_test_data_path + "000001.flo", flow1);
    loadFlow(kitti_test_data_path + "000002.flo", flow2);

    loadRGB(kitti_test_data_path + "000000.png", rgb0);
    loadRGB(kitti_test_data_path + "000001.png", rgb1);
    loadRGB(kitti_test_data_path + "000002.png", rgb2);
    loadRGB(kitti_test_data_path + "000003.png", rgb3);

    // TODO:
    tracking_params_.n_features = 1000;
    tracking_params_.scale_factor = 1.2;
    tracking_params_.n_levels = 8;
    tracking_params_.init_threshold_fast = 20;
    tracking_params_.min_threshold_fast = 7;
  }

  std::string kitti_test_data_path;
  cv::Mat flow0, flow1, flow2;
  cv::Mat rgb0, rgb1, rgb2, rgb3;

  TrackingParams tracking_params_;
};

TEST_F(TestTracker, testShow)
{
  CameraParams params(CameraParams::Intrinsics{ 718.8560, 718.8560, 607.1928, 185.2157 },
                      CameraParams::Distortion{ 0, 0, 0, 0 }, cv::Size(1242, 375), "none", 388.1822);

  Camera camera(params);
  FeatureTracker tracker(tracking_params_, camera);

  vdo::ImagePacket packet0(rgb0, cv::Mat(), flow0, cv::Mat());
  vdo::ImagePacket packet1(rgb1, cv::Mat(), flow1, cv::Mat());
  vdo::ImagePacket packet2(rgb2, cv::Mat(), flow2, cv::Mat());
  size_t n_optical_flow, n_new_tracks;
  tracker.track(packet0, n_optical_flow, n_new_tracks);
  tracker.track(packet1, n_optical_flow, n_new_tracks);
  tracker.track(packet2, n_optical_flow, n_new_tracks);

  // cv::Mat flow_viz;
  // utils::flowToRgb(flow1,flow_viz);
  // cv::Mat disp = utils::concatenateImagesHorizontally(rgb1, flow_viz);
  // cv::imshow("disp", disp);
  // cv::waitKey(0);
}