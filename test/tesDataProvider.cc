#include "Camera.h"
#include "DataProvider.h"
#include "utils/UtilsOpenCV.h"
#include "viz/OpenCvDisplay.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

#include <thread>

DECLARE_string(test_data_path);

using namespace vdo;

class TestDataProvider : public ::testing::Test
{
public:
  TestDataProvider() : kitti_test_data_path(FLAGS_test_data_path + std::string("/KITTI/"))
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
    cv::Mat flow;
    loadFlow(kitti_test_data_path + "000000.flo", flow);
    mask = cv::Mat(flow.rows, flow.cols, CV_32SC1);
    loadSemanticMask(kitti_test_data_path + "000101.txt", mask);
  }

  std::string kitti_test_data_path;
  cv::Mat mask;
};

// TEST_F(TestDataProvider, testUniqueObject)
// {
//     cv::Mat mask_viz;
//     OpenCvDisplay::drawSemanticInstances(cv::Mat::zeros( mask.size(), CV_8UC3 ), mask, mask_viz);
//     cv::imshow( "Contours", mask_viz );
//     cv::waitKey(0);
//     cv::Mat mask_8;
//     mask.convertTo(mask_8, CV_8UC1);

//     cv::RNG rng(12345);
//     int thresh = 1;
//     cv::Mat canny_output;
//     // cv::Canny( mask_8, canny_output, thresh, thresh*3, 3 );
//     // std::vector<std::vector<cv::Point> > contours;
//     // std::vector<cv::Vec4i> hierarchy;
//     // cv::findContours( canny_output, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE );
//     // cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );
//     // LOG(ERROR) << contours.size();
//     // for( size_t i = 0; i< contours.size(); i++ )
//     // {
//     //     cv::Scalar color = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
//     //     cv::drawContours( drawing, contours, (int)i, color, CV_FILLED, cv::LINE_8, hierarchy, 0 );
//     //     // for()
//     // }

//     cv::Mat object_mask = (mask_8 > 0);
//     std::vector<cv::Mat> contours;
//     cv::Mat hierarchy;
//     findContours(object_mask, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
//     cv::Mat drawing = cv::Mat::zeros( mask.size(), CV_8UC3 );

//     LOG(ERROR) << contours.size();
//     for( size_t i = 0; i< contours.size(); i++ )
//     {
//         cv::Scalar color = cv::Scalar(static_cast<int>(i+1), 0, 0 );
//         cv::drawContours( drawing, contours, (int)i, color, CV_FILLED, cv::LINE_8, hierarchy, 0 );
//         // for()
//     }

//     cv::Mat udpated_mask = cv::Mat::zeros(mask.size(), CV_32SC1);
//     for(int i = 0; i < udpated_mask.rows; i++) {
//         for(int j = 0; j < udpated_mask.cols; j++) {
//             udpated_mask.at<int>(i, j) = drawing.at<cv::Vec3b>(i, j)[0];
//         }
//     }
//     // drawing.copyTo(udpated_mask);
//     // cv::cvtColor(drawing, udpated_mask, CV_RGB2GRAY);
//     // udpated_mask.convertTo(udpated_mask, CV_32SC1);
//     // LOG(ERROR) << contours.size();

//     OpenCvDisplay::drawSemanticInstances(cv::Mat::zeros( mask.size(), CV_8UC3 ), udpated_mask, mask_viz);
//     cv::imshow( "Updated mask", mask_viz );
//     cv::waitKey(0);

//     // cv::imshow( "Contours", drawing );
//     // cv::waitKey(0);

// }