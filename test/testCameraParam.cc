#include "Camera.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <thread>

using namespace vdo;

/**
 * @brief compareKeypoints compares two sets of keypoints
 */
void compareKeypoints(const KeypointsCV& kpts_1, const KeypointsCV& kpts_2, const float& tol)
{
  ASSERT_EQ(kpts_1.size(), kpts_2.size());
  for (size_t i = 0u; i < kpts_1.size(); i++)
  {
    const auto& kpt_1 = kpts_1[i];
    const auto& kpt_2 = kpts_2[i];
    EXPECT_NEAR(kpt_1.pt.x, kpt_2.pt.x, tol);
    EXPECT_NEAR(kpt_1.pt.y, kpt_2.pt.y, tol);
  }
}

/**
 * @brief compareLandmarks compares two sets of landmarks
 */
void compareLandmarks(const Landmarks& lmks_1, const Landmarks& lmks_2, const float& tol)
{
  ASSERT_EQ(lmks_1.size(), lmks_2.size());
  for (size_t i = 0u; i < lmks_1.size(); i++)
  {
    const auto& lmk_1 = lmks_1[i];
    const auto& lmk_2 = lmks_2[i];
    EXPECT_NEAR(lmk_1.x(), lmk_2.x(), tol);
    EXPECT_NEAR(lmk_1.y(), lmk_2.y(), tol);
    EXPECT_NEAR(lmk_1.z(), lmk_2.z(), tol);
  }
}

TEST(testCameraParams, basicConstruction)
{
  // Intrinsics.
  const std::vector<double> intrinsics_expected = { 458.654, 457.296, 367.215, 248.375 };
  //   // Distortion coefficients.
  const std::vector<double> distortion_expected = { -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05 };

  const cv::Size size_expected(752, 480);

  const std::string expected_distortion_model = "equidistant";

  // Sensor extrinsics wrt. the body-frame.
  gtsam::Rot3 R_expected(0.0148655429818, -0.999880929698, 0.00414029679422, 0.999557249008, 0.0149672133247,
                         0.025715529948, -0.0257744366974, 0.00375618835797, 0.999660727178);
  gtsam::Point3 T_expected(-0.0216401454975, -0.064676986768, 0.00981073058949);
  gtsam::Pose3 pose_expected(R_expected, T_expected);

  double expected_base_line = 0.05;

  CameraParams params(intrinsics_expected, distortion_expected, size_expected, expected_distortion_model,
                      expected_base_line);

  EXPECT_EQ(size_expected.width, params.ImageWidth());
  EXPECT_EQ(size_expected.height, params.ImageHeight());
  EXPECT_EQ(expected_base_line, params.baseline);

  for (size_t c = 0u; c < 4u; c++)
  {
    EXPECT_DOUBLE_EQ(intrinsics_expected[c], params.intrinsics[c]);
  }
  EXPECT_DOUBLE_EQ(intrinsics_expected[0], params.K.at<double>(0, 0));
  EXPECT_DOUBLE_EQ(intrinsics_expected[1], params.K.at<double>(1, 1));
  EXPECT_DOUBLE_EQ(intrinsics_expected[2], params.K.at<double>(0, 2));
  EXPECT_DOUBLE_EQ(intrinsics_expected[3], params.K.at<double>(1, 2));

  EXPECT_DOUBLE_EQ(intrinsics_expected[0], params.fx());
  EXPECT_DOUBLE_EQ(intrinsics_expected[1], params.fy());
  EXPECT_DOUBLE_EQ(intrinsics_expected[2], params.cu());
  EXPECT_DOUBLE_EQ(intrinsics_expected[3], params.cv());
  //   EXPECT_EQ(cam_params.intrinsics_.size(), 4u);
  //   gtsam::Cal3DS2 gtsam_calib;
  //   CameraParams::createGtsamCalibration(cam_params.distortion_coeff_mat_,
  //                                        cam_params.intrinsics_,
  //                                        &gtsam_calib);
  //   EXPECT_DOUBLE_EQ(intrinsics_expected[0], gtsam_calib.fx());
  //   EXPECT_DOUBLE_EQ(intrinsics_expected[1], gtsam_calib.fy());
  //   EXPECT_DOUBLE_EQ(0u, gtsam_calib.skew());
  //   EXPECT_DOUBLE_EQ(intrinsics_expected[2], gtsam_calib.px());
  //   EXPECT_DOUBLE_EQ(intrinsics_expected[3], gtsam_calib.py());

  for (size_t c = 0u; c < 4u; c++)
  {
    EXPECT_DOUBLE_EQ(distortion_expected[c], params.D.at<double>(c));
  }
  EXPECT_EQ(params.D.rows, 1u);
  EXPECT_EQ(params.D.cols, 4u);
  //   EXPECT_DOUBLE_EQ(distortion_expected[0], gtsam_calib.k1());
  //   EXPECT_DOUBLE_EQ(distortion_expected[1], gtsam_calib.k2());
  //   EXPECT_DOUBLE_EQ(distortion_expected[2], gtsam_calib.p1());
  //   EXPECT_DOUBLE_EQ(distortion_expected[3], gtsam_calib.p2());
}

TEST(testCameraParams, convertDistortionVectorToMatrix)
{
  std::vector<double> distortion_coeffs;

  // 4 distortion params
  distortion_coeffs = { 1.0, -2.0, 1.3, 10 };
  cv::Mat distortion_coeffs_mat;
  CameraParams::convertDistortionVectorToMatrix(distortion_coeffs, &distortion_coeffs_mat);
  EXPECT_EQ(distortion_coeffs_mat.cols, distortion_coeffs.size());
  EXPECT_EQ(distortion_coeffs_mat.rows, 1u);
  for (size_t i = 0u; i < distortion_coeffs.size(); i++)
  {
    EXPECT_EQ(distortion_coeffs_mat.at<double>(0, i), distortion_coeffs.at(i));
  }

  // 5 distortion params
  distortion_coeffs = { 1, 1.2f, 3u, 4l, 5.34 };  //! randomize types as well
  CameraParams::convertDistortionVectorToMatrix(distortion_coeffs, &distortion_coeffs_mat);
  EXPECT_EQ(distortion_coeffs_mat.cols, distortion_coeffs.size());
  EXPECT_EQ(distortion_coeffs_mat.rows, 1u);
  for (size_t i = 0u; i < distortion_coeffs.size(); i++)
  {
    EXPECT_EQ(distortion_coeffs_mat.at<double>(0u, i), distortion_coeffs.at(i));
  }

  // n distortion params
  distortion_coeffs = { 1.0, 1.2, 3.2, 4.3, 5.34, 10203, 1818.9, 1.9 };
  CameraParams::convertDistortionVectorToMatrix(distortion_coeffs, &distortion_coeffs_mat);
  EXPECT_EQ(distortion_coeffs_mat.cols, distortion_coeffs.size());
  EXPECT_EQ(distortion_coeffs_mat.rows, 1u);
  for (size_t i = 0u; i < distortion_coeffs.size(); i++)
  {
    EXPECT_EQ(distortion_coeffs_mat.at<double>(0u, i), distortion_coeffs.at(i));
  }
}

TEST(Camera, project)
{
  Landmarks lmks;
  lmks.push_back(Landmark(0.0, 0.0, 1.0));
  lmks.push_back(Landmark(0.0, 0.0, 2.0));
  lmks.push_back(Landmark(0.0, 1.0, 2.0));
  lmks.push_back(Landmark(0.0, 10.0, 20.0));
  lmks.push_back(Landmark(1.0, 0.0, 2.0));

  CameraParams::Intrinsics intrinsics(4);
  CameraParams::Distortion distortion(4);

  intrinsics.at(0) = 1.0;  // fx
  intrinsics.at(1) = 1.0;  // fy
  intrinsics.at(2) = 3.0;  // u0
  intrinsics.at(3) = 2.0;  // v0
  KeypointsCV expected_kpts;
  expected_kpts.push_back(KeypointCV(intrinsics.at(2), intrinsics.at(3), 0));
  expected_kpts.push_back(KeypointCV(intrinsics.at(2), intrinsics.at(3), 0));
  expected_kpts.push_back(KeypointCV(3.0, 1.0 / 2.0 + 2.0, 0));
  expected_kpts.push_back(KeypointCV(3.0, 1.0 / 2.0 + 2.0, 0));
  expected_kpts.push_back(KeypointCV(1.0 / 2.0 + 3.0, 2.0, 0));

  CameraParams camera_params(intrinsics, distortion, cv::Size(640, 480), "none", 0.05);

  auto camera = vdo::make_unique<Camera>(camera_params);

  KeypointsCV actual_kpts;
  EXPECT_NO_THROW(camera->project(lmks, &actual_kpts));
  compareKeypoints(expected_kpts, actual_kpts, 0.0001f);
}

TEST(Camera, backProjectSingleSimple)
{
  // Easy test first, back-project keypoint at the center of the image with
  // a given depth.
  CameraParams::Intrinsics intrinsics(4);
  CameraParams::Distortion distortion(4);
  intrinsics.at(0) = 721.5377;  // fx
  intrinsics.at(1) = 721.5377;  // fy
  intrinsics.at(2) = 609.5593;  // u0
  intrinsics.at(3) = 172.8540;  // v0
  CameraParams camera_params(intrinsics, distortion, cv::Size(640, 480), "none", 0.05);

  auto camera = vdo::make_unique<Camera>(camera_params);

  KeypointCV kpt(camera_params.cu(), camera_params.cv(), 0);
  Landmark actual_lmk;
  double depth = 2.0;
  camera->backProject(kpt, depth, &actual_lmk);

  Landmark expected_lmk(0.0, 0.0, depth);
  EXPECT_NEAR(expected_lmk.x(), actual_lmk.x(), 0.0001);
  EXPECT_NEAR(expected_lmk.y(), actual_lmk.y(), 0.0001);
  EXPECT_NEAR(expected_lmk.z(), actual_lmk.z(), 0.0001);
}

TEST(Camera, backProjectMultipleSimple)
{
  // Easy test first, back-project keypoints at the center of the image with
  // different depths.
  CameraParams::Intrinsics intrinsics(4);
  CameraParams::Distortion distortion(4);
  intrinsics.at(0) = 721.5377;  // fx
  intrinsics.at(1) = 721.5377;  // fy
  intrinsics.at(2) = 609.5593;  // u0
  intrinsics.at(3) = 172.8540;  // v0
  CameraParams camera_params(intrinsics, distortion, cv::Size(640, 480), "none", 0.05);

  auto camera = vdo::make_unique<Camera>(camera_params);

  KeypointCV kpt(camera_params.cu(), camera_params.cv(), 0);
  // Create 3 keypoints centered at image with different depths
  KeypointsCV kpts(3, kpt);
  std::vector<double> depths = { 2.0, 3.0, 4.5 };
  Landmarks actual_lmks;
  camera->backProject(kpts, depths, &actual_lmks);

  Landmarks expected_lmks;
  for (const auto& depth : depths)
  {
    expected_lmks.push_back(Landmark(0.0, 0.0, depth));
  }

  compareLandmarks(actual_lmks, expected_lmks, 0.0001);
}

TEST(Camera, backProjectSingleTopLeft)
{
  // Back-project keypoint at the center of the image with a given depth.
  CameraParams::Intrinsics intrinsics(4);
  CameraParams::Distortion distortion(4);

  double fx = 30.9 / 2.2;
  double fy = 12.0 / 23.0;
  double cu = 390.8;
  double cv = 142.2;

  intrinsics.at(0) = fx;  // fx
  intrinsics.at(1) = fy;  // fy
  intrinsics.at(2) = cu;  // u0
  intrinsics.at(3) = cv;  // v0
  CameraParams camera_params(intrinsics, distortion, cv::Size(640, 480), "none", 0.05);

  auto camera = vdo::make_unique<Camera>(camera_params);

  Landmark actual_lmk;
  double depth = 2.0;
  KeypointCV kpt(0.0, 0.0, 0);  // Top-left corner
  camera->backProject(kpt, depth, &actual_lmk);

  Landmark expected_lmk(depth / fx * (-cu), depth / fy * (-cv), depth);
  EXPECT_NEAR(expected_lmk.x(), actual_lmk.x(), 0.0001);
  EXPECT_NEAR(expected_lmk.y(), actual_lmk.y(), 0.0001);
  EXPECT_NEAR(expected_lmk.z(), actual_lmk.z(), 0.0001);
}
