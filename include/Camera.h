#pragma once

#include "Macros.h"
#include "Types.h"
#include <vector>
#include <opencv2/opencv.hpp>

namespace vdo
{
class CameraParams
{
public:
  VDO_POINTER_TYPEDEFS(CameraParams);

  using Distortion = std::vector<double>;
  // fu, fv, cu, cv
  using Intrinsics = std::vector<double>;  // must be length 4!!
  using CameraModel = std::string;

  CameraParams(const Intrinsics& intrinsics_, const Distortion& distortion_, const cv::Size& image_size_,
               const std::string& distortion_model_, double baseline_);

  inline double fx() const
  {
    return intrinsics[0];
  }
  inline double fy() const
  {
    return intrinsics[1];
  }
  inline double cu() const
  {
    return intrinsics[2];
  }
  inline double cv() const
  {
    return intrinsics[3];
  }
  inline int ImageWidth() const
  {
    return image_size.width;
  }
  inline int ImageHeight() const
  {
    return image_size.height;
  }

  virtual ~CameraParams() = default;

  static void convertDistortionVectorToMatrix(const Distortion& distortion_coeffs, cv::Mat* distortion_coeffs_mat);

  static void convertIntrinsicsVectorToMatrix(const Intrinsics& intrinsics, cv::Mat* camera_matrix);

  /** Taken from: https://github.com/ethz-asl/image_undistort
   * @brief stringToDistortion
   * @param distortion_model
   * @param camera_model
   * @return actual distortion model enum class
   */
  static DistortionModel stringToDistortion(const std::string& distortion_model, const std::string& camera_model);

  // static void createGtsamCalibration(const cv::Mat& distortion,
  //                                 const Intrinsics& intrinsics,
  //                                 gtsam::Cal3DS2* calibration);

  const std::string toString() const;

public:
  // updates cv Mat P
  // for now only works if FISH_EYE
  //   void estimateNewMatrixForDistortion();

  //! fu, fv, cu, cv
  const Intrinsics intrinsics;
  const Distortion distortion_coeff;
  const cv::Size image_size;

  const double baseline;  // assuming rgbd or stereo

  //! Distortion parameters
  DistortionModel distortion_model;

  //! OpenCV structures: needed to compute the undistortion map.
  //! 3x3 camera matrix K (last row is {0,0,1})
  cv::Mat K;

  //! New camera matrix constructed from
  //! estimateNewCameraMatrixForUndistortRectify
  cv::Mat P;

  cv::Mat D;
};

class Camera
{
public:
  VDO_POINTER_TYPEDEFS(Camera);

  Camera(const CameraParams& params_);

  inline double Baseline() const
  {
    return params.baseline;
  }
  inline const CameraParams& Params() const
  {
    return params;
  }

  /**
   * @brief Projects a 3D point in the camera frame into the image frame as a
   * keypoint. Doesnt do any checks if the keypoint lands in the frame; use in
   * conjunction with isLandmarkContained
   *
   * @param lmks A 3D landmark
   * @param kpts The 2D keypoint to be set
   */
  void project(const Landmark& lmk, KeypointCV* kpt) const;

  /**
   * @brief Projects a list 3D points in the camera frame into the image frame.
   * Doesnt do any checks if the keypoint lands in the frame; use in conjunction
   * with isLandmarkContained
   *
   * @param lmks A list of 3D points
   * @param kpts A list of 2D keypoints to be set
   */
  void project(const Landmarks& lmks, KeypointsCV* kpts) const;

  /**
   * @brief Checks if a given keypoint is inside the image AND in front the
   * camera (ie. z > 0).
   *
   * Assume that the camera is orientated with z-axis pointing along the line of
   * sight of the camera, otherwise the z > 0 check is invalid.
   *
   * @param kpts Projected Keypoint
   * @param depth Depth of the 3D point (along the z axis)
   * @return true If the keypoint is visible from the camera frustrum
   * @return false
   */
  bool isKeypointContained(const KeypointCV& kpts, Depth depth) const;

  /**
   * @brief Back projects a list of keypoints from the image frame and into the
   * camera frame given a depth in z.
   *
   * Assume that the number of keypoints and number of depth values are the same
   * and are aligned by index.
   *
   * @param kps List of keypoints to back project
   * @param depths List of depth values to project along
   * @param lmks The 3D landmarks to set.
   */
  void backProject(const KeypointsCV& kps, const Depths& depths, Landmarks* lmks) const;

  /**
   * @brief Back projects a single keypoint from the image frame and into the
   * camera frame given a depth in z.
   *
   * @param kp Keypoint to back project
   * @param depth Depth value to project along
   * @param lmk  3D landmark to set.
   */
  void backProject(const KeypointCV& kp, const Depth& depth, Landmark* lmk) const;

  /**
   * @brief Checks if a landmark can be seen in the image frustrum. The 3D point
   * must be in the camera frame. The variable keypoint is also set to the
   * projected u,v coordinates. The keypoint will be set regardless of whether
   * the landmark is visible.
   *
   * @param lmk 3D landmark in the camera frame to check.
   * @param keypoint The projected of the landmark in the image frame.
   * @return true If the landmark is visible from the camera frame.
   * @return false
   */
  bool isLandmarkContained(const Landmark& lmk, KeypointCV& keypoint) const;

  /**
   * @brief Checks if a landmark can be seen in the image frustrum. The 3D point
   * must be in the camera frame.
   *
   * @param lmk 3D landmark in the camera frame to check.
   * @return true If the landmark is visible from the camera frame.
   * @return false
   */
  bool isLandmarkContained(const Landmark& lmk) const;

private:
  CameraParams params;
};

}  // namespace vdo
