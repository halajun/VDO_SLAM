#pragma once

#include "Macros.h"
#include "Types.h"
#include <vector>
#include <opencv2/opencv.hpp>

namespace vdo
{

class CameraParams {
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

  const double baseline; //assuming rgbd or stereo

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


class Camera {

public:
  VDO_POINTER_TYPEDEFS(Camera);

  Camera(const CameraParams& params_);

private:
  CameraParams params;

};
    
} // namespace vdo
