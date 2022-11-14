#include "Camera.h"

#include <glog/logging.h>

namespace vdo
{
CameraParams::CameraParams(const Intrinsics& intrinsics_, const Distortion& distortion_, const cv::Size& image_size_,
                           const std::string& distortion_model_, double baseline_)
  : intrinsics(intrinsics_)
  , distortion_coeff(distortion_)
  , image_size(image_size_)
  , distortion_model(CameraParams::stringToDistortion(distortion_model_, "pinhole"))
  , baseline(baseline_)
{
  CHECK_EQ(intrinsics.size(), 4u) << "Intrinsics must be of length 4 - [fx fy cu cv]";
  CHECK_GT(distortion_coeff.size(), 0u);

  CameraParams::convertDistortionVectorToMatrix(distortion_coeff, &D);
  CameraParams::convertIntrinsicsVectorToMatrix(intrinsics, &K);
}

void CameraParams::convertDistortionVectorToMatrix(const Distortion& distortion_coeffs, cv::Mat* distortion_coeffs_mat)
{
  *distortion_coeffs_mat = cv::Mat::zeros(1, distortion_coeffs.size(), CV_64F);
  for (int k = 0; k < distortion_coeffs_mat->cols; k++)
  {
    distortion_coeffs_mat->at<double>(0, k) = distortion_coeffs[k];
  }
}

void CameraParams::convertIntrinsicsVectorToMatrix(const Intrinsics& intrinsics, cv::Mat* camera_matrix)
{
  *camera_matrix = cv::Mat::eye(3, 3, CV_64F);
  camera_matrix->at<double>(0, 0) = intrinsics[0];
  camera_matrix->at<double>(1, 1) = intrinsics[1];
  camera_matrix->at<double>(0, 2) = intrinsics[2];
  camera_matrix->at<double>(1, 2) = intrinsics[3];
}

DistortionModel CameraParams::stringToDistortion(const std::string& distortion_model, const std::string& camera_model)
{
  std::string lower_case_distortion_model = distortion_model;
  std::string lower_case_camera_model = camera_model;

  std::transform(lower_case_distortion_model.begin(), lower_case_distortion_model.end(),
                 lower_case_distortion_model.begin(), ::tolower);
  std::transform(lower_case_camera_model.begin(), lower_case_camera_model.end(), lower_case_camera_model.begin(),
                 ::tolower);

  if (lower_case_camera_model == "pinhole")
  {
    if (lower_case_distortion_model == "none")
    {
      return DistortionModel::NONE;
    }
    else if ((lower_case_distortion_model == "plumb_bob") || (lower_case_distortion_model == "radial-tangential") ||
             (lower_case_distortion_model == "radtan"))
    {
      return DistortionModel::RADTAN;
    }
    else if (lower_case_distortion_model == "equidistant")
    {
      return DistortionModel::EQUIDISTANT;
    }
    else if (lower_case_distortion_model == "kannala_brandt")
    {
      return DistortionModel::FISH_EYE;
    }
    else
    {
      LOG(ERROR) << "Unrecognized distortion model for pinhole camera. Valid "
                    "pinhole distortion model options are 'none', 'radtan', "
                    "'equidistant', 'fish eye'.";
    }
  }
  else
  {
    LOG(ERROR) << "Unrecognized camera model. Valid camera models are 'pinhole'";
  }
}

const std::string CameraParams::toString() const
{
  std::stringstream out;
  out << "\n- cv " << cv() << "\nimage_size: \n- width: " << ImageWidth() << "\n- height: " << ImageHeight()
      << "\n- K: " << K << '\n'
      << "- Distortion Model: " << distortionToString(distortion_model) << '\n'
      << "- D: " << D << '\n'
      << "- P: " << P << '\n'
      << "- Baseline: " << baseline;

  return out.str();
}

Camera::Camera(const CameraParams& params_) : params(params_)
{
}

}  // namespace vdo