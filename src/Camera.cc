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

void Camera::project(const Landmarks& lmks, KeypointsCV* kpts) const
{
  if (kpts == nullptr)
  {
    LOG(WARNING) << "Kpts vector is null";
    return;
  }
  kpts->clear();
  const auto& n_lmks = lmks.size();
  kpts->resize(n_lmks);
  // Can be greatly optimized with matrix mult or vectorization
  for (size_t i = 0u; i < n_lmks; i++)
  {
    project(lmks[i], &(*kpts)[i]);
  }
}

void Camera::project(const Landmark& lmk, KeypointCV* kpt) const
{
  const double x = lmk.x();
  const double y = lmk.y();
  const double z = lmk.z();

  // lmks should be in camera frame P_c in R3
  float u = static_cast<float>((x * params.fx()) / z + params.cu());
  float v = static_cast<float>((y * params.fy()) / z + params.cv());

  kpt->pt.x = u;
  kpt->pt.y = v;
}

bool Camera::isKeypointContained(const KeypointCV& kpt, Depth depth) const
{
  return (kpt.pt.x > 0 && kpt.pt.x < params.ImageWidth() && kpt.pt.y > 0 && kpt.pt.y < params.ImageHeight() &&
          depth > 0);
}

void Camera::backProject(const KeypointsCV& kps, const Depths& depths, Landmarks* lmks) const
{
  if (lmks == nullptr)
  {
    LOG(WARNING) << "landmark vector is null";
    return;
  }
  lmks->reserve(kps.size());
  for (size_t i = 0u; i < kps.size(); i++)
  {
    Landmark lmk;
    backProject(kps[i], depths[i], &lmk);
    lmks->push_back(lmk);
  }
}

// lmk should be in camera frame
void Camera::backProject(const KeypointCV& kp, const Depth& depth, Landmark* lmk) const
{
  const double u = static_cast<double>(kp.pt.x);
  const double v = static_cast<double>(kp.pt.y);
  const double z = depth;

  // const double x = (u- cam_params.intrinsics.cu) * z
  // * 1.0/cam_params.intrinsics.fx; const double y =
  // (v-cam_params.intrinsics.cv) * z * 1.0/cam_params.intrinsics.fy;
  const float x = static_cast<float>((u - params.cu()) * z * 1.0 / params.fx());
  const float y = static_cast<float>((v - params.cv()) * z * 1.0 / params.fy());

  (*lmk)(0) = x;
  (*lmk)(1) = y;
  (*lmk)(2) = z;
}

bool Camera::isLandmarkContained(const Landmark& lmk) const
{
  KeypointCV keyPoint;
  project(lmk, &keyPoint);
  return isKeypointContained(keyPoint, lmk(2));
}

bool Camera::isLandmarkContained(const Landmark& lmk, KeypointCV& keypoint) const
{
  project(lmk, &keypoint);
  return isKeypointContained(keypoint, lmk(2));
}

}  // namespace vdo