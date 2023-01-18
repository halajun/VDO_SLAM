#include "MotionSolver.h"
#include "utils/UtilsGtsam.h"
#include "utils/UtilsOpenCV.h"

#include <glog/logging.h>

namespace vdo
{
MotionSolver::Result MotionSolver::solvePnpOrMotion(const KeypointsCV& current_uv, const Landmarks& previous_3d_cam,
                                                    const TrackletIds& tracklet_ids, const gtsam::Pose3& previous_pose,
                                                    const gtsam::Pose3& motion_model, const Camera& camera,
                                                    const MotionSolver::Options& options)
{
  // check that current uv, previous 3d poins and tracklets are all the same size
  CHECK_EQ(current_uv.size(), previous_3d_cam.size()) << "2d-3d must be the same size! (" <<
                                                                current_uv.size() << " vs " <<
                                                                previous_3d_cam.size();
  CHECK_EQ(current_uv.size(), tracklet_ids.size()) << "correspondences and tracklet ID's must be the same size! (" <<
                                                                current_uv.size() << " vs " <<
                                                                tracklet_ids.size();

  MotionSolver::Result result;

  static constexpr size_t kMinPointsRansac = 5;  // required for ransac
  if (current_uv.size() < kMinPointsRansac)
  {
    LOG(WARNING) << "Not enoug points to run PnP RASAC with - needed " << kMinPointsRansac;
    result.success = false;
    return result;
  }

  std::vector<cv::Point2f> current_2d;
  std::vector<cv::Point3f> previous_3d;

  for (size_t i = 0; i < current_uv.size(); i++)
  {
    current_2d.push_back(current_uv.at(i).pt);

    Landmark lmk_c = previous_3d_cam.at(i);
    // solve in world frame
    if (options.solver == MotionSolver::Options::WORLD)
    {
      Landmark lmk_world = previous_pose.transformFrom(lmk_c);
      previous_3d.push_back(cv::Point3f(static_cast<float>(lmk_world.x()), static_cast<float>(lmk_world.y()),
                                        static_cast<float>(lmk_world.z())));
    }
    else
    {
      previous_3d.push_back(
          cv::Point3f(static_cast<float>(lmk_c.x()), static_cast<float>(lmk_c.y()), static_cast<float>(lmk_c.z())));
    }
  }


  VLOG(10) << "Solving pose with options\n" 
           << " - solver: " << (options.solver == MotionSolver::Options::Solver::CAMERA ? "Camera" : "World") << "\n"
           << " - reprojection error: " << options.reprojection_error << "\n"
           << " - pnp iterations: " << options.pnp_iterations << "\n"
           << " - pnp confidence: " << options.pnp_confidence << "\n"; 

  gtsam::Pose3 pnp_pose;
  TrackletIds pnp_inliers, pnp_outliers;
  bool pnp_result = estimatePosePnP(
    current_2d, 
    previous_3d, 
    tracklet_ids, 
    options, 
    camera, 
    previous_pose, 
    pnp_pose,
    pnp_inliers,
    pnp_outliers);

  gtsam::Pose3 motion_pose;
  TrackletIds motion_inliers, motion_outliers;
  double reprojection_error = 0, reprojection_error_inliers = 0;
  bool motion_result = estimatePoseMotion(
    current_2d,
    previous_3d,
    tracklet_ids,
    options,
    camera,
    previous_pose, 
    motion_model,
    motion_pose,
    motion_inliers,
    motion_outliers,
    reprojection_error,
    reprojection_error_inliers);

  if(!pnp_result) {
    TODO:
    LOG(ERROR) << "PnP result was false";
  }

  const size_t& num_pnp_inliers = pnp_inliers.size();
  const size_t& num_motion_inliers = motion_inliers.size();

  //select model based on inlier
  if(num_pnp_inliers >= num_motion_inliers) {
    result.inliers = pnp_inliers;
    result.outliers = pnp_outliers;
    result.pose = pnp_pose;
    result.options = options;
    result.model = Result::Selection::PNP;
    result.success = pnp_result;
  }
  else {
    result.inliers = motion_inliers;
    result.outliers = motion_outliers;
    result.pose = motion_pose;
    result.options = options;
    result.model = Result::Selection::MOTION_MODEL;
    result.success = motion_result;
  }

  result.motion_model_error = reprojection_error;
  result.motion_model_inlier_error = reprojection_error_inliers;
  return result;
}

bool MotionSolver::estimatePosePnP(const std::vector<cv::Point2f>& current_2d,
                                   const std::vector<cv::Point3f>& previous_3d, const TrackletIds& tracklet_ids,
                                   const Options& options, const Camera& camera, const gtsam::Pose3& previous_pose,
                                   gtsam::Pose3& pose, TrackletIds& inliers, TrackletIds& outliers)
{
  static constexpr size_t kMinPointsRansac = 5;  // required for ransac
  if (current_2d.size() < kMinPointsRansac)
  {
    LOG(WARNING) << "Not enoug points to run PnP RASAC with - needed " << kMinPointsRansac;
    return false;
  }

  const cv::Mat& K = camera.Params().K;
  const cv::Mat& D = camera.Params().D;

  // solve PnP
  cv::Mat Rvec(3, 1, CV_64FC1);
  cv::Mat Tvec(3, 1, CV_64FC1);
  cv::Mat Rot(3, 3, CV_64FC1);
  cv::Mat pnp_inliers;  // a [1 x N] vector
  cv::solvePnPRansac(previous_3d, current_2d, K, D, Rvec, Tvec, false, options.pnp_iterations,
                     options.reprojection_error, options.pnp_confidence, pnp_inliers, cv::SOLVEPNP_AP3P);

  // recover rotation
  cv::Rodrigues(Rvec, Rot);
  pose = utils::cvMatsToGtsamPose3(Rot, Tvec).inverse();

  if (options.solver == MotionSolver::Options::CAMERA)
  {
    pose = previous_pose * pose;  // compose to world frame
  }

  inliers.clear();
  outliers.clear();
  // recover inlier outlier id's from PNP
  for (size_t i = 0; i < pnp_inliers.rows; i++)
  {
    int inlier_index = pnp_inliers.at<int>(i);
    size_t inlier_tracklet = tracklet_ids[inlier_index];
    inliers.push_back(inlier_tracklet);
  }
  // calculate outliers
  tracking_tools::determineOutlierIds(inliers, tracklet_ids, outliers);
  CHECK_EQ((inliers.size() + outliers.size()), tracklet_ids.size());
  CHECK_EQ(inliers.size(), pnp_inliers.rows);
  VLOG(1) << "PNP Inliers/total = " << pnp_inliers.rows << "/" << previous_3d.size();

  return true;
}

bool MotionSolver::estimatePoseMotion(const std::vector<cv::Point2f>& current_2d,
                                   const std::vector<cv::Point3f>& previous_3d, const TrackletIds& tracklet_ids,
                                   const Options& options, const Camera& camera, const gtsam::Pose3& previous_pose,
                                   const gtsam::Pose3& motion_model, gtsam::Pose3& pose, TrackletIds& inliers,
                                   TrackletIds& outliers, double& reprojection_error,
                                   double& reprojection_error_inliers)
{

  //^wT_c_{T-1}
  pose = previous_pose * motion_model;

  //clear/reset outputs
  reprojection_error = 0.0;
  reprojection_error_inliers = 0.0;
  int successful_projections = 0;
  inliers.clear();
  outliers.clear();
  for (TrackletId i = 0; i < tracklet_ids.size(); i++)
  {
    const TrackletId tracklet_id = tracklet_ids[i];
    Landmark lmk_camera;     // the previous 3d point as projected into the current camera frame using the motion model
    KeypointCV kp_observed;  // the 2d observation of the projected kp
    kp_observed.pt = current_2d[i];

    //TODO:(jesse) going back and forth between types here
    cv::Point3f cv_lmk_previous = previous_3d[i];
    gtsam::Point3 lmk_previous(static_cast<double>(cv_lmk_previous.x), static_cast<double>(cv_lmk_previous.y),
                               static_cast<double>(cv_lmk_previous.z));

    if (options.solver == MotionSolver::Options::CAMERA)
    {
      // lmk_previous will be in camera frame of the previous camera pose
      // we need to translate it into the current camera pose
      // we use the motion model which should be the transform between C_{t-1} and C_t
      // lmk_camera = previous_frame->motion_model_.inverse() * lmk_previous;
      lmk_camera = motion_model.inverse() * lmk_previous;
    }
    else
    {
      // lmk_previous will be in the world frame abd we want it in the camera frame
      lmk_camera = pose.inverse() * lmk_previous;
    }

    // at this point lmk camera is a landmark in the reference frame of camera t and kp is the observation
    // in the image frame
    // calcualte reporojection error
    KeypointCV projected;
    bool is_contained = camera.isLandmarkContained(lmk_camera, projected);
    if (!is_contained)
    {
      continue;
    }
    else
    {
      successful_projections++;
    }

    const float u_err = kp_observed.pt.x - projected.pt.x;
    const float v_err = kp_observed.pt.y - projected.pt.y;
    const double err = static_cast<double>(std::sqrt(u_err * u_err + v_err * v_err));
    reprojection_error += err;

    if (err < options.reprojection_error)
    {
      inliers.push_back(tracklet_id);
      reprojection_error_inliers += err;
    }
  }

  // housekeeping
  reprojection_error /= tracklet_ids.size();
  reprojection_error_inliers /= inliers.size();

  LOG(INFO) << "Motion model estimation\n inliers - " << inliers.size() << "\ntotal repr error - "
            << reprojection_error << "\ninlier repr error - " << reprojection_error_inliers
            << "\nsuccessful projections - " << successful_projections << "/" << tracklet_ids.size();


  tracking_tools::determineOutlierIds(inliers, tracklet_ids, outliers);
  CHECK_EQ((inliers.size() + outliers.size()), tracklet_ids.size());
  LOG(INFO) << "Motion Inliers/total = " << inliers.size() << "/" << tracklet_ids.size();
  return true;
}

}  // namespace vdo