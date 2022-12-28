#include "Optimizer.h"
#include "Frame.h"
#include "FrontendOutput.h"
#include "Frontend-Definitions.h"
#include "utils/Metrics.h"
#include "utils/UtilsOpenCV.h"
#include "utils/UtilsG2O.h"
#include "UtilsGtsam.h"
#include "Tracking-Tools.h"

#include <opencv2/core/eigen.hpp>
#include <glog/logging.h>

#include "dependencies/g2o/g2o/types/vertex_se3.h"
#include "dependencies/g2o/g2o/types/vertex_pointxyz.h"
#include "dependencies/g2o/g2o/types/edge_se3.h"
#include "dependencies/g2o/g2o/types/edge_se3_pointxyz.h"
#include "dependencies/g2o/g2o/types/edge_se3_prior.h"
#include "dependencies/g2o/g2o/core/block_solver.h"
#include "dependencies/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "dependencies/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "dependencies/g2o/g2o/solvers/linear_solver_csparse.h"
#include "dependencies/g2o/g2o/core/robust_kernel_impl.h"
#include "dependencies/g2o/g2o/solvers/linear_solver_eigen.h"
#include "dependencies/g2o/g2o/solvers/linear_solver_dense.h"

namespace vdo
{

PoseOptimizationFlow2Cam::PoseOptimizationFlow2Cam(const Camera& camera) : camera_(camera) {}


void PoseOptimizationFlow2Cam::operator()(Frame::Ptr previous_frame, Frame::Ptr current_frame) {
  float rp_thres = 0.04;  // 0.01
  bool updateflow = true;

  g2o::SparseOptimizer optimizer;
  // optimizer.setVerbose(true);
  g2o::BlockSolver_6_3::LinearSolverType* linearSolver;

  linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

  g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

  optimizer.setAlgorithm(solver);

  int nInitialCorrespondences = 0;

  // Set MapPoint vertices
  // const int N = TemperalMatch.size();

  // Set Frame vertex
  CHECK(previous_frame);
  CHECK(current_frame);
  LOG(INFO) << "starting PoseOptimizationFlow2Cam...";
  g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
  //NOTE: we operate on the inverse of the camera pose (which is Twc) -> this is a left over of how the original code was written
  cv::Mat Tcw_current = utils::gtsamPose3ToCvMat(current_frame->pose_.inverse());  // initial with camera pose
  cv::Mat Tcw_previous = utils::gtsamPose3ToCvMat(previous_frame->pose_.inverse());  // initial with camera pose

  vSE3->setEstimate(utils::toSE3Quat(Tcw_current));
  vSE3->setId(0);
  vSE3->setFixed(false);
  optimizer.addVertex(vSE3);

  TrackletIds tracklet_ids;  // feature tracklets from the current frame so we can assign values as inliers
  for(Feature::Ptr current_feature : current_frame->features_) {
    CHECK(current_feature);
    const std::size_t& tracklet_id = current_feature->tracklet_id;

    Feature::Ptr previous_feature = previous_frame->getByTrackletId(tracklet_id);

    //the only tracklets we should include are INLIERS and if we have a previous track (ie age > 1)
    if (current_feature->inlier && previous_feature) {
      tracklet_ids.push_back(tracklet_id);
    }
  }

  // Set Edge info
  std::vector<g2o::EdgeSE3ProjectFlow2*> vpEdgesMono;
  std::vector<size_t> vnIndexEdgeMono;
  const size_t N = tracklet_ids.size();
  vpEdgesMono.reserve(N);
  vnIndexEdgeMono.reserve(N);

  // // parameter for robust function
  const float deltaMono = sqrt(rp_thres);  // 5.991

  // bool mono = 1;  // monocular
  float repro_e = 0;

  for (int i = 0; i < N; i++)
  {

    const TrackletId tracklet_id = tracklet_ids[i];
    Feature::Ptr previous_feature = previous_frame->getByTrackletId(tracklet_id);
    CHECK(previous_feature);

    CHECK(previous_feature->inlier);
   
    nInitialCorrespondences++;

    // Set Flow vertices
    g2o::VertexSBAFlow* vFlo = new g2o::VertexSBAFlow();
    // Eigen::Matrix<double, 3, 1> flow_d = Converter::toVector3d(pLastFrame->ObtainFlowDepthCamera(TemperalMatch[i], 0));
    gtsam::Vector3 flow_d = obtainFlowDepth(*previous_frame, tracklet_id);
    vFlo->setEstimate(flow_d.head(2));
    const int id = i + 1;
    vFlo->setId(id);
    vFlo->setMarginalized(true);
    optimizer.addVertex(vFlo);

    Eigen::Matrix<double, 2, 1> obs_2d;
    const cv::KeyPoint& previous_kp = previous_feature->keypoint;
    // const cv::KeyPoint& kpUn = pLastFrame->mvStatKeys[TemperalMatch[i]];
    obs_2d << previous_kp.pt.x, previous_kp.pt.y;

    // Set Binary Edges
    g2o::EdgeSE3ProjectFlow2* e = new g2o::EdgeSE3ProjectFlow2();

    // vertex ID -> VFlow estimate (type  g2o::VertexSBAFlow())
    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
    // vertex ID = 0 -> the inital pose
    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
    e->setMeasurement(obs_2d);
    Eigen::Matrix2d info_flow;

    // information matrix
    info_flow << 0.1, 0.0, 0.0, 0.1;
    e->setInformation(Eigen::Matrix2d::Identity() * info_flow);

    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    e->setRobustKernel(rk);
    rk->setDelta(deltaMono);

    const CameraParams& cam_params = camera_.Params();

    e->fx = cam_params.fx();
    e->fy = cam_params.fy();
    e->cx = cam_params.cu();
    e->cy = cam_params.cv();

    e->depth = flow_d(2);

    const cv::Mat Rlw = Tcw_previous.rowRange(0, 3).colRange(0, 3);
    const cv::Mat Rwl = Rlw.t();
    const cv::Mat tlw = Tcw_previous.rowRange(0, 3).col(3);
    const cv::Mat twl = -Rlw.t() * tlw;
    e->Twl.setIdentity(4, 4);

    Eigen::Matrix<double, 3, 3> Rwl_matrix;
    Eigen::Matrix<double, 3, 1> twl_vector;
    cv::cv2eigen(Rwl, Rwl_matrix);
    cv::cv2eigen(twl, twl_vector);

    // e->Twl = Twc_matrix;

    e->Twl.block(0, 0, 3, 3) = Rwl_matrix;
    e->Twl.col(3).head(3) = twl_vector;

    optimizer.addEdge(e);

    vpEdgesMono.push_back(e);
    vnIndexEdgeMono.push_back(i);

    Eigen::Matrix<double, 2, 1> obs_flo;
    // flow in x, y
    obs_flo << flow_d(0), flow_d(1);

    // Set Unary Edges (constraints)
    g2o::EdgeFlowPrior* e_con = new g2o::EdgeFlowPrior();
    e_con->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
    e_con->setMeasurement(obs_flo);
    Eigen::Matrix2d invSigma2_flo;
    invSigma2_flo << 0.3, 0.0, 0.0, 0.3;
    e_con->setInformation(Eigen::Matrix2d::Identity() * invSigma2_flo);
    optimizer.addEdge(e_con);
  }

  if (nInitialCorrespondences < 3) {
    LOG(WARNING) << "init correspondences < 3";
    return;

  }

  // // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
  // // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
  const float chi2Mono[4] = { rp_thres,rp_thres, rp_thres, rp_thres };  // {5.991,5.991,5.991,5.991} {4,4,4,4}
  const int its[4] = { 100, 100, 100, 100 };

  int nBad = 0;
  for (size_t it = 0; it < 4; it++)
  {
    vSE3->setEstimate(utils::toSE3Quat(Tcw_current));
    optimizer.initializeOptimization(0);
    optimizer.optimize(its[it]);

    nBad = 0;

    // monocular
    for (size_t i = 0; i < vpEdgesMono.size(); i++)
    {
      g2o::EdgeSE3ProjectFlow2* e = vpEdgesMono[i];

      const size_t idx = vnIndexEdgeMono[i];
      const TrackletId tracklet_id = tracklet_ids[idx];
      Feature::Ptr feature = previous_frame->getByTrackletId(tracklet_id);
      Feature::Ptr current_feature = current_frame->getByTrackletId(tracklet_id);

      //at the start of this optimzation all edges in the graph are constructed from INLIER
      //observations. So if something is now no longer an inlier it was marked as an outlier during the
      //optimziation process
      // if (!feature->inlier)
      // {
        e->computeError();
      // }

      const float chi2 = e->chi2();

      if (chi2 > chi2Mono[it])
      {
        feature->inlier = false;
        current_feature->inlier = false;
        e->setLevel(1);
        nBad++;
      }
      else
      {
        // ++++ new added for calculating re-projection error +++
        //this only calculates for each one -> we should show for each iteration!!
        if (it == 0)
        {
          repro_e = repro_e + std::sqrt(chi2);
        }
        feature->inlier = true;
        current_feature->inlier = true;
        e->setLevel(0);
      }

      if (it == 2)
        e->setRobustKernel(0);
    }

    if (optimizer.edges().size() < 5)
      break;
  }
  std::cout << std::endl;
  // // *** Recover optimized pose and return number of inliers ***
  g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
  g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
  gtsam::Pose3 Xcw = utils::toGtsamPose3(SE3quat_recov);

  // LOG(INFO) << "pose before estimate - \n" <<  current_frame->pose_;
  current_frame->pose_ = Xcw.inverse();
  // LOG(INFO) << "pose after estimate - \n" <<  current_frame->pose_;

  int updated_n_flows = 0;
  // *** Recover optimized optical flow ***
  for (int i = 0; i < N; ++i)
  {
    const TrackletId tracklet_id = tracklet_ids[i];
    g2o::VertexSBAFlow* vFlow = static_cast<g2o::VertexSBAFlow*>(optimizer.vertex(i + 1));
    Feature::Ptr previous_feature = previous_frame->getByTrackletId(tracklet_id);
    CHECK(previous_feature);



    if (updateflow && previous_feature->inlier)
    {
      Feature::Ptr current_feature = current_frame->getByTrackletId(tracklet_id);

      CHECK(current_feature);

      
      Eigen::Vector2d flow_new = vFlow->estimate();
      cv::Point2d optimized_flow(flow_new(0), flow_new(1));
      previous_feature->optical_flow = optimized_flow;

      cv::KeyPoint predicted_kp = previous_feature->keypoint;
      predicted_kp.pt.x += static_cast<double>(flow_new(0));
      predicted_kp.pt.y += static_cast<double>(flow_new(1));
      previous_feature->predicted_keypoint = predicted_kp;

      current_feature->keypoint = previous_feature->predicted_keypoint; //update the actual KP

      updated_n_flows++;
    }
  }
  int inliers = nInitialCorrespondences - nBad;
  LOG(INFO) << "(Camera) inliers number/total numbers: " << inliers << "/" << nInitialCorrespondences;
  repro_e = repro_e / inliers;
  LOG(INFO) << "re-projection error from the optimization: " << repro_e;
  LOG(INFO) << "Updated flows " << updated_n_flows;

  // return nInitialCorrespondences - nBad;
}

IncrementalOptimizer::IncrementalOptimizer(const BackendParams& params, const Camera& camera)
  : FactorGraphManager(params), camera_(camera)
{
  params_.print();
}

IncrementalOptimizer::~IncrementalOptimizer() {
  //do some logging at the end
  int num_more_than_five_tracket = 0;
  //TODO: check if still inliers?
  for(const auto& tracklet_id_feature : all_tracklets_) {

    if(tracklet_id_feature.second.size() > 5) {
      num_more_than_five_tracket++;
    }
  }

  LOG(INFO) << "Num tracklets of length > 5 - " << num_more_than_five_tracket;
}

BackendOutput::Ptr IncrementalOptimizer::process(const FrontendOutput& input)
{
  BackendOutput::Ptr output = nullptr;
  if (state == State::kBoostrap)
  {
    output = processBoostrap(input);
  }
  else if (state == State::kNominal)
  {
    output = processNominal(input);
  }

  return output;
}

BackendOutput::Ptr IncrementalOptimizer::processBoostrap(const FrontendOutput& input)
{
  LOG(INFO) << input.frame_id_;
  gtsam::Key state_key = input.frame_id_;
  LOG(INFO) << state_key;
  const gtsam::Pose3 estimated_pose = input.estimated_pose_;
  const GroundTruthInputPacket::ConstOptional ground_truth = input.ground_truth_;

  const Frame::Ptr& frame = input.frame_;

  // first pose so we add pose prior
  addCameraPose(state_key, estimated_pose);
  addCameraPosePrior(state_key, estimated_pose);

  handleStaticFeatures(frame->features_);
  // odometry

  optimize(state_key);

  state = State::kNominal;
  return nullptr;
}

BackendOutput::Ptr IncrementalOptimizer::processNominal(const FrontendOutput& input)
{
  LOG(INFO) << input.frame_id_;
  gtsam::Key state_key = input.frame_id_;
  LOG(INFO) << state_key;
  CHECK_GE(state_key, 0u);
  const gtsam::Pose3 estimated_pose = input.estimated_pose_;
  const GroundTruthInputPacket::ConstOptional ground_truth = input.ground_truth_;
  const Frame::Ptr& frame = input.frame_;

  double t_error_before_opt, r_error_before_opt, t_error_after_opt, r_error_after_opt;
  calculatePoseError(estimated_pose, ground_truth->X_wc, t_error_before_opt, r_error_before_opt);

  // first pose so we add pose prior
  gtsam::Key pose_key = addCameraPose(state_key, estimated_pose);
  handleStaticFeatures(frame->features_);

  // // TODO:for now!
  // // get prior camera motion
  gtsam::Key prev_pose_key = poseKey(state_key - 1);
  gtsam::Pose3 previous_pose = state_.at<gtsam::Pose3>(prev_pose_key);
  gtsam::Pose3 odometry = previous_pose.inverse() * estimated_pose;

  addBetweenFactor(state_key - 1, state_key, odometry);

  optimize(state_key);

  LOG(INFO) << "Num features added " << num_static_features_added;
  // get updated pose
  gtsam::Pose3 best_pose = state_.at<gtsam::Pose3>(pose_key);

  LOG(INFO) << "Opt pose\n" << best_pose;

  //this shows a decrease in error always!!
  calculatePoseError(best_pose, ground_truth->X_wc, t_error_after_opt, r_error_after_opt);
  LOG(INFO) << std::fixed << "ATE Errors:\n"
            << "Error before: t - " << t_error_before_opt << ", r - " << r_error_before_opt << "\n"
            << "Error after: t - " << t_error_after_opt << ", r - " << r_error_after_opt << "\n";

  num_static_features_added = 0;
  BackendOutput::Ptr output = std::make_shared<BackendOutput>();
  output->estimated_pose_ = best_pose;
  return output;
}

void IncrementalOptimizer::handleStaticFeatures(const FeaturePtrs& static_features)
{
  // this should handle all new features currently in the map
  for (Feature::Ptr feature_ptr : static_features)
  {
    const Feature& feature = *feature_ptr;
    const size_t tracklet_id = feature.tracklet_id;
    const size_t frame_id = feature.frame_id;
    if (feature.inlier)
    {


      gtsam::Key pose_key = poseKey(frame_id);
      // LOG(INFO) << pose_key;
      gtsam::Symbol symb(pose_key);

      // LOG(INFO) << "pose symbol '" << symb.chr() << "' "
      //           << "and index " << symb.index();

      // if the landmark is already in the map add it directly
      gtsam::Key potential_landmark_key = staticLandmarkKey(tracklet_id);
      // gtsam::Symbol plk(potential_landmark_key);
      // LOG(INFO) << "pose symbol '" << plk.chr() << "' "
      //           << "and index " << plk.index();
      if (isKeyInGraph(potential_landmark_key))
      {
        Landmark lmk_camera;
        camera_.backProject(feature.keypoint, feature.depth, &lmk_camera);
        gtsam::Key lmk_key = addLandmarkToGraph(tracklet_id, frame_id, lmk_camera);
        // sanity check
        CHECK_EQ(potential_landmark_key, lmk_key);
      }
      else
      {
        // add it to the feature map
        collectFeature(feature);
      }

      //we also just all to the static map
      // if not already in tracklet map, create new vector and add
      if (all_tracklets_.find(tracklet_id) != all_tracklets_.end())
      {
        all_tracklets_.at(tracklet_id).push_back(feature_ptr);
      }
      else
      {
        all_tracklets_.insert({ tracklet_id, { feature_ptr } });
      }
    }

  }

  // LOG(INFO) <<
  // go through tracklets and check length (these should correspond to a landmarks age)
  static constexpr size_t kMinTrackletLength = 2u;
  std::vector<size_t> tracklet_ids_to_delete;  // which tracklets have now been added to the map
  for (const auto& [tracklet_id, features] : unadded_static_tracklets_)
  {
    gtsam::Key potential_landmark_key = staticLandmarkKey(tracklet_id);
    gtsam::Symbol plk(potential_landmark_key);
    CHECK(!isKeyInGraph(potential_landmark_key))
        << "key " << plk.chr() << " index " << plk.index() << " should not be in the graph yet!!";

    if (features.size() >= kMinTrackletLength)
    {
      // go through all the features and add them
      for (const auto& feature : features)
      {
        const size_t frame_id = feature.frame_id;
        // sanity check
        CHECK_EQ(tracklet_id, feature.tracklet_id);
        Landmark lmk_camera;
        camera_.backProject(feature.keypoint, feature.depth, &lmk_camera);
        gtsam::Symbol lmk_key = addLandmarkToGraph(tracklet_id, frame_id, lmk_camera);
        // sanity check
        CHECK_EQ(potential_landmark_key, lmk_key);

        // mark this tracklet as added
        tracklet_ids_to_delete.push_back(tracklet_id);
      }
    }
  }

  // remove landmakrs from the map as they should all now appear in the graph
  for (size_t ids_to_remove : tracklet_ids_to_delete)
  {
    gtsam::Key lmk_key = staticLandmarkKey(ids_to_remove);
    // sanity check
    CHECK(isKeyInGraph(lmk_key));
    unadded_static_tracklets_.erase(ids_to_remove);
  }
}

// landmarks in camera frame
void IncrementalOptimizer::collectFeature(const Feature& feature)
{
  CHECK(feature.inlier);
  const size_t& tracklet_id = feature.tracklet_id;
  // if not already in tracklet map, create new vector and add
  if (unadded_static_tracklets_.find(tracklet_id) != unadded_static_tracklets_.end())
  {
    unadded_static_tracklets_.at(tracklet_id).push_back(feature);
  }
  else
  {
    unadded_static_tracklets_.insert({ tracklet_id, { feature } });
  }
}

gtsam::Key IncrementalOptimizer::addLandmarkToGraph(const size_t tracklet_id, const size_t frame_id,
                                                    const Landmark& lmk_camera)
{
  gtsam::Key pose_key = poseKey(frame_id);
  gtsam::Symbol lmk_key = addStaticLandmark(tracklet_id, pose_key, lmk_camera);
  static_feature_slots_.insert({ lmk_key, std::make_pair(tracklet_id, frame_id) });

  num_static_features_added++;

  return lmk_key;
}

}  // namespace vdo