#include "backend/VdoSlamBackend.h"
#include "utils/UtilsGTSAM.h"
#include "utils/UtilsOpenCv.h"

#include <glog/logging.h>
#include <fstream>
#include <memory>

#include <boost/foreach.hpp>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/Pose3.h>

#include "utils/macros.h"
#include "utils/timing.h"
#include "factors/Point3DFactor.h"
#include "factors/LandmarkMotionTernaryFactor.h"
#include "factors/AltitudeFactor.h"
#include "utils/dataset.h"
#include "Map.h"
#include "Optimizer.h"
#include "utils/timing.h"
// #include "Converter.h"
#include <chrono>
using namespace std::chrono;

#include "visualizer/PltPlotter.h"
#include <matplotlibcpp.h>

namespace plt = matplotlibcpp;



namespace VDO_SLAM {

VdoSlamBackend::VdoSlamBackend(Map* map_, const cv::Mat& Calib_K_, BackendParams::Ptr params_) 
    :   map(CHECK_NOTNULL(map_)),
        K(Calib_K_),
        params(CHECK_NOTNULL(params_)),
        count_unique_id(1),
        pre_camera_pose_vertex(-1),
        curr_camera_pose_vertex(0),
        current_frame(-1),
        key_to_unique_vertices(),
        cameraPosePrior(nullptr),
        odometryNoiseModel(nullptr),
        point3DNoiseModel(nullptr),
        objectMotionSmootherNoiseModel(nullptr),
        dynamicPoint3DNoiseModel(nullptr),
        objectMotionNoiseModel(nullptr) {

        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.evaluateNonlinearError = false;
        parameters.relinearizeSkip = 1;

        // isam = VDO_SLAM::make_unique<gtsam::IncrementalFixedLagSmoother>(5);
        isam = VDO_SLAM::make_unique<gtsam::ISAM2>(parameters);
        K_calib =  utils::cvMat2Cal3_S2(K);

        //parsed in as float but we want it as a double... cuz gtsam
        //will this mess up all our other data types?
        cv::Mat K_d;
        K.convertTo(K_d, CV_64F);
        camera = std::make_shared<Camera>(K_d);
        camera->print();

        setupNoiseModels();        

        // //TODO: for 3d Points only. See optimzier.
        // robust_noise_model =  gtsam::noiseModel::Robust::Create(
        //     gtsam::noiseModel::mEstimator::Huber::Create(0.0001), point3DNoiseModel);  // robust

        CHECK_NOTNULL(cameraPosePrior);
        CHECK_NOTNULL(odometryNoiseModel);
        CHECK_NOTNULL(point3DNoiseModel);
        CHECK_NOTNULL(objectMotionSmootherNoiseModel);
        CHECK_NOTNULL(dynamicPoint3DNoiseModel);
        CHECK_NOTNULL(objectMotionNoiseModel);


        cameraPosePrior->print("Camera Pose Prior ");
        odometryNoiseModel->print("Odometry Noise Model ");
        point3DNoiseModel->print("3D Point Noise Model ");
        objectMotionSmootherNoiseModel->print("Object motion smoother ");
        dynamicPoint3DNoiseModel->print("Dynamic 3d point ");
        objectMotionNoiseModel->print("Object Motion model: ");
        CHECK_NOTNULL(K_calib);

        //init plots
        Plotter::initPlot((PlotInfo){
            .title = "Error before optimization",
            .x_label = "# Frames",
            .y_label = "Chi-squared error"});

        Plotter::initPlot((PlotInfo){
            .title = "Error after optimization",
            .x_label = "# Frames",
            .y_label = "Chi-squared error"});

        Plotter::initPlot((PlotInfo){
            .title = "Time for each update step of iSAM2",
            .x_label = "# Frames",
            .y_label = "Time (s)"});

        Plotter::initPlot((PlotInfo){
            .title = "Number of factors in the iSAM2 factor graph",
            .x_label = "# Frames",
            .y_label = "-"});

        Plotter::initPlot((PlotInfo){
            .title = "Number of variables in the system",
            .x_label = "# Frames",
            .y_label = "-"});

        Plotter::initPlot((PlotInfo){
            .title = "# of Cliques",
            .x_label = "# Frames",
            .y_label = "-"});

        Plotter::initPlot((PlotInfo){
            .title = "Max Clique Size",
            .x_label = "# Frames",
            .y_label = "-"});

        // do_manager = VDO_SLAM::make_unique<DynamicObjectManager>(map_);
    }

void VdoSlamBackend::process(bool run_as_incremental) {
    const int N = map->vpFeatSta.size(); // Number of Frames
    LOG(INFO) << "Running incremental update on frame " << N;
    current_frame = N - 1;

    std::vector<std::vector<std::pair<int, int> > > StaTracks = map->TrackletSta;
    std::vector<std::vector<std::pair<int, int> > > DynTracks = map->TrackletDyn;

    std::vector<int>  vnFLS_tmp(map->vpFeatSta[current_frame].size(),-1);
    vnFeaLabSta.push_back(vnFLS_tmp);
    vnFeaMakSta.push_back(vnFLS_tmp);

    std::vector<int> vnFLD_tmp(map->vpFeatDyn[current_frame].size(),-1);
    vnFeaLabDyn.push_back(vnFLD_tmp);
    vnFeaMakDyn.push_back(vnFLD_tmp);


    CHECK_EQ(vnFeaLabSta.size(), N);
    CHECK_EQ(vnFeaMakSta.size(), N);
    CHECK_EQ(vnFeaLabDyn.size(), N);
    CHECK_EQ(vnFeaMakDyn.size(), N);

    if(current_frame == 0) {
        CHECK_EQ(N, 1);
        std::vector<gtsam::Key> v_id_tmp(1,-1);
        unique_vertices.push_back(v_id_tmp);
        
        std::vector<int> obj_id_tmp(1, -1);
        objUniqueId.push_back(obj_id_tmp);
    }
    else {
        //careful about casting -> vnRMLabel is int and we're casting to uint32...?
        std::vector<gtsam::Key> v_id_tmp(map->vnRMLabel[current_frame-1].size(),-1);
        unique_vertices.push_back(v_id_tmp);

        std::vector<int> obj_id_tmp(map->vmRigidMotion[current_frame-1].size()-1,-1);
        objUniqueId.push_back(obj_id_tmp);
    }

    CHECK_EQ(unique_vertices.size(), N);
    CHECK_EQ(objUniqueId.size(), N);


    static_tracklets.update(StaTracks);
    dynamic_tracklets.update(DynTracks);

    LOG(INFO) << "vnRMLabel size " << map->vnRMLabel.size();
    CHECK_EQ(map->vnRMLabel.size(), map->vmRigidMotion.size());
    for(size_t i = 0; i < map->vnRMLabel.size(); i++) {
        //number of RM labels should be the same as the number of rigid motions vmRigidMotion
        CHECK_EQ(map->vnRMLabel[i].size(), map->vmRigidMotion[i].size());
        // LOG(INFO) << "N labels: " << i << " " <<  map->vnRMLabel[i].size();
    }

    //can get from point to label from vnFeatLabel[i][j]

    // CHECK_EQ(map->nObjID.size(), dynamic_tracklets.size());
    // std::ofstream file_ti("dynamic_track_info.txt", std::ios_base::app);
    // file_ti << "Frame N: " << current_frame << std::endl;
    // for(size_t i = 0; i < dynamic_tracklets.size(); i++) {
    //     DynamicTrackletManager::TypedTracklet& tracklet = dynamic_tracklets[i];
    //     file_ti << "Tracklet ID: " << tracklet.TrackletId() << " is WT " << tracklet.isWellTracked() << "OBJ ID " << map->nObjID[tracklet.TrackletId()] << std::endl;

    //     for(size_t j = 0; j < tracklet.size(); j++) {
    //         DynamicTrackletManager::Observation obs = dynamic_tracklets[i][j];
    //         file_ti << obs->to_string();
    //     }
    // }
    // file_ti.close();
    gtsam::Pose3 camera_pose = utils::cvMatToGtsamPose3(map->vmCameraPose[current_frame]);
    cv::Mat camera_pose_cv = map->vmCameraPose[current_frame];
    // camera_pose = camera_pose.compose(gtsam::Pose3::identity());
    addCameraPoseToGraph(camera_pose, (count_unique_id), current_frame);
    LOG(INFO) << "Added camera pose at " << (count_unique_id);
   
    //is first (or current_frame == 0?)
    if(current_frame == 0) {
        // graph.addPrior(count_unique_id, camera_pose, cameraPosePrior);
        graph.push_back(gtsam::PriorFactor<gtsam::Pose3>((count_unique_id), camera_pose, cameraPosePrior));
        LOG(INFO) << "Added camera prior";
    }

    unique_vertices[current_frame][0] = count_unique_id;
    
    curr_camera_pose_vertex = count_unique_id;
    count_unique_id++;
    
    if(current_frame > 0) {
        CHECK(pre_camera_pose_vertex != -1);
        gtsam::Pose3 rigid_motion = utils::cvMatToGtsamPose3(map->vmRigidMotion[current_frame-1][0]);
        graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            (pre_camera_pose_vertex),
            (curr_camera_pose_vertex),
            rigid_motion,
            odometryNoiseModel
        );
        LOG(INFO) << "Added motion constraint factor between id: " << (pre_camera_pose_vertex) << " " << (curr_camera_pose_vertex);
    }

    static int n_landmark_add = 0;
    static int n_factor_add = 0;
    timing::Timer static_points_timer("backend/static_points");
    //start by adding the static points from the new frame
    for(size_t point_id = 0; point_id < map->vpFeatSta[current_frame].size(); point_id++) {
        if(static_tracklets.exists(current_frame, point_id)) {
            StaticTrackletManager::TypedTracklet tracklet = static_tracklets.getTracklet(current_frame, point_id);

            if(tracklet.isWellTracked()) {
                StaticTrackletManager::Observations obs_to_add = tracklet.getNotAdded();
                // LOG(INFO) << tracklet.TrackletId() << " is well tracked " << obs_to_add.size() << " and is new: "<< tracklet.isNew();

                for (StaticTrackletManager::Observation obs : obs_to_add) {
                    FrameId frame_id = obs->frame_id;
                    FeatureId feature_id = obs->point_id;
                    int track_id = obs->tracklet_id;
                    int position_id = obs->tracklet_position;
                    obs->was_added = true;

                    gtsam::Key pose_key = unique_vertices[frame_id][0];

                    if (position_id == 0) {
                        gtsam::Point3 X_w = utils::cvMatToGtsamPoint3(map->vp3DPointSta[frame_id][feature_id]);
                        obs->key = static_cast<int>(count_unique_id);
                        // obs->was_added = true;
                        // LOG(INFO) << obs->key;
                        addLandmarkToGraph(X_w, (count_unique_id), frame_id, feature_id);
                        n_landmark_add++;

                        cv::Mat Xc = Optimizer::Get3DinCamera(map->vpFeatSta[frame_id][feature_id],map->vfDepSta[frame_id][feature_id],K);
                        gtsam::Point3 X_c_point = utils::cvMatToGtsamPoint3(Xc);
                        
                        // addPoint3DFactor(X_c_point, curr_camera_pose_vertex, count_unique_id);
                        addPoint3DFactor(X_c_point, (pose_key), (count_unique_id));
                        n_factor_add++;

                        // update unique id
                        //better to use track id as Symbol but then will need to make all variables use differnt symbols
                        vnFeaMakSta[frame_id][feature_id] = count_unique_id;
                        count_unique_id++;
                    }
                    else {

                        StaticTrackletManager::Observation first_obs = tracklet[0];

                        // StaticTrackletManager::Observation prev_obs = tracklet.getPreviousObservation(obs);
                        auto tracklet_key = first_obs->key;
                        // obs->key = prev_obs->key;
                        // obs->was_added = true;
                        //lets do a sanity check
                        //previous obs key has been set
                        CHECK(first_obs->tracklet_id == track_id);
                        CHECK(first_obs->was_added);
                        CHECK(first_obs->tracklet_position == 0) << " was " << first_obs->tracklet_position;

                        CHECK(tracklet_key != -1) << "Previous key was " << tracklet_key;
                        cv::Mat Xc = Optimizer::Get3DinCamera(map->vpFeatSta[frame_id][feature_id],map->vfDepSta[frame_id][feature_id],K);
                        gtsam::Point3 X_c_point = utils::cvMatToGtsamPoint3(Xc);
                        addPoint3DFactor(X_c_point, (pose_key), (tracklet_key));
                        n_factor_add++;
                    }
                }
                
                //why does this make everything work?
                tracklet.markAsAdded(obs_to_add);
            }   
        }
        else {
            // LOG(WARNING) << "tracks dont exist for current frame " << current_frame << " point id " << point_id; 
        }
    }
    static_points_timer.Stop();


    LOG(INFO) << n_landmark_add << " " << n_factor_add;
    LOG(INFO) << "Finished for static";
    timing::Timer dynamic_points_timer("backend/dynamic_points");
    if(current_frame > 0) {
        //will be an issue for in cremental as we only want to add the motion when we add all the points
        for (int j = 1; j < map->vmRigidMotion[current_frame-1].size(); j++) {
            gtsam::Pose3 object_motion = gtsam::Pose3::identity();
            //check if we have a better motion model for this object
            if(current_frame>2) {

                // trace back the previous id in vnRMLabel
                int trace_id = -1;
                for (int k = 0; k < map->vnRMLabel[current_frame-2].size(); ++k)
                {
                    if (map->vnRMLabel[current_frame-2][k]==map->vnRMLabel[current_frame-1][j])
                    {
                        trace_id = k;
                        break;
                    }
                }

                //if trace exists
                // if(trace_id != -1) {
                //     gtsam::Key previous_motion_key = (unique_vertices[current_frame-2][trace_id]);
                
                //     //if key is in the values
                //     if(state_.exists(previous_motion_key)) {
                //         object_motion = isam->calculateEstimate<gtsam::Pose3>((previous_motion_key));
                //         LOG(INFO) << "Using motion prior " << (previous_motion_key) << " " << object_motion;

                //         // //also add a smoothing factor here
                //         gtsam::Pose3 object_motion_smoother = gtsam::Pose3::identity() ;
                //         graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
                //             previous_motion_key,
                //             count_unique_id,
                //             object_motion_smoother,
                //             objectMotionSmootherNoiseModel
                //         );

                //     }

                // }

            }
            //if is first motion
            logObjectMotion(count_unique_id, map->vnRMLabel[current_frame-1][j]);
            addMotionToGraph(object_motion, (count_unique_id), current_frame-1, j);

            unique_vertices[current_frame-1][j]= count_unique_id;
            LOG(INFO) << "Added RM at " << current_frame -1 << " " << j << " " << count_unique_id;
            count_unique_id++;
        }

        for(size_t point_id = 0; point_id < map->vpFeatDyn[current_frame].size(); point_id++) {
            if(dynamic_tracklets.exists(current_frame, point_id)) {
                DynamicTrackletManager::TypedTracklet tracklet = dynamic_tracklets.getTracklet(current_frame, point_id);

                if(tracklet.isWellTracked()) {
                    DynamicTrackletManager::Observations obs_to_add = tracklet.getNotAdded();

                    for(DynamicTrackletManager::Observation obs : obs_to_add) {
                        FrameId frame_id = obs->frame_id;
                        FeatureId feature_id = obs->point_id;
                        int track_id = obs->tracklet_id;
                        int position_id = obs->tracklet_position;
                        int obs_label = map->nObjID[track_id];

                        gtsam::Key pose_key = unique_vertices[frame_id][0];


                        if(position_id == 0) {
                            // LOG(INFO) << obj_position_id << " " << obs_label;
                            gtsam::Point3 X_w = utils::cvMatToGtsamPoint3(map->vp3DPointDyn[frame_id][feature_id]);
                            obs->key = static_cast<int>(count_unique_id);
                            obs->was_added = true;
                            addDynamicLandmarkToGraph(X_w, (count_unique_id), frame_id, feature_id);
                            logObjectMotion(count_unique_id, obs_label);

                            cv::Mat Xc = Optimizer::Get3DinCamera(map->vpFeatDyn[frame_id][feature_id],map->vfDepDyn[frame_id][feature_id],K);
                            gtsam::Point3 X_c_point = utils::cvMatToGtsamPoint3(Xc);
                            // addDynamicPoint3DFactor(X_c_point, curr_camera_pose_vertex, count_unique_id);
                            addDynamicPoint3DFactor(X_c_point, (pose_key), (count_unique_id));
                            // logObjectMotion(count_unique_id, obs_label);
                            //better to use track id as Symbol but then will need to make all variables use differnt symbols
                            vnFeaMakDyn[frame_id][feature_id] = count_unique_id;
                            count_unique_id++;
                        }
                        else {

                            DynamicTrackletManager::Observation previous_obs = tracklet.getPreviousObservation(obs);
                            int previous_key = previous_obs->key;
                            int previous_frame = previous_obs->frame_id;
                            CHECK(previous_frame >= 0);
                            
                            int obj_position_id = -1;
                            // if(frame_id > 0) {
                                //find which obj has the same label the vmLabel and this should be the correct index
                                //ignore index zero as this is camera motion
                            CHECK_EQ(map->vnRMLabel[previous_frame].size(), map->vmRigidMotion[previous_frame].size());
                            for(size_t i = 1; i < map->vnRMLabel[previous_frame].size(); i++) {
                                if(map->vnRMLabel[previous_frame][i] == obs_label) {
                                    obj_position_id = unique_vertices[previous_frame][i];
                                    CHECK(obj_position_id != -1) << "Frame " << previous_frame << " i " << i << " " << obj_position_id;
                                    break;
                                }
                            }
                            // }


                             //save as new vertex
                            gtsam::Point3 X_w = utils::cvMatToGtsamPoint3(map->vp3DPointDyn[frame_id][feature_id]);
                            obs->key = static_cast<int>(count_unique_id);
                            obs->was_added = true;
                            addDynamicLandmarkToGraph(X_w, (count_unique_id), frame_id, feature_id);
                            logObjectMotion(count_unique_id, obs_label);

                            cv::Mat Xc = Optimizer::Get3DinCamera(map->vpFeatDyn[frame_id][feature_id],map->vfDepDyn[frame_id][feature_id],K);
                            gtsam::Point3 X_c_point = utils::cvMatToGtsamPoint3(Xc);
                            // addDynamicPoint3DFactor(X_c_point, curr_camera_pose_vertex, count_unique_id);
                            addDynamicPoint3DFactor(X_c_point, (pose_key), (count_unique_id));
                            // logObjectMotion(count_unique_id, obs_label);
                            //lets do a sanity check
                            //previous obs key has been set
                            CHECK(previous_obs->tracklet_id == track_id);
                            CHECK(previous_obs->was_added);
                            CHECK(previous_key != -1) << "Previous key was " << previous_key;

                            CHECK_EQ(previous_key, vnFeaMakDyn[frame_id-1][previous_obs->point_id]);
                            CHECK_EQ(previous_key, vnFeaMakDyn[DynTracks[track_id][position_id-1].first][DynTracks[track_id][position_id-1].second]);
                            CHECK(obj_position_id != -1) << "Frame " << frame_id - 1 << " pos " << position_id;
                            gtsam::Point3 initial_measurement(0, 0, 0);
                            addLandmarkMotionFactor(initial_measurement, (count_unique_id), (previous_key), (obj_position_id));
                            // logObjectMotion(count_unique_id, obs_label);
                            //better to use track id as Symbol but then will need to make all variables use differnt symbols
                            vnFeaMakDyn[frame_id][feature_id] = count_unique_id;
                            count_unique_id++;
                        }
                    }
                    tracklet.markAsAdded(obs_to_add);
                
                }
            }
        }
    }
    dynamic_points_timer.Stop();
    pre_camera_pose_vertex = curr_camera_pose_vertex;

    if(run_as_incremental) {
        optimize();
    }
    
    // updateMapFromIncremental();

}

void VdoSlamBackend::calculateError() {
    // //first disp comparision to GT
    // //go over all the frames?
    // double t_sum_refined = 0, r_sum_refined  = 0;
    // double t_sum_original = 0, r_sum_original  = 0;
    // const size_t N = getMapSize();
    // for (int i = 0; i < N; ++i) {
    //     //get camera poses
    //     gtsam::Key pose_key = unique_vertices[i][0];
    //     gtsam::Pose3 refined_camera_pose = isam->calculateEstimate<gtsam::Pose3>(pose_key);

    //     gtsam::Pose3 original_camera_pose = utils::cvMatToGtsamPose3(map->vmCameraPose[i]);
    //     gtsam::Pose3 gt_camera_pose = utils::cvMatToGtsamPose3(map->vmCameraPose_GT[i]);

    //     std::pair<double, double> errors_original = utils::computeRotationAndTranslationErrors(
    //                                                     gt_camera_pose, original_camera_pose);

    //     r_sum_original += errors_original.first;
    //     t_sum_original += errors_original.second;

    //     std::pair<double, double> errors_refined = utils::computeRotationAndTranslationErrors(
    //                                                     gt_camera_pose, refined_camera_pose);

    //     r_sum_refined += errors_refined.first;
    //     t_sum_refined += errors_refined.second;
    // }

    // r_sum_original /= (N);
    // t_sum_original /= (N);
    // r_sum_refined /= (N);
    // t_sum_refined /= (N);

    // LOG(INFO) << "Average camera error GTSAM\n"
    //           << "Original R: " << r_sum_original << " T: " << t_sum_original << "\n"
    //           << "Refined R: " << r_sum_refined << " T: " << t_sum_refined;
    result.print("Current estimate: ");

}




gtsam::Values VdoSlamBackend::calculateCurrentEstimate() const {
    return isam->calculateEstimate();

}

const size_t VdoSlamBackend::getMapSize() const { return map->vpFeatSta.size(); }

void VdoSlamBackend::setupNoiseModels() {

    //robust noise model
    //this is on the odom noise model
    gtsam::noiseModel::Base::shared_ptr huberObjectMotion;
    gtsam::noiseModel::Base::shared_ptr huberPoint3D;
    CHECK_NOTNULL(params);
    LOG(INFO) << "Setting up noise models for backend";
    cameraPosePrior = gtsam::noiseModel::Isotropic::Sigma(6u,  params->var_camera_prior);


    point3DNoiseModel = gtsam::noiseModel::Isotropic::Sigma(3u, params->var_3d_static);

    dynamicPoint3DNoiseModel = gtsam::noiseModel::Isotropic::Sigma(3u,  params->var_3d_dyn);
                    // (gtsam::Vector(3) << (gtsam::Vector3::Identity() * params->var_3d_dyn)).finished());

    odometryNoiseModel = gtsam::noiseModel::Isotropic::Sigma(6u,  params->var_camera);
                // (gtsam::Vector(6) << (gtsam::Vector6::Identity() * params->var_camera)).finished());

    objectMotionNoiseModel = gtsam::noiseModel::Isotropic::Sigma(3u, params->var_obj);
                // (gtsam::Vector(3) << (gtsam::Vector3::Identity() * params->var_obj)).finished());

    objectMotionSmootherNoiseModel = gtsam::noiseModel::Isotropic::Sigma(6u, params->var_obj_smooth);
                // (gtsam::Vector(6) << (gtsam::Vector6::Identity() * params->var_obj_smooth)).finished());



    if(params->use_robust_kernel) {
        LOG(INFO) << "Using robust kernal";
        //assuming that this doesnt mess with with original pointer as we're reassigning the member ptrs
        auto pose3dNoiseModelTemp = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(params->k_huber_3d_points), point3DNoiseModel);

        point3DNoiseModel =pose3dNoiseModelTemp;
        //TODO: not using dynamic points yet
        dynamicPoint3DNoiseModel = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(params->k_huber_obj_motion), dynamicPoint3DNoiseModel);
    }


    
}

void VdoSlamBackend::addToKeyVertexMapping(const gtsam::Key& key, FrameId curr_frame, FeatureId feature_id, unsigned char symbol) {
    auto it = key_to_unique_vertices.find(key);
    CHECK(it == key_to_unique_vertices.end()) << "gtsam key is not unique in the key to vertex map";

    IJSymbol ij_symbol;
    ij_symbol.i = curr_frame;
    ij_symbol.j = feature_id;
    ij_symbol.symbol = symbol;
    key_to_unique_vertices.insert({key, ij_symbol});
}

void VdoSlamBackend::addCameraPoseToGraph(const gtsam::Pose3& pose, gtsam::Key key, FrameId curr_frame) {
    all_values.insert(key, pose);
    camera_pose_to_update.emplace(key, std::make_pair(curr_frame, 0));

    // addToKeyVertexMapping(key, curr_frame, 0, kSymbolCameraPose3Key);
}

void VdoSlamBackend::addMotionToGraph(const gtsam::Pose3& motion, gtsam::Key key, FrameId curr_frame, FeatureId object_id) {
    all_values.insert(key, motion);
    object_motions_to_update.emplace(key, std::make_pair(curr_frame, object_id));
    // addToKeyVertexMapping(key, curr_frame, object_id, kSymbolMotion3Key);
}



void VdoSlamBackend::addLandmarkToGraph(const gtsam::Point3& landmark, gtsam::Key key, FrameId curr_frame, FeatureId feature_id) {
    all_values.insert(key, landmark);
    static_points_to_update.emplace(key, std::make_pair(curr_frame, feature_id));
}

void VdoSlamBackend::addDynamicLandmarkToGraph(const gtsam::Point3& landmark, gtsam::Key key, FrameId curr_frame, FeatureId feature_id) {
    all_values.insert(key, landmark);
    dynamic_points_to_update.emplace(key, std::make_pair(curr_frame, feature_id));
    // addToKeyVertexMapping(key, curr_frame, feature_id, kSymbolDynamicPoint3Key);
}

// void VdoSlamBackend::addPoint2DFactor(const gtsam::Point2& measurement, gtsam::Key pose_key, gtsam::Key landmark_key) {
//     static gtsam::noiseModel::Base::shared_ptr point2DNoiseModel = gtsam::noiseModel::Diagonal::Sigmas(
//                     (gtsam::Vector(2) << gtsam::Vector2::Constant(params->var_3d_static)).finished());
//     // noiseModel::Isotropic::shared_ptr point2DNoiseModel = noiseModel::Isotropic::Sigma(2, 1.0);
//     graph.push_back(gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>(
//         measurement,
//         point2DNoiseModel,
//         pose_key,
//         landmark_key,
//         K_calib
//     ));
// }


void VdoSlamBackend::addPoint3DFactor(const gtsam::Point3& measurement, gtsam::Key pose_key, gtsam::Key landmark_key) {


    graph.emplace_shared<Point3DFactor>(
        pose_key,
        landmark_key,
        measurement,
        point3DNoiseModel
    );

}

void VdoSlamBackend::addDynamicPoint3DFactor(const gtsam::Point3& measurement, gtsam::Key pose_key, gtsam::Key landmark_key) {
    // CHECK(new_dynamic_lmks.exists(landmark_key) || isam->valueExists(landmark_key)) << "Dyanmic point landmark must be added";
    // observed_dyn_landmarks[landmark_key].push_back(
    //     boost::make_shared<Point3DFactor>(
    //         pose_key,
    //         landmark_key,
    //         measurement,
    //         dynamicPoint3DNoiseModel
    //     )
    // );

    graph.emplace_shared<Point3DFactor>(
        pose_key,
        landmark_key,
        measurement,
        dynamicPoint3DNoiseModel
    );
}


void VdoSlamBackend::addLandmarkMotionFactor(const gtsam::Point3& measurement, gtsam::Key current_point_key, 
            gtsam::Key previous_point_key, gtsam::Key motion_key) {

    graph.emplace_shared<LandmarkMotionTernaryFactor>(
            previous_point_key,
            current_point_key,
            motion_key,
            measurement,
            objectMotionNoiseModel
    );


}

void VdoSlamBackend::updateMap(const gtsam::Values& state) {
    LOG(INFO) << "Updating " << camera_pose_to_update.size() << " Camera poses";

    //update camera pose
    for (const auto& [key, frame_slot]: camera_pose_to_update ) {
        if(!state.exists(key)) {
            LOG(WARNING) << "pose key " << key << " should exist in the state";
            continue;
        }

        const auto& frame_id = frame_slot.first;
        gtsam::Pose3 camea_pose_refined = state.at<gtsam::Pose3>(key);
        cv::Mat cv_pose_refined = utils::gtsamPose3ToCvMat(camea_pose_refined);
        map->vmCameraPose[frame_id] = cv_pose_refined;

        //now update camera motion
        if (frame_id > 0) {
            gtsam::Pose3 camera_pose_prev = utils::cvMatToGtsamPose3(map->vmCameraPose[frame_id-1]);
            gtsam::Pose3 rigid_motion_refied = camera_pose_prev.inverse() * camea_pose_refined;
            map->vmRigidMotion[frame_id-1][0] = utils::gtsamPose3ToCvMat(rigid_motion_refied);
        }
    }

    LOG(INFO) << "Updating " << object_motions_to_update.size() << " Object motions";
    //update motion
    for (const auto& [key, frame_slot]: object_motions_to_update ) {
        if(!state.exists(key)) {
            LOG(WARNING) << "motion key " << key << " should exist in the state";
            continue;
        }

        gtsam::Pose3 object_motion = state.at<gtsam::Pose3>(key);
        cv::Mat object_motion_refined = utils::gtsamPose3ToCvMat(object_motion);
        map->vmRigidMotion[frame_slot.first][frame_slot.second] = object_motion_refined;
    }

    //update static points
    LOG(INFO) << "Updating " << static_points_to_update.size() << " Static Points";

    for (const auto& [key, frame_slot]: static_points_to_update ) {
        if(!state.exists(key)) {
            LOG(WARNING) << "static point key " << key << " should exist in the state";
            continue;
        }

        gtsam::Point3 static_point = state.at<gtsam::Point3>(key);
        cv::Mat static_point_refined = utils::gtsamPoint3ToCvMat(static_point);
        map->vp3DPointSta[frame_slot.first][frame_slot.second] = static_point_refined;
    }

    //update dynamic points
     LOG(INFO) << "Updating " << dynamic_points_to_update.size() << " Dynamic Points";
    for (const auto& [key, frame_slot]: dynamic_points_to_update ) {
        if(!state.exists(key)) {
            LOG(WARNING) << "dynamic point key " << key << " should exist in the state";
            continue;
        }

        gtsam::Point3 dynamic_point = state.at<gtsam::Point3>(key);
        cv::Mat dynamic_point_refined = utils::gtsamPoint3ToCvMat(dynamic_point);
        // LOG(INFO) << dynamic_point_refined;
        //TODO: causes cfree deallocate in cv::Mat
        map->vp3DPointDyn[frame_slot.first][frame_slot.second] = dynamic_point_refined;
    }

    //clear all maps
    // camera_pose_to_update.clear();
    // object_motions_to_update.clear();
    // static_points_to_update.clear();
    // dynamic_points_to_update.clear();

    camera_pose_to_update = std::map<gtsam::Key, FrameSlot>();
    object_motions_to_update = std::map<gtsam::Key, FrameSlot>();
    static_points_to_update = std::map<gtsam::Key, FrameSlot>();
    dynamic_points_to_update = std::map<gtsam::Key, FrameSlot>();

    LOG(INFO) << "Done update";
    // //first update camera motion
    // gtsam::Pose3 camea_pose_refined = isam->calculateEstimate<gtsam::Pose3>(curr_camera_pose_vertex);
    // cv::Mat cv_pose_refined = utils::gtsamPose3ToCvMat(camea_pose_refined);
    // map->vmCameraPose[current_frame] = cv_pose_refined;
    // // map->vmCameraPose_RF[frame_id] = cv_pose_refined;

    // //update camera motion
    // if(current_frame > 0) {
    //      gtsam::Pose3 camera_pose_prev = utils::cvMatToGtsamPose3(map->vmCameraPose[current_frame-1]);
    //         gtsam::Pose3 rigid_motion_refied = camera_pose_prev.inverse() * camea_pose_refined;
    //         //     map->vmRigidMotion[frame_id-1][0] = Converter::toInvMatrix(
    //         //         map->vmCameraPose[frame_id-1])*map->vmCameraPose[frame_id];
    //         map->vmRigidMotion[current_frame-1][0] = utils::gtsamPose3ToCvMat(rigid_motion_refied);
    // }
    // // for()
    // //TODO: clear keys to update
}

void VdoSlamBackend::optimizeLM() {
     LOG(INFO) << "Num gtsam vertex " << all_values.size();
    LOG(INFO) << "Num gtsam edges " << graph.size();
    try {
        // boost::shared_ptr<gtsam::GaussianFactorGraph> linearization = graph.linearize(all_values);
        // gtsam::Matrix hessian = linearization->augmentedHessian();
        // gtsam::Matrix jacobian = linearization->augmentedJacobian();

        // std::ofstream file("hessian.txt");
        // if (file.is_open()) {
        //     file << hessian;
        //     file.close();
        // }

        LOG(INFO) << "Begin LM OPT";
        gtsam::LevenbergMarquardtParams params;
        params.verbosityLM = gtsam::LevenbergMarquardtParams::verbosityLMTranslator("TRYLAMBDA");
        gtsam::LevenbergMarquardtOptimizer lm_optimizer(graph, all_values, params);
        LOG(INFO) << "LM error before " << lm_optimizer.error();
        Values result = lm_optimizer.optimize();
        LOG(INFO) << "Done LM OPT";
        LOG(INFO) << "LM error " << lm_optimizer.error();
        LOG(INFO) << "LM iterations " << lm_optimizer.iterations();

        updateMap(result);
    }
    catch (const gtsam::ValuesKeyDoesNotExist& e) {
            LOG(WARNING) << e.what();
            auto it = key_to_unique_vertices.find(e.key());
            if (it == key_to_unique_vertices.end()) {
                LOG(INFO) << "key was also not in key vertex mapping";
            }
            else {
                LOG(INFO) << key_to_unique_vertices[e.key()];
            }
            throw e;

        }
        catch(const gtsam::IndeterminantLinearSystemException& e) {
            LOG(WARNING) << e.what();
            auto it = key_to_unique_vertices.find(e.nearbyVariable());
            if (it == key_to_unique_vertices.end()) {
                LOG(INFO) << "key was also not in key vertex mapping";
            }
            else {
                LOG(INFO) << key_to_unique_vertices[e.nearbyVariable()];
            }
            throw e;
        }
}

void VdoSlamBackend::makePlots() {
    // const std::string save_root_path = "/root/data/vdo_slam/results/";
    // std::vector<double> x;
    // for(int i = 0; i < getMapSize(); i++) {
    //     x.push_back(i+1);
    // }
    Plotter::makePlots();
    Plotter::drawDynamicSize(dynamic_motion_map_total);

    isam->saveGraph("/root/data/vdo_slam/results/isam2.dot");


    // // // plt::figure(1);
    // plt::title("Dyanamic Object tracksd");
    // plt::xlabel("n frames");
    // plt::ylabel("Total number of vars");

    // for(const auto& e : dynamic_motion_map_total) {
    //     int total = 0;
    //     std::vector<int> num_vars;
    //     for(const auto& f : e.second) {
    //         total += f.size();
    //         num_vars.push_back(total);
    //     }

    //     plt::named_plot("Obj: " + std::to_string(e.first), x, num_vars);        
    // }


    // // Enable legend.
    // plt::legend();

    // plt::save(save_root_path + "object_variables.png");
   
    // plt::figure(2);
    // plt::title("Error after optimization");
    // plt::xlabel("n frames");
    // plt::ylabel("log-likelihood error");
    // plt::plot(x, error_after_v);
    // plt::save(save_root_path + "error_after_opt.png");

    // plt::figure(3);
    // plt::title("Time for each update step of iSAM2");
    // plt::xlabel("n frames");
    // plt::ylabel("time (s)");
    // plt::plot(x, time_to_optimize);
    // plt::save(save_root_path + "time_to_optimize.png");

    // plt::figure(4);
    // plt::title("Number of factors in the iSAM2 factor graph");
    // plt::xlabel("n frames");
    // plt::ylabel("n factors");
    // plt::plot(x, no_factors);
    // plt::save(save_root_path + "no_factors.png");

    // plt::figure(5);
    // plt::title("Number of variables in the iSAM2 factor graph");
    // plt::xlabel("n frames");
    // plt::ylabel("n variables");
    // plt::plot(x, no_variables);
    // plt::save(save_root_path + "no_variables.png");

    




}

cv::Mat VdoSlamBackend::getBestPoseEstimate() {
    gtsam::Key pose_key = unique_vertices[current_frame][0];
    gtsam::Pose3 pose = state_.at<gtsam::Pose3>((pose_key));
    return utils::gtsamPose3ToCvMat(pose.inverse());
}

void VdoSlamBackend::optimize() {
    ///failure is always on previous camera pose
    if(current_frame > 0) {
        //will throw KeyAlreadyExists<J> so a good test to see if the front end -> backend processing is working
        try {

            //attempt to marginalize out dynamic poins and motion... should maybe wait till i have good graphs but whatbeer
            //shoudl be done before updateing map which cleanrs these std::maps
            KeyTimestampMap timestamps;
            

            // for (const auto& [key, frame_slot]: camera_pose_to_update ) {
            //     timestamps[key] = current_frame;
            // }

            // for (const auto& [key, frame_slot]: object_motions_to_update ) {
            //     timestamps[key] = current_frame;
            // }

            // for (const auto& [key, frame_slot]: static_points_to_update ) {
            //     timestamps[key] = current_frame;
            // }

            // for (const auto& [key, frame_slot]: dynamic_points_to_update ) {
            //     timestamps[key] = current_frame;
            // }

            timing::Timer isam_udpate_timer("backend/isam2_update");
            auto start = high_resolution_clock::now();
            result = isam->update(graph, all_values);

            // gtsam::IncrementalFixedLagSmoother::Result fl_result = isam->update(graph, all_values, timestamps);
            auto stop = high_resolution_clock::now();

            // result = isam->getISAM2Result();

            auto duration = duration_cast<milliseconds>(stop - start);
            // result = isam->update();
            // result = isam->update();
            // result = isam->update();
            isam_udpate_timer.Stop();

            // gtsam::NonlinearFactorGraph graph_ = gtsam::NonlinearFactorGraph(isam->getFactors());  // clone, expensive but safer!
            gtsam::NonlinearFactorGraph graph_ = gtsam::NonlinearFactorGraph(isam->getFactorsUnsafe());  // clone, expensive but safer!
            if(state_.size() > 0) {
                LOG(INFO) << "Size of current graph " << graph_.size();
                LOG(INFO) << "Size of current state " << state_.size();
                gtsam::Values state_before_opt = gtsam::Values(state_);
                BOOST_FOREACH (const gtsam::Values::ConstKeyValuePair& key_value,
                   all_values) {
                state_before_opt.insert(key_value.key, key_value.value);
                }

                double error_before = graph_.error(state_before_opt);
                state_ = isam->calculateEstimate();
                updateMap(state_);
                double error_after = graph.error(state_);
                LOG(INFO) << "Optimization Errors:\n"
                        << "Error before: " << error_before
                        << "\n"
                        << "Error after: " << error_after;
                double change = error_before - error_after;

                Plotter::appendData("Error before optimization", error_before);
                Plotter::appendData("Error after optimization", error_after);
                // // change_error.push_back(change);
                // error_before_v.push_back(error_before);
                // error_after_v.push_back(error_after);
            }
            else {
                state_ = isam->calculateEstimate();
                // error_before_v.push_back(0);
                // error_after_v.push_back(0);
                Plotter::appendData("Error before optimization", 0);
                Plotter::appendData("Error after optimization", 0);
            }

            Plotter::appendData("Number of factors in the iSAM2 factor graph", static_cast<double>(graph_.size()));
            Plotter::appendData("Number of variables in the system", static_cast<double>(state_.size()));
            Plotter::appendData("Time for each update step of iSAM2", duration.count());
            // no_factors.push_back(graph_.size());
            // no_variables.push_back(state_.size());
            // time_to_optimize.push_back(duration.count());

            double num_cliques = static_cast<double>(isam->size());
            LOG(INFO) << "# cliques " << num_cliques;
            Plotter::appendData("# of Cliques", num_cliques);

            double num_roots = static_cast<double>(isam->roots().size());
            LOG(INFO) << "# roots " << num_roots;

            //find maximum clique
            int max_clique_size = 0;
            const auto& nodes = isam->nodes();
            //each shared clique is a ISAM2Clique
            for(const auto& e : nodes) {
                const ISAM2Clique::shared_ptr& clique = e.second;
                double tree_size = static_cast<double>(clique->treeSize());
                

                //ignore root node?
                if(tree_size > max_clique_size && !clique->isRoot()) {
                    max_clique_size = tree_size;
                }
            }

            for(const auto& e : dynamic_motion_map) {
                LOG(INFO) << "Vars for object " << e.first << " - size " << e.second.size();
            }

            LOG(INFO) << "Max Clique Size " << max_clique_size;
            Plotter::appendData("Max Clique Size", static_cast<double>(max_clique_size));

            isam->printStats();


            //updayte the dynamic motion total map by adding all the newly added keys
            //and then clearing the map so its fresh for the next update
            for(auto& e : dynamic_motion_map) {
                if(dynamic_motion_map_total.find(e.first) == dynamic_motion_map_total.end()) {
                    dynamic_motion_map_total[e.first]= std::vector<std::vector<gtsam::Key>>();
                }
                
                dynamic_motion_map_total[e.first].push_back(e.second);
                e.second = std::vector<gtsam::Key>();
            }
            
            all_values.clear();
            graph.resize(0);

            // debug_info.print();
            debug_info.reset();
            // updateMapFromIncremental();
        }
        catch (const gtsam::ValuesKeyDoesNotExist& e)
        {
            LOG(ERROR) << e.what();
            const gtsam::Key& var = e.key();
            gtsam::Symbol symb(var);

            LOG(ERROR) << "ERROR: Variable has type '" << symb.chr() << "' "
                    << "and index " << symb.index() << std::endl;
            throw e;
        }
        catch (const gtsam::IndeterminantLinearSystemException& e)
        {
            LOG(WARNING) << e.what();
            const gtsam::Key& var = e.nearbyVariable();
            gtsam::Symbol symb(var);

            LOG(ERROR) << "ERROR: Variable has type '" << symb.chr() << "' "
                    << "and index " << symb.index() << std::endl;
            throw e;
            
        }
        
    }
    

}

void VdoSlamBackend::logObjectMotion(gtsam::Key key, int object_label) {
    if(dynamic_motion_map.find(object_label)== dynamic_motion_map.end()) {
        dynamic_motion_map[object_label] = std::vector<gtsam::Key>();
    }

    dynamic_motion_map[object_label].push_back(key);
}

void VdoSlamBackend::writeG2o(const std::string& file_name) {
    VDO_SLAM::utils::writeG2o(graph, all_values, file_name);
}

// void VdoSlamBackend::updateMapFromSymbol(const gtsam::Key& key, const IJSymbol& vertex_symbol) {
//     const int frame_id = vertex_symbol.i;
//     const int feature_id = vertex_symbol.j;


//     //we have updated a pose
//     if(vertex_symbol.symbol == kSymbolCameraPose3Key) {
//         gtsam::Pose3 camea_pose_refined = isam->calculateEstimate<gtsam::Pose3>(key);
//         map->vmCameraPose[frame_id] = utils::gtsamPose3ToCvMat(camea_pose_refined);
//         // num_poses_update++;
        
//         if(frame_id > 0) {

//             // CHECK_MAT_TYPES(Converter::toInvMatrix(
//             //     map->vmCameraPose[frame_id-1]), map->vmCameraPose[frame_id]);
//             // gtsam::Pose3 motion_refined = camea_pose_refined.inverse() * camea_pose_refined;

//             gtsam::Pose3 camera_pose_prev = utils::cvMatToGtsamPose3(map->vmCameraPose[frame_id-1]);
//             gtsam::Pose3 rigid_motion_refied = camera_pose_prev.inverse() * camea_pose_refined;
//             //     map->vmRigidMotion[frame_id-1][0] = Converter::toInvMatrix(
//             //         map->vmCameraPose[frame_id-1])*map->vmCameraPose[frame_id];
//             map->vmRigidMotion[frame_id-1][0] = utils::gtsamPose3ToCvMat(rigid_motion_refied);
//         }
//     }

//     if(vertex_symbol.symbol == kSymbolPoint3Key) {
//         // num_points_updated++;
//         gtsam::Point3 point = isam->calculateEstimate<gtsam::Point3>(key);
//         if(vnFeaMakSta[frame_id][feature_id] != -1) {
//             map->vp3DPointSta[frame_id][feature_id] = utils::gtsamPoint3ToCvMat(point);
//             // num_points_updated++;
//         }
//         else {
//             //log warning?
//         }
//     }
// }

}